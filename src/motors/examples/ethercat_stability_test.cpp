// Standalone 1 kHz, twelve-motor PVT hold test. No policy, IMU or Torch.
#include "driver/myact/myact_motor_controller.hpp"
#include "driver/myact/motor_units.hpp"
#include "protocol/ethercat/ethercat_adapter_igh.hpp"
#include "tool/tool.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
namespace mb = motor_base;
constexpr std::size_t kMotors = 12;
static_assert(myactua::kNumSlaves == kMotors, "This test requires all 12 motor slaves");
constexpr std::int64_t kPeriodNs = 1'000'000;
constexpr std::int64_t kCommandValidityNs = 10'000'000;
constexpr double kKp = 180.0;
constexpr double kKd = 10.0;
constexpr std::size_t kQueueCapacity = 8192;
volatile std::sig_atomic_t g_interrupt = 0;

void signal_handler(int) { g_interrupt = 1; }

enum class Phase : unsigned { Startup, Hold, Stopping };
enum class StopReason : unsigned {
    None, Completed, Interrupted, StartupFailed, TerminalFault,
    CommandRejected, LogOverflow, LogWriteFailed
};
const char* reason_name(StopReason reason)
{
    switch (reason) {
        case StopReason::None: return "running";
        case StopReason::Completed: return "completed";
        case StopReason::Interrupted: return "interrupted";
        case StopReason::StartupFailed: return "startup_failed";
        case StopReason::TerminalFault: return "terminal_fault";
        case StopReason::CommandRejected: return "command_rejected";
        case StopReason::LogOverflow: return "log_overflow";
        case StopReason::LogWriteFailed: return "log_write_failed";
    }
    return "unknown";
}

enum Anomaly : std::uint32_t {
    LinkDown = 1U << 0, WkcBad = 1U << 1, SlaveCountChanged = 1U << 2,
    MotorNotReady = 1U << 3, InvalidPdo = 1U << 4, DriveFault = 1U << 5,
    Disabled = 1U << 6, ModeChanged = 1U << 7, TerminalFault = 1U << 8,
    CommandRejected = 1U << 9, WorkOverrun = 1U << 10,
    CycleGap = 1U << 11, LogOverflow = 1U << 12
};
constexpr std::uint32_t kCommunicationMask =
    LinkDown | WkcBad | SlaveCountChanged | MotorNotReady | InvalidPdo;
constexpr std::uint32_t kControlMask =
    DriveFault | Disabled | ModeChanged | TerminalFault | CommandRejected;
constexpr std::uint32_t kTimingMask = WorkOverrun | CycleGap;

double position_scale(std::size_t i)
{
    return (i == 4 || i == 5 || i == 10 || i == 11)
        ? myactua::kAnkleRawPosToRad : myactua::kRawPosToRad;
}
double target_scale(std::size_t i)
{
    return (i == 4 || i == 5 || i == 10 || i == 11)
        ? myactua::kAnkleRadToRawPos : myactua::kRadToRawPos;
}

struct MotorSample {
    ec_slave_config_state_t slave{};
    myactua::RxPDO raw{};        // Domain bytes, possibly stale/partial.
    myactua::RxPDO last_valid{}; // Updated under the same validity rule as the driver.
    myactua::TxPDO tx{};
    std::int64_t last_valid_ns{0};
    bool pdo_valid{false};
};
struct CycleSample {
    std::uint64_t index{0};
    std::int64_t timestamp_ns{0};
    std::int64_t end_ns{0};
    std::int64_t interval_ns{0};
    std::int64_t work_ns{0};
    std::int64_t elapsed_ns{0};
    Phase phase{Phase::Startup};
    myactua::EthercatBusHealthSnapshot bus{};
    unsigned ready_motors{0};
    std::uint32_t anomalies{0};
    std::array<MotorSample, kMotors> motors{};
};

std::uint32_t classify(const CycleSample& s, unsigned expected_wkc,
                       unsigned expected_slaves)
{
    std::uint32_t flags = s.anomalies;
    if (!s.bus.master_link_up) flags |= LinkDown;
    if (s.bus.wc_state != EC_WC_COMPLETE ||
        s.bus.working_counter != expected_wkc) flags |= WkcBad;
    if (s.bus.slaves_responding != expected_slaves) flags |= SlaveCountChanged;
    if (s.ready_motors != kMotors) flags |= MotorNotReady;
    for (const auto& motor : s.motors) {
        if (!motor.pdo_valid) {
            flags |= InvalidPdo;
            continue; // Never interpret invalid PDO as a new drive fault.
        }
        if (myactua::is_fault(motor.raw.status_word) || motor.raw.error != 0)
            flags |= DriveFault;
        if (s.phase == Phase::Hold) {
            if (!myactua::is_operation_enabled(motor.raw.status_word))
                flags |= Disabled;
            if (motor.raw.op_mode != static_cast<int8_t>(myactua::MyactControlMode::PVT))
                flags |= ModeChanged;
        }
    }
    if (s.work_ns >= kPeriodNs) flags |= WorkOverrun;
    if (s.interval_ns >= 2 * kPeriodNs) flags |= CycleGap;
    return flags;
}

struct Metric {
    std::uint64_t count{0};
    std::int64_t min{std::numeric_limits<std::int64_t>::max()}, max{0};
    long double sum{0};
    void add(std::int64_t value)
    {
        ++count;
        min = std::min(min, value);
        max = std::max(max, value);
        sum += value;
    }
    void json(std::ostream& out) const
    {
        out << "{\"samples\":" << count << ",\"min_us\":"
            << (count ? min / 1000.0 : 0.0) << ",\"mean_us\":"
            << (count ? static_cast<double>(sum / count / 1000.0L) : 0.0)
            << ",\"max_us\":" << max / 1000.0 << '}';
    }
};

struct Statistics {
    std::uint64_t cycles{0}, hold_cycles{0}, anomaly_cycles{0};
    std::uint64_t communication_bad_cycles{0}, control_fault_cycles{0};
    std::uint64_t timing_anomaly_cycles{0}, wc_zero_cycles{0}, wc_incomplete_cycles{0};
    std::uint64_t wkc_changed_cycles{0}, slave_count_changed_cycles{0};
    std::uint64_t motor_not_ready_cycles{0}, invalid_pdo_cycles{0};
    std::uint64_t work_overruns{0}, cycle_gaps{0};
    std::int64_t first_anomaly_ns{0}, first_communication_anomaly_ns{0};
    std::int64_t first_timing_anomaly_ns{0};
    Metric interval, work;

    void add(const CycleSample& s, unsigned expected_wkc)
    {
        ++cycles;
        if (s.anomalies && first_anomaly_ns == 0) first_anomaly_ns = s.timestamp_ns;
        if (s.phase != Phase::Hold) return;
        ++hold_cycles;
        if (s.interval_ns > 0) interval.add(s.interval_ns);
        work.add(s.work_ns);
        anomaly_cycles += s.anomalies != 0;
        communication_bad_cycles += (s.anomalies & kCommunicationMask) != 0;
        control_fault_cycles += (s.anomalies & kControlMask) != 0;
        timing_anomaly_cycles += (s.anomalies & kTimingMask) != 0;
        wc_zero_cycles += s.bus.wc_state == EC_WC_ZERO;
        wc_incomplete_cycles += s.bus.wc_state == EC_WC_INCOMPLETE;
        wkc_changed_cycles += s.bus.working_counter != expected_wkc;
        slave_count_changed_cycles += (s.anomalies & SlaveCountChanged) != 0;
        motor_not_ready_cycles += (s.anomalies & MotorNotReady) != 0;
        invalid_pdo_cycles += (s.anomalies & InvalidPdo) != 0;
        work_overruns += (s.anomalies & WorkOverrun) != 0;
        cycle_gaps += (s.anomalies & CycleGap) != 0;
        if ((s.anomalies & kCommunicationMask) && !first_communication_anomaly_ns)
            first_communication_anomaly_ns = s.timestamp_ns;
        if ((s.anomalies & kTimingMask) && !first_timing_anomaly_ns)
            first_timing_anomaly_ns = s.timestamp_ns;
    }
};

void write_headers(std::ostream& cycles, std::ostream& events)
{
    cycles << "cycle_index,timestamp_monotonic_ns,end_monotonic_ns,phase,elapsed_s,"
              "wc_state,working_counter,link_up,slaves_responding,master_al_states,"
              "ready_motors,cycle_interval_us,pdo_work_us,anomaly_mask\n";
    events << "cycle_index,timestamp_monotonic_ns,phase,anomaly_mask,motor_id,"
              "ethercat_position,online,operational,al_state,pdo_valid,"
              "last_valid_monotonic_ns,feedback_age_us,raw_status_word,raw_error_code,"
              "raw_mode,raw_position,raw_velocity,raw_torque,last_valid_status_word,"
              "last_valid_error_code,last_valid_mode,position_rad,velocity_rad_s,"
              "torque_percent,tx_control_word,tx_mode,tx_target_position_raw,"
              "tx_target_velocity_raw,tx_target_torque_raw,tx_kp_raw,tx_kd_raw\n";
}

void write_sample(std::ostream& cycles, std::ostream& events, const CycleSample& s)
{
    cycles << s.index << ',' << s.timestamp_ns << ',' << s.end_ns << ','
           << static_cast<unsigned>(s.phase) << ',' << s.elapsed_ns / 1e9 << ','
           << static_cast<unsigned>(s.bus.wc_state) << ',' << s.bus.working_counter << ','
           << s.bus.master_link_up << ',' << s.bus.slaves_responding << ','
           << s.bus.al_states << ',' << s.ready_motors << ','
           << s.interval_ns / 1000.0 << ',' << s.work_ns / 1000.0 << ','
           << s.anomalies << '\n';
    if (!s.anomalies) return;
    for (std::size_t i = 0; i < kMotors; ++i) {
        const auto& m = s.motors[i];
        const double scale = position_scale(i);
        events << s.index << ',' << s.timestamp_ns << ','
               << static_cast<unsigned>(s.phase) << ',' << s.anomalies << ',' << i << ','
               << myactua::kSlavePositions[i] << ',' << m.slave.online << ','
               << m.slave.operational << ',' << m.slave.al_state << ',' << m.pdo_valid << ','
               << m.last_valid_ns << ',';
        if (m.last_valid_ns) events << (s.timestamp_ns - m.last_valid_ns) / 1000.0;
        else events << "nan";
        events << ',' << m.raw.status_word << ',' << m.raw.error << ','
               << static_cast<int>(m.raw.op_mode) << ',' << m.raw.pos << ',' << m.raw.vel
               << ',' << m.raw.torque << ',' << m.last_valid.status_word << ','
               << m.last_valid.error << ',' << static_cast<int>(m.last_valid.op_mode) << ',';
        if (m.last_valid_ns)
            events << m.last_valid.pos * scale << ',' << m.last_valid.vel * scale << ','
                   << m.last_valid.torque * myactua::kRawTorqueToPercent;
        else events << "nan,nan,nan";
        events << ',' << m.tx.control_word << ',' << static_cast<int>(m.tx.op_mode)
               << ',' << m.tx.target_pos << ',' << m.tx.target_vel << ','
               << m.tx.target_torque << ',' << m.tx.pvt_kp << ',' << m.tx.pvt_kd << '\n';
    }
}

struct TestRun {
    explicit TestRun(std::int64_t duration, std::size_t capacity = kQueueCapacity)
        : duration_ns(duration), buffer(capacity)
    {
        // All storage is allocated and touched before the RT thread starts.
        volatile unsigned char* pages = reinterpret_cast<volatile unsigned char*>(buffer.data());
        for (std::size_t i = 0; i < buffer.size() * sizeof(CycleSample); i += 4096)
            pages[i] = 0;
        anchor_monotonic_ns = robot_base::monotonic_now_ns();
        anchor_unix_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }

    const std::int64_t duration_ns;
    std::int64_t anchor_monotonic_ns{0}, anchor_unix_ns{0};
    unsigned expected_wkc{0}, expected_slaves{0}; // Set before RT startup.
    std::atomic<bool> active{false}, armed{false}, started{false}, stopping{false};
    std::atomic<StopReason> reason{StopReason::None};
    std::atomic<bool> producer_done{false}, write_failed{false};
    std::vector<CycleSample> buffer;
    std::atomic<std::uint64_t> head{0}, tail{0};
    std::uint64_t max_queue_depth{0}, dropped_cycles{0};
    bool overflowed{false};
    CycleSample overflow_sample{}; // First rejected sample survives a full queue.
    CycleSample current{};
    std::array<myactua::RxPDO, kMotors> last_valid{};
    std::array<std::int64_t, kMotors> last_valid_ns{};
    std::array<myactua::TxPDO, kMotors> latest_tx{};
    std::array<mb::ImpedanceSetpoint, kMotors> hold{};
    std::array<std::int32_t, kMotors> hold_raw{};
    bool hold_captured{false}, hold_sent{false};
    std::int64_t previous_start_ns{0}, run_start_ns{0}, run_end_ns{0};
    std::uint64_t next_index{0};
    Statistics stats; // Logger thread only; read after join.
    std::ofstream cycles_file, events_file;
    std::thread writer;

    void finish(StopReason value) noexcept
    {
        StopReason expected = StopReason::None;
        reason.compare_exchange_strong(expected, value, std::memory_order_acq_rel);
    }

    bool push(const CycleSample& sample) noexcept
    {
        if (overflowed) {
            ++dropped_cycles;
            return false;
        }
        const auto h = head.load(std::memory_order_relaxed);
        const auto t = tail.load(std::memory_order_acquire);
        if (h - t == buffer.size()) {
            overflow_sample = sample;
            overflow_sample.anomalies |= LogOverflow;
            overflowed = true;
            ++dropped_cycles;
            finish(StopReason::LogOverflow);
            return false;
        }
        buffer[h % buffer.size()] = sample;
        max_queue_depth = std::max(max_queue_depth, h - t + 1);
        head.store(h + 1, std::memory_order_release);
        return true;
    }

    bool pop(CycleSample& sample) noexcept
    {
        const auto t = tail.load(std::memory_order_relaxed);
        if (t == head.load(std::memory_order_acquire)) return false;
        sample = buffer[t % buffer.size()];
        tail.store(t + 1, std::memory_order_release);
        return true;
    }

    void begin(std::int64_t now) noexcept
    {
        current = {};
        current.index = next_index++;
        current.timestamp_ns = now;
        current.interval_ns = previous_start_ns ? now - previous_start_ns : 0;
        previous_start_ns = now;
        current.phase = stopping.load(std::memory_order_acquire) ? Phase::Stopping
            : (started.load(std::memory_order_relaxed) ? Phase::Hold : Phase::Startup);
    }

    void observe_receive(const myactua::EthercatBusHealthSnapshot& bus,
                         const std::array<ec_slave_config_state_t, kMotors>& slaves,
                         const std::array<myactua::RxPDO, kMotors>& rx) noexcept
    {
        current.bus = bus;
        for (std::size_t i = 0; i < kMotors; ++i) {
            auto& motor = current.motors[i];
            motor.slave = slaves[i];
            motor.raw = rx[i];
            const bool configured = slaves[i].online && slaves[i].operational;
            current.ready_motors += configured;
            motor.pdo_valid = bus.master_link_up && bus.wc_state == EC_WC_COMPLETE &&
                              configured;
            if (motor.pdo_valid) {
                last_valid[i] = rx[i];
                last_valid_ns[i] = current.timestamp_ns;
            }
            motor.last_valid = last_valid[i];
            motor.last_valid_ns = last_valid_ns[i];
        }
    }

    bool feedback_can_hold() const noexcept
    {
        for (const auto& motor : current.motors) {
            if (!motor.pdo_valid || !myactua::is_operation_enabled(motor.raw.status_word) ||
                myactua::is_fault(motor.raw.status_word) || motor.raw.error != 0 ||
                motor.raw.op_mode != static_cast<int8_t>(myactua::MyactControlMode::PVT))
                return false;
        }
        return true;
    }

    void generate_hold(myactua::MyActMotorController& controller) noexcept
    {
        if (!armed.load(std::memory_order_acquire) ||
            stopping.load(std::memory_order_acquire) ||
            reason.load(std::memory_order_acquire) != StopReason::None ||
            controller.terminal_fault_latched() || !feedback_can_hold()) return;
        if (!hold_captured) {
            for (std::size_t i = 0; i < kMotors; ++i) {
                hold[i] = mb::ImpedanceSetpoint(
                    current.motors[i].raw.pos * position_scale(i), 0.0, 0.0, kKp, kKd);
                hold_raw[i] = robot_base::double_to_i32(hold[i].position_rad * target_scale(i));
            }
            hold_captured = true;
        }
        auto command = mb::ControlCommand::set_impedance_targets_fixed(hold.data(), hold.size());
        const auto now = robot_base::monotonic_now_ns();
        command.timing.produced_at_ns = now;
        command.timing.valid_until_ns = now + kCommandValidityNs;
        // This is the sole producer. The base controller consumes it NEXT cycle.
        if (controller.send_policy_setpoint(command).status != mb::CommandSubmitStatus::ACCEPTED)
            current.anomalies |= CommandRejected;
    }

    bool hold_applied() const noexcept
    {
        if (!hold_captured || !feedback_can_hold()) return false;
        for (std::size_t i = 0; i < kMotors; ++i) {
            const auto& tx = latest_tx[i];
            if (tx.control_word != myactua::CMD_ENABLE_OPERATION ||
                tx.op_mode != static_cast<int8_t>(myactua::MyactControlMode::PVT) ||
                tx.target_pos != hold_raw[i] || tx.target_vel != 0 || tx.target_torque != 0 ||
                tx.pvt_kp != 180'000 || tx.pvt_kd != 10'000) return false;
        }
        return true;
    }

    void end(std::int64_t now, bool terminal_fault) noexcept
    {
        current.end_ns = now;
        current.work_ns = now - current.timestamp_ns;
        for (std::size_t i = 0; i < kMotors; ++i) current.motors[i].tx = latest_tx[i];
        if (!started.load(std::memory_order_relaxed) &&
            armed.load(std::memory_order_acquire) &&
            reason.load(std::memory_order_acquire) == StopReason::None &&
            !stopping.load(std::memory_order_acquire) && !terminal_fault && hold_applied()) {
            // Require valid feedback after the first matching outgoing hold frame.
            if (hold_sent) {
                run_start_ns = current.timestamp_ns;
                current.phase = Phase::Hold;
                started.store(true, std::memory_order_release);
            }
            hold_sent = true;
        }
        if (current.phase == Phase::Hold) run_end_ns = now;
        current.elapsed_ns = run_start_ns ? run_end_ns - run_start_ns : 0;
        if (terminal_fault) current.anomalies |= TerminalFault;
        current.anomalies = classify(current, expected_wkc, expected_slaves);
        push(current); // Preserve the triggering cycle before announcing completion/fault.
        if (terminal_fault) finish(StopReason::TerminalFault);
        else if (current.anomalies & CommandRejected) finish(StopReason::CommandRejected);
        else if (write_failed.load(std::memory_order_acquire)) finish(StopReason::LogWriteFailed);
        else if (current.phase == Phase::Hold && current.elapsed_ns >= duration_ns)
            finish(StopReason::Completed);
    }

    bool start_writer(const std::filesystem::path& directory)
    {
        cycles_file.open(directory / "cycles.csv");
        events_file.open(directory / "motor_events.csv");
        write_headers(cycles_file, events_file);
        cycles_file.flush();
        events_file.flush();
        if (!cycles_file || !events_file) return false;
        cycles_file << std::setprecision(12);
        events_file << std::setprecision(12);
        std::string error;
        const robot_base::ThreadRuntimeOptions options{
            {}, robot_base::ThreadSchedulingPolicy::OTHER, 0, 0};
        if (!robot_base::start_configured_thread(writer, "ecat_log", options,
                [this] { writer_loop(); }, error)) {
            std::cerr << "[stability] logger setup failed: " << error << '\n';
            return false;
        }
        return true;
    }

    void writer_loop()
    {
        auto next_progress = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        CycleSample sample;
        CycleSample latest;
        while (true) {
            // Acquire completion BEFORE draining: after completion no producer can
            // publish behind the final empty check.
            const bool done = producer_done.load(std::memory_order_acquire);
            bool any = false, anomaly = false;
            while (pop(sample)) {
                any = true;
                latest = sample;
                stats.add(sample, expected_wkc);
                if (!write_failed.load(std::memory_order_relaxed))
                    write_sample(cycles_file, events_file, sample);
                anomaly |= sample.anomalies != 0;
            }
            const auto now = std::chrono::steady_clock::now();
            if (anomaly || now >= next_progress || done) {
                cycles_file.flush();
                events_file.flush();
                if (!cycles_file || !events_file) {
                    write_failed.store(true, std::memory_order_release);
                    finish(StopReason::LogWriteFailed);
                }
            }
            if (now >= next_progress) {
                std::cout << "[stability] elapsed=" << latest.elapsed_ns / 1e9
                          << "s phase=" << static_cast<unsigned>(latest.phase)
                          << " wc_state=" << static_cast<unsigned>(latest.bus.wc_state)
                          << " WKC=" << latest.bus.working_counter
                          << " slaves=" << latest.bus.slaves_responding
                          << " ready=" << latest.ready_motors << "/12"
                          << " comm_bad=" << stats.communication_bad_cycles
                          << " timing_bad=" << stats.timing_anomaly_cycles << std::endl;
                next_progress = now + std::chrono::seconds(1);
            }
            if (done) break;
            if (!any) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (overflowed) {
            stats.add(overflow_sample, expected_wkc);
            write_sample(cycles_file, events_file, overflow_sample);
        }
        cycles_file.flush();
        events_file.flush();
        if (!cycles_file || !events_file) write_failed.store(true, std::memory_order_release);
    }

    void stop_writer()
    {
        producer_done.store(true, std::memory_order_release); // RT must already be joined.
        if (writer.joinable()) writer.join();
        cycles_file.close();
        events_file.close();
        if (cycles_file.fail() || events_file.fail())
            write_failed.store(true, std::memory_order_release);
    }

    bool passed(bool stop_confirmed) const
    {
        return reason.load() == StopReason::Completed && run_start_ns > 0 &&
            run_end_ns - run_start_ns >= duration_ns && stats.hold_cycles > 0 &&
            stats.communication_bad_cycles == 0 && stats.control_fault_cycles == 0 &&
            !overflowed && !write_failed.load() && stop_confirmed;
    }

    bool write_summary(const std::filesystem::path& directory, bool stop_confirmed,
                       bool released_after_comm_loss)
    {
        std::ofstream out(directory / "summary.json");
        out << std::boolalpha << std::setprecision(12)
            << "{\n  \"result\":\"" << (passed(stop_confirmed) ? "PASS" : "FAIL")
            << "\",\n  \"stop_reason\":\""
            << reason_name(write_failed.load() ? StopReason::LogWriteFailed : reason.load())
            << "\",\n  \"scope\":\"supported_static_motor_position_hold\","
            << "\n  \"anchor_monotonic_ns\":" << anchor_monotonic_ns
            << ",\n  \"anchor_unix_ns\":" << anchor_unix_ns
            << ",\n  \"requested_duration_s\":" << duration_ns / 1e9
            << ",\n  \"actual_hold_duration_s\":"
            << (run_start_ns ? (run_end_ns - run_start_ns) / 1e9 : 0.0)
            << ",\n  \"hold_start_monotonic_ns\":" << run_start_ns
            << ",\n  \"hold_end_monotonic_ns\":" << run_end_ns
            << ",\n  \"period_ns\":" << kPeriodNs
            << ",\n  \"scheduler\":\"SCHED_FIFO\",\n  \"rt_priority\":80,"
            << "\n  \"motor_count\":12,\n  \"kp\":" << kKp << ",\n  \"kd\":" << kKd
            << ",\n  \"command_validity_ns\":" << kCommandValidityNs
            << ",\n  \"comm_watchdog_fault_cycles\":10,"
            << "\n  \"process_data_watchdog_ms\":100,"
            << "\n  \"expected_working_counter\":" << expected_wkc
            << ",\n  \"expected_slaves_responding\":" << expected_slaves
            << ",\n  \"cycles_written\":" << stats.cycles
            << ",\n  \"hold_cycles\":" << stats.hold_cycles
            << ",\n  \"hold_anomaly_cycles\":" << stats.anomaly_cycles
            << ",\n  \"communication_bad_cycles\":" << stats.communication_bad_cycles
            << ",\n  \"control_fault_cycles\":" << stats.control_fault_cycles
            << ",\n  \"wc_zero_cycles\":" << stats.wc_zero_cycles
            << ",\n  \"wc_incomplete_cycles\":" << stats.wc_incomplete_cycles
            << ",\n  \"wkc_changed_cycles\":" << stats.wkc_changed_cycles
            << ",\n  \"slave_count_changed_cycles\":" << stats.slave_count_changed_cycles
            << ",\n  \"motor_not_ready_cycles\":" << stats.motor_not_ready_cycles
            << ",\n  \"invalid_pdo_cycles\":" << stats.invalid_pdo_cycles
            << ",\n  \"first_anomaly_monotonic_ns\":" << stats.first_anomaly_ns
            << ",\n  \"first_communication_anomaly_monotonic_ns\":"
            << stats.first_communication_anomaly_ns
            << ",\n  \"first_timing_anomaly_monotonic_ns\":" << stats.first_timing_anomaly_ns
            << ",\n  \"timing_anomaly_cycles\":" << stats.timing_anomaly_cycles
            << ",\n  \"work_overruns\":" << stats.work_overruns
            << ",\n  \"cycle_gaps\":" << stats.cycle_gaps
            << ",\n  \"cycle_interval\":";
        stats.interval.json(out);
        out << ",\n  \"pdo_work\":";
        stats.work.json(out);
        out << ",\n  \"queue_capacity\":" << buffer.size()
            << ",\n  \"max_queue_depth\":" << max_queue_depth
            << ",\n  \"dropped_cycles\":" << dropped_cycles
            << ",\n  \"overflow_sample_salvaged\":" << overflowed
            << ",\n  \"log_complete\":" << (!overflowed && !write_failed.load())
            << ",\n  \"stop_confirmed\":" << stop_confirmed
            << ",\n  \"released_after_comm_loss\":" << released_after_comm_loss
            << "\n}\n";
        out.flush();
        return static_cast<bool>(out);
    }
};

class StabilityAdapter final : public myactua::EthercatAdapterIGH {
public:
    explicit StabilityAdapter(TestRun& run) : run_(run) {}
    myactua::MyActMotorController* controller{nullptr}; // Assigned before RT starts.
    void receive_physical() override
    {
        const bool recording = run_.active.load(std::memory_order_acquire);
        if (recording) run_.begin(robot_base::monotonic_now_ns());
        EthercatAdapterIGH::receive_physical();
        if (!recording) return; // Startup readiness runs before the RT thread.
        std::array<ec_slave_config_state_t, kMotors> slaves{};
        std::array<myactua::RxPDO, kMotors> rx{};
        for (std::size_t i = 0; i < kMotors; ++i) {
            slaves[i] = get_slave_state(static_cast<int>(i));
            rx[i] = EthercatAdapterIGH::receive(static_cast<int>(i));
        }
        run_.observe_receive(get_bus_health(), slaves, rx);
        run_.generate_hold(*controller);
    }
    void send(int index, const myactua::TxPDO& pdo) override
    {
        run_.latest_tx[static_cast<std::size_t>(index)] = pdo;
        EthercatAdapterIGH::send(index, pdo);
    }
    void send_physical() override
    {
        EthercatAdapterIGH::send_physical();
        if (run_.active.load(std::memory_order_acquire))
            run_.end(robot_base::monotonic_now_ns(), controller->terminal_fault_latched());
    }
private:
    TestRun& run_;
};

bool wait_command(myactua::MyActMotorController& controller,
                  const mb::CommandSubmitResult& submitted, bool stopping)
{
    if (submitted.status != mb::CommandSubmitStatus::ACCEPTED || !submitted.command_id)
        return false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
    while (std::chrono::steady_clock::now() < deadline) {
        const auto state = controller.get_discrete_command_result(*submitted.command_id);
        if (state == mb::DiscreteCommandResult::SUCCEEDED) return true;
        if (state != mb::DiscreteCommandResult::PENDING) return false;
        if (!stopping && (g_interrupt || controller.terminal_fault_latched())) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

CycleSample healthy_sample()
{
    CycleSample s;
    s.timestamp_ns = 1'000'000'000;
    s.end_ns = s.timestamp_ns + 100'000;
    s.work_ns = 100'000;
    s.interval_ns = kPeriodNs;
    s.phase = Phase::Hold;
    s.bus.master_link_up = true;
    s.bus.wc_state = EC_WC_COMPLETE;
    s.bus.working_counter = 36;
    s.bus.slaves_responding = 15;
    s.bus.al_states = 8;
    s.ready_motors = kMotors;
    for (auto& m : s.motors) {
        m.slave.online = m.slave.operational = 1;
        m.slave.al_state = 8;
        m.pdo_valid = true;
        m.last_valid_ns = s.timestamp_ns;
        m.raw.status_word = myactua::CMD_ENABLE_OPERATION;
        m.raw.status_word &= ~myactua::BIT_FAULT;
        m.raw.op_mode = static_cast<int8_t>(myactua::MyactControlMode::PVT);
        m.last_valid = m.raw;
    }
    return s;
}

bool self_test()
{
    auto check = [](bool ok, const char* what) {
        if (!ok) std::cerr << "[self-test] FAILED: " << what << '\n';
        return ok;
    };
    bool ok = true;
    const auto good = healthy_sample();
    ok &= check(classify(good, 36, 15) == 0, "healthy full topology");
    auto bad = good;
    bad.bus.wc_state = EC_WC_ZERO;
    bad.bus.working_counter = 0;
    bad.bus.slaves_responding = 14;
    bad.ready_motors = 11;
    bad.motors[0].slave.online = bad.motors[0].slave.operational = 0;
    bad.motors[0].pdo_valid = false;
    bad.anomalies = classify(bad, 36, 15);
    ok &= check((bad.anomalies & (WkcBad | SlaveCountChanged | MotorNotReady | InvalidPdo)) ==
                (WkcBad | SlaveCountChanged | MotorNotReady | InvalidPdo), "WKC 36->0 / 15->14 / motor 0 offline");
    Statistics stats;
    stats.add(good, 36);
    stats.add(bad, 36);
    ok &= check(stats.hold_cycles == 2 && stats.communication_bad_cycles == 1 &&
                stats.wc_zero_cycles == 1 && stats.first_communication_anomaly_ns == bad.timestamp_ns,
                "fault counters and first timestamp");
    bad = good;
    bad.bus.wc_state = EC_WC_INCOMPLETE;
    ok &= check((classify(bad, 36, 15) & WkcBad) != 0, "incomplete WKC at unchanged value");
    bad = good;
    bad.motors[4].raw.error = 0x1234;
    ok &= check((classify(bad, 36, 15) & DriveFault) != 0, "valid drive error");
    bad.motors[4].pdo_valid = false;
    ok &= check((classify(bad, 36, 15) & DriveFault) == 0, "invalid PDO not a fresh drive error");
    bad = good;
    bad.motors[2].raw.status_word = 0;
    bad.motors[3].raw.op_mode = 0;
    bad.work_ns = kPeriodNs;
    bad.interval_ns = 2 * kPeriodNs;
    const auto flags = classify(bad, 36, 15);
    ok &= check((flags & (Disabled | ModeChanged | WorkOverrun | CycleGap)) ==
                (Disabled | ModeChanged | WorkOverrun | CycleGap), "drive/clock anomaly thresholds");

    TestRun run(2 * kPeriodNs, 2);
    run.expected_wkc = 36;
    run.expected_slaves = 15;
    std::array<ec_slave_config_state_t, kMotors> slaves{};
    std::array<myactua::RxPDO, kMotors> rx{};
    for (std::size_t i = 0; i < kMotors; ++i) {
        slaves[i] = good.motors[i].slave;
        rx[i] = good.motors[i].raw;
        rx[i].pos = static_cast<int32_t>(1000 + i);
    }
    run.begin(good.timestamp_ns);
    run.observe_receive(good.bus, slaves, rx);
    const auto saved = run.current.motors[0].last_valid;
    auto health = good.bus;
    health.wc_state = EC_WC_ZERO;
    health.working_counter = 0;
    rx[0].pos = -9999;
    run.begin(good.timestamp_ns + kPeriodNs);
    run.observe_receive(health, slaves, rx);
    ok &= check(!run.current.motors[0].pdo_valid &&
                run.current.motors[0].last_valid.pos == saved.pos &&
                run.current.motors[0].last_valid_ns == good.timestamp_ns &&
                run.current.motors[0].raw.pos == -9999, "raw vs last valid PDO / age");
    run.begin(good.timestamp_ns + 2 * kPeriodNs);
    run.observe_receive(good.bus, slaves, rx);
    slaves[0].online = 0;
    run.begin(good.timestamp_ns + 3 * kPeriodNs);
    run.observe_receive(good.bus, slaves, rx);
    ok &= check(!run.current.motors[0].pdo_valid &&
                run.current.motors[0].last_valid_ns == good.timestamp_ns + 2 * kPeriodNs,
                "unconfigured motor cannot refresh feedback");

    ok &= check(run.push(good) && run.push(bad) && !run.push(bad) &&
                run.overflowed && run.dropped_cycles == 1 &&
                run.reason.load() == StopReason::LogOverflow &&
                (run.overflow_sample.anomalies & LogOverflow), "bounded queue overflow preserves evidence");
    CycleSample popped;
    ok &= check(run.pop(popped) && popped.timestamp_ns == good.timestamp_ns &&
                run.pop(popped) && !run.pop(popped), "queue drain order");

    TestRun terminal(kPeriodNs, 4);
    terminal.expected_wkc = 36;
    terminal.expected_slaves = 15;
    terminal.current = good;
    terminal.run_start_ns = good.timestamp_ns;
    terminal.started.store(true);
    terminal.end(good.end_ns, true);
    ok &= check(terminal.reason.load() == StopReason::TerminalFault &&
                terminal.pop(popped) && (popped.anomalies & TerminalFault),
                "triggering cycle recorded before terminal stop");
    terminal.stats.add(popped, 36);
    ok &= check(!terminal.passed(true), "terminal fault cannot pass");

    TestRun completed(kPeriodNs, 4);
    completed.expected_wkc = 36;
    completed.expected_slaves = 15;
    completed.current = good;
    completed.run_start_ns = good.timestamp_ns - kPeriodNs;
    completed.started.store(true);
    completed.end(good.end_ns, false);
    completed.pop(popped);
    completed.stats.add(popped, 36);
    ok &= check(completed.reason.load() == StopReason::Completed && completed.passed(true) &&
                !completed.passed(false), "duration and confirmed stop acceptance");
    completed.reason.store(StopReason::Interrupted);
    ok &= check(!completed.passed(true), "early interrupt cannot pass");
    completed.reason.store(StopReason::Completed);
    completed.write_failed.store(true);
    ok &= check(!completed.passed(true), "write failure cannot pass");

    TestRun gating(10 * kPeriodNs, 4);
    gating.expected_wkc = 36;
    gating.expected_slaves = 15;
    gating.armed.store(true);
    gating.current = good;
    gating.current.phase = Phase::Startup;
    gating.end(good.end_ns, false);
    ok &= check(!gating.started.load(), "enable alone cannot start hold timer");
    gating.hold_captured = true;
    for (std::size_t i = 0; i < kMotors; ++i) {
        auto& tx = gating.latest_tx[i];
        tx.control_word = myactua::CMD_ENABLE_OPERATION;
        tx.op_mode = static_cast<int8_t>(myactua::MyactControlMode::PVT);
        tx.pvt_kp = 180'000;
        tx.pvt_kd = 10'000;
    }
    gating.current = good;
    gating.current.phase = Phase::Startup;
    gating.end(good.end_ns, false);
    ok &= check(!gating.started.load() && gating.hold_sent,
                "first outgoing hold must precede valid feedback");
    gating.current = good;
    gating.current.timestamp_ns += kPeriodNs;
    gating.current.phase = Phase::Startup;
    gating.end(good.end_ns + kPeriodNs, false);
    ok &= check(gating.started.load() && gating.run_start_ns == good.timestamp_ns + kPeriodNs,
                "timer starts only after full hold and subsequent feedback");

    std::ostringstream cycles, events;
    write_headers(cycles, events);
    bad.anomalies = classify(bad, 36, 15);
    write_sample(cycles, events, bad);
    const auto csv = cycles.str(), event_csv = events.str();
    ok &= check(std::count(csv.begin(), csv.end(), '\n') == 2 &&
                std::count(event_csv.begin(), event_csv.end(), '\n') == 13 &&
                event_csv.find("last_valid_monotonic_ns") != std::string::npos,
                "one cycle and all twelve motor event rows");
    // Entire thirty-minute statistics path, without sleeping or touching hardware.
    Statistics long_run;
    for (std::uint64_t i = 0; i < 1'800'000; ++i) long_run.add(good, 36);
    ok &= check(long_run.hold_cycles == 1'800'000 && long_run.communication_bad_cycles == 0 &&
                long_run.interval.min == kPeriodNs && long_run.work.max == good.work_ns,
                "thirty-minute counter capacity");
    std::cout << "[stability] self-test " << (ok ? "passed" : "failed") << '\n';
    return ok;
}

} // namespace

int main(int argc, char** argv)
{
    std::int64_t duration_s = 1800;
    std::filesystem::path root = "logs/ethercat_stability";
    bool run_self_test = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--self-test") run_self_test = true;
        else if (arg == "--duration" && i + 1 < argc) {
            const std::string value = argv[++i];
            std::size_t parsed = 0;
            try {
                duration_s = std::stoll(value, &parsed);
            } catch (const std::exception&) {
                std::cerr << "[stability] duration must be positive integer seconds\n";
                return 2;
            }
            if (parsed != value.size() || duration_s <= 0 ||
                duration_s > std::numeric_limits<std::int64_t>::max() / 1'000'000'000) {
                std::cerr << "[stability] duration is out of range\n";
                return 2;
            }
        } else if (arg == "--output-dir" && i + 1 < argc) root = argv[++i];
        else {
            std::cout << "usage: ethercat_stability_test [--duration 1800] "
                         "[--output-dir logs/ethercat_stability] [--self-test]\n";
            return arg == "--help" ? 0 : 2;
        }
    }
    if (run_self_test) return self_test() ? 0 : 1;
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto run = std::make_unique<TestRun>(duration_s * 1'000'000'000);
    const auto directory = root / ("run_" + std::to_string(run->anchor_unix_ns) +
                                   "_" + std::to_string(getpid()));
    std::error_code error;
    if (!std::filesystem::create_directories(directory, error) || error) {
        std::cerr << "[stability] cannot create log directory: " << directory
                  << " " << error.message() << '\n';
        return 2;
    }
    if (!run->start_writer(directory)) {
        std::cerr << "[stability] cannot start log writer\n";
        return 2; // No hardware has been initialized.
    }
    auto adapter = std::make_shared<StabilityAdapter>(*run);
    myactua::MyActMotorController::Options options;
    options.rt_period_ns = kPeriodNs;
    options.comm_watchdog_fault_cycles = 10;
    options.rt_thread_options = {{}, robot_base::ThreadSchedulingPolicy::FIFO, 80, 0};
    myactua::MyActMotorController controller(adapter, kMotors, options);
    adapter->controller = &controller;
    controller.set_print_info({});
    std::cout << "[stability] supported static hold; motors=12 rate=1000 Hz duration="
              << duration_s << "s Kp=" << kKp << " Kd=" << kKd
              << "\n[stability] logs: " << directory << std::endl;

    bool rt_started = false, stop_confirmed = false, released_after_comm_loss = false;
    if (!controller.connect() || g_interrupt ||
        !controller.wait_all_motors_ready(50'000, 100) || g_interrupt) {
        run->finish(g_interrupt ? StopReason::Interrupted : StopReason::StartupFailed);
    } else {
        const auto baseline = adapter->get_bus_health();
        run->expected_wkc = baseline.working_counter;
        run->expected_slaves = baseline.slaves_responding;
        run->active.store(true, std::memory_order_release);
        rt_started = controller.start();
        if (!rt_started) run->finish(StopReason::StartupFailed);
        else if (!wait_command(controller, controller.send_discrete_command(
                     mb::ControlCommand::set_mode(mb::MotorControlMode::IMPEDANCE)), false) ||
                 !wait_command(controller, controller.send_discrete_command(
                     mb::ControlCommand::restart()), false)) {
            run->finish(g_interrupt ? StopReason::Interrupted : StopReason::StartupFailed);
        } else {
            run->armed.store(true, std::memory_order_release);
            const auto hold_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
            while (run->reason.load(std::memory_order_acquire) == StopReason::None) {
                if (g_interrupt) run->finish(StopReason::Interrupted);
                else if (run->write_failed.load(std::memory_order_acquire))
                    run->finish(StopReason::LogWriteFailed);
                else if (!run->started.load(std::memory_order_acquire) &&
                         std::chrono::steady_clock::now() >= hold_deadline)
                    run->finish(StopReason::StartupFailed);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    run->stopping.store(true, std::memory_order_release);
    run->armed.store(false, std::memory_order_release);
    if (rt_started) {
        do {
            stop_confirmed = wait_command(controller,
                controller.send_discrete_command(mb::ControlCommand::stop()), true);
            if (stop_confirmed) break;
            released_after_comm_loss = controller.communication_fault_latched() &&
                controller.current_communication_fault() != myactua::MyactCommunicationFaultReason::None;
            if (!released_after_comm_loss)
                std::cerr << "[stability] STOP unconfirmed; retaining RT and retrying\n";
        } while (!released_after_comm_loss);
        controller.shutdown();
        if (released_after_comm_loss) {
            // No further process data refresh: let the configured 100 ms ESC
            // watchdog expire before the adapter/master is released.
            std::this_thread::sleep_for(std::chrono::milliseconds(120));
            std::cerr << "[stability] communication lost; STOP unconfirmed; "
                         "released after process-data watchdog timeout\n";
        }
    }
    run->active.store(false, std::memory_order_release);
    run->stop_writer();
    const bool summary_written = run->write_summary(directory, stop_confirmed,
                                                    released_after_comm_loss);
    if (!summary_written) run->write_failed.store(true);
    const bool passed = summary_written && run->passed(stop_confirmed);
    std::cout << "[stability] " << (passed ? "PASS" : "FAIL")
              << " reason=" << reason_name(run->write_failed.load()
                    ? StopReason::LogWriteFailed : run->reason.load())
              << " stop_confirmed=" << stop_confirmed
              << " comm_bad=" << run->stats.communication_bad_cycles
              << " control_faults=" << run->stats.control_fault_cycles
              << " timing_bad=" << run->stats.timing_anomaly_cycles
              << " log_complete=" << (!run->overflowed && !run->write_failed.load())
              << "\n[stability] summary: " << directory / "summary.json" << '\n';
    if (!summary_written) std::cerr << "[stability] failed to write summary.json\n";
    return passed ? 0 : 1;
}
