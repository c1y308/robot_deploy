#include "ControlTypes.hpp"
#include "EthercatAdapterIGH.hpp"
#include "ankle_motor_ik.hpp"
#include "motor_control.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

volatile std::sig_atomic_t g_should_stop = 0;

constexpr char kIfName[] = "enp8s0";
constexpr int kNumMotors = 12;
constexpr int kWaitReadyTimeoutMs = 20000;
constexpr int kWaitReadyPollMs = 100;
constexpr int kWarmupMs = 1000;
constexpr int kModeSwitchWaitMs = 1500;
constexpr int kRestartWaitMs = 1000;
constexpr int kMoveToStandMs = 3000;
constexpr int kStandHoldMs = 1000;
constexpr int kPreStepHoldMs = 300;
constexpr int kStepHoldMs = 500;
constexpr int kReturnHoldMs = 500;
constexpr int kCommandPeriodMs = 20;
constexpr int kTrials = 10;
constexpr int kResponseTimeoutMs = 300;
constexpr double kStepFraction = 0.40;
constexpr double kResponseThresholdFraction = 0.10;
constexpr double kDefaultKp = 250.0;
constexpr double kDefaultKd = 5.0;
constexpr double kJointMinRad = -0.45;
constexpr double kJointMaxRad = 0.45;
constexpr double kMinimumDeltaRad = 1e-6;
constexpr std::size_t kSampleBufferCapacity = 240000;

const std::array<double, kNumMotors> kStandPoseRad = {
    0.0, 0.0, -0.0, -0.0, 0.0, 0.0, 0.05, 0.05, -0.0, -0.0, 0.0, 0.0
};

const std::array<std::array<double, 2>, kNumMotors> kActionClip = {{
    {-0.12, 0.12},
    {-0.12, 0.12},
    {-0.30, 0.30},
    {-0.30, 0.30},
    {-0.12, 0.12},
    {-0.12, 0.12},
    { 0.00, 0.45},
    { 0.00, 0.45},
    {-0.25, 0.25},
    {-0.25, 0.25},
    {-0.08, 0.08},
    {-0.08, 0.08},
}};

const std::array<int, 8> kModelToMotorIndex = {0, 6, 1, 7, 2, 8, 3, 9};
const std::array<int, kNumMotors> kMotorToModelDirection = {
     1, -1, 1,  1, -1, -1,
    -1,  1, 1, -1, -1, -1
};

struct AnkleParallelMap {
    int model_pitch_dof = -1;
    int model_roll_dof = -1;
    int upper_motor_index = -1;
    int lower_motor_index = -1;
};

const AnkleParallelMap kLeftAnkleParallel{8, 10, 4, 5};
const AnkleParallelMap kRightAnkleParallel{9, 11, 10, 11};

struct Sample {
    Clock::time_point time{};
    std::array<double, kNumMotors> position_rad{};
    std::array<uint8_t, kNumMotors> comm_ok{};
    bool valid = false;
};

class SampleRecorder {
public:
    void reset()
    {
        recording_.store(false, std::memory_order_release);
        write_count_.store(0, std::memory_order_release);
        overflow_.store(false, std::memory_order_release);
    }

    void set_recording(bool enabled)
    {
        recording_.store(enabled, std::memory_order_release);
    }

    void record(const std::vector<myactua::MotorStatusSnapshot>& status)
    {
        if (!recording_.load(std::memory_order_relaxed)) {
            return;
        }

        const std::uint64_t index = write_count_.fetch_add(1, std::memory_order_relaxed);
        if (index >= samples_.size()) {
            overflow_.store(true, std::memory_order_relaxed);
            return;
        }

        Sample& sample = samples_[static_cast<std::size_t>(index)];
        sample.time = Clock::now();
        for (int i = 0; i < kNumMotors; ++i) {
            if (i < static_cast<int>(status.size())) {
                sample.position_rad[i] = myactua::MYACTUA::raw_pos_to_rad(status[i].position);
                sample.comm_ok[i] = status[i].comm_ok ? 1U : 0U;
            } else {
                sample.position_rad[i] = std::numeric_limits<double>::quiet_NaN();
                sample.comm_ok[i] = 0U;
            }
        }
        sample.valid = true;
    }

    std::size_t sample_count() const
    {
        const std::uint64_t count = write_count_.load(std::memory_order_acquire);
        return static_cast<std::size_t>(
            std::min<std::uint64_t>(count, samples_.size()));
    }

    bool overflowed() const
    {
        return overflow_.load(std::memory_order_acquire);
    }

    const Sample& sample(std::size_t index) const
    {
        return samples_[index];
    }

private:
    std::array<Sample, kSampleBufferCapacity> samples_{};
    std::atomic<std::uint64_t> write_count_{0};
    std::atomic<bool> recording_{false};
    std::atomic<bool> overflow_{false};
};

SampleRecorder g_recorder;

struct StepEvent {
    int trial = 0;
    Clock::time_point command_time{};
    std::array<double, kNumMotors> baseline_rad{};
    std::array<double, kNumMotors> target_rad{};
};

void signal_handler(int)
{
    g_should_stop = 1;
}

void sleep_ms(int ms)
{
    constexpr int kSliceMs = 20;
    for (int slept = 0; slept < ms && !g_should_stop; slept += kSliceMs) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(std::min(kSliceMs, ms - slept)));
    }
}

void force_sleep_ms(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int direction_for_motor(int motor_index)
{
    if (motor_index < 0 || motor_index >= kNumMotors) {
        return 1;
    }
    return kMotorToModelDirection[static_cast<std::size_t>(motor_index)];
}

bool ankle_map_indices_in_range(const AnkleParallelMap& ankle_map)
{
    return ankle_map.model_pitch_dof >= 0 &&
           ankle_map.model_pitch_dof < kNumMotors &&
           ankle_map.model_roll_dof >= 0 &&
           ankle_map.model_roll_dof < kNumMotors &&
           ankle_map.upper_motor_index >= 0 &&
           ankle_map.upper_motor_index < kNumMotors &&
           ankle_map.lower_motor_index >= 0 &&
           ankle_map.lower_motor_index < kNumMotors;
}

bool is_parallel_ankle_model_dof(int model_index)
{
    return model_index == kLeftAnkleParallel.model_pitch_dof ||
           model_index == kLeftAnkleParallel.model_roll_dof ||
           model_index == kRightAnkleParallel.model_pitch_dof ||
           model_index == kRightAnkleParallel.model_roll_dof;
}

int direct_model_mapping_slot(int model_index)
{
    int slot = 0;
    for (int i = 0; i < model_index; ++i) {
        if (!is_parallel_ankle_model_dof(i)) {
            ++slot;
        }
    }
    return slot;
}

double clamp_model_position(int model_index, double q_rad)
{
    const double lower = kStandPoseRad[static_cast<std::size_t>(model_index)] + kJointMinRad;
    const double upper = kStandPoseRad[static_cast<std::size_t>(model_index)] + kJointMaxRad;
    return std::clamp(q_rad, lower, upper);
}

class TargetBuilder {
public:
    bool build(const std::array<double, kNumMotors>& target_q_model_rad,
               std::array<double, kNumMotors>& target_motor_rad)
    {
        target_motor_rad.fill(0.0);

        for (int model_index = 0; model_index < kNumMotors; ++model_index) {
            if (is_parallel_ankle_model_dof(model_index)) {
                continue;
            }

            const int mapping_slot = direct_model_mapping_slot(model_index);
            if (mapping_slot < 0 ||
                mapping_slot >= static_cast<int>(kModelToMotorIndex.size())) {
                return false;
            }

            const int motor_index = kModelToMotorIndex[static_cast<std::size_t>(mapping_slot)];
            if (motor_index < 0 || motor_index >= kNumMotors) {
                return false;
            }

            const double q = clamp_model_position(model_index,
                                                  target_q_model_rad[static_cast<std::size_t>(model_index)]);
            target_motor_rad[static_cast<std::size_t>(motor_index)] =
                static_cast<double>(direction_for_motor(motor_index)) * q;
        }

        return apply_ankle_ik(target_q_model_rad, target_motor_rad,
                              kLeftAnkleParallel, left_ankle_solver_,
                              left_ankle_last_upper_motor_,
                              left_ankle_last_lower_motor_,
                              left_ankle_solved_) &&
               apply_ankle_ik(target_q_model_rad, target_motor_rad,
                              kRightAnkleParallel, right_ankle_solver_,
                              right_ankle_last_upper_motor_,
                              right_ankle_last_lower_motor_,
                              right_ankle_solved_);
    }

private:
    bool apply_ankle_ik(const std::array<double, kNumMotors>& target_q_model_rad,
                        std::array<double, kNumMotors>& target_motor_rad,
                        const AnkleParallelMap& ankle_map,
                        ankle_motor_ik::Solver& solver,
                        double& last_upper_motor,
                        double& last_lower_motor,
                        bool& solved)
    {
        if (!ankle_map_indices_in_range(ankle_map)) {
            return false;
        }

        const double pitch = clamp_model_position(
            ankle_map.model_pitch_dof,
            target_q_model_rad[static_cast<std::size_t>(ankle_map.model_pitch_dof)]);
        const double roll = clamp_model_position(
            ankle_map.model_roll_dof,
            target_q_model_rad[static_cast<std::size_t>(ankle_map.model_roll_dof)]);

        const ankle_motor_ik::MotorAngles result = solver.solve(roll, pitch);
        double upper_motor = 0.0;
        double lower_motor = 0.0;
        if (result.reachable()) {
            upper_motor = result.motor1;
            lower_motor = result.motor2;
            last_upper_motor = upper_motor;
            last_lower_motor = lower_motor;
            solved = true;
        } else if (solved) {
            upper_motor = last_upper_motor;
            lower_motor = last_lower_motor;
        }

        target_motor_rad[static_cast<std::size_t>(ankle_map.upper_motor_index)] =
            static_cast<double>(direction_for_motor(ankle_map.upper_motor_index)) *
            upper_motor;
        target_motor_rad[static_cast<std::size_t>(ankle_map.lower_motor_index)] =
            static_cast<double>(direction_for_motor(ankle_map.lower_motor_index)) *
            lower_motor;
        return true;
    }

    ankle_motor_ik::Solver left_ankle_solver_;
    ankle_motor_ik::Solver right_ankle_solver_;
    double left_ankle_last_upper_motor_ = 0.0;
    double left_ankle_last_lower_motor_ = 0.0;
    double right_ankle_last_upper_motor_ = 0.0;
    double right_ankle_last_lower_motor_ = 0.0;
    bool left_ankle_solved_ = false;
    bool right_ankle_solved_ = false;
};

std::array<double, kNumMotors> make_step_model_pose(int trial)
{
    std::array<double, kNumMotors> pose = kStandPoseRad;
    const bool positive_trial = (trial % 2) == 0;

    for (int model_index = 0; model_index < kNumMotors; ++model_index) {
        const auto& clip = kActionClip[static_cast<std::size_t>(model_index)];
        double offset = 0.0;
        if (clip[0] < 0.0 && clip[1] > 0.0) {
            offset = positive_trial ? clip[1] * kStepFraction : clip[0] * kStepFraction;
        } else if (clip[1] > 0.0) {
            offset = clip[1] * kStepFraction;
        } else if (clip[0] < 0.0) {
            offset = clip[0] * kStepFraction;
        }

        pose[static_cast<std::size_t>(model_index)] =
            clamp_model_position(model_index,
                                 kStandPoseRad[static_cast<std::size_t>(model_index)] + offset);
    }

    return pose;
}

std::vector<myactua::MitSetpoint> make_mit_setpoints(
    const std::array<double, kNumMotors>& target_rad)
{
    std::vector<myactua::MitSetpoint> setpoints;
    setpoints.reserve(kNumMotors);
    for (double position_rad : target_rad) {
        setpoints.emplace_back(myactua::MYACTUA::rad_to_deg(position_rad),
                               0.0,
                               0.0,
                               kDefaultKp,
                               kDefaultKd);
    }
    return setpoints;
}

void send_mit_targets(myactua::MYACTUA& controller,
                      const std::array<double, kNumMotors>& target_rad)
{
    controller.send_command(
        myactua::ControlCommand::SetMitSetpoints(make_mit_setpoints(target_rad)));
}

bool read_motor_positions(myactua::MYACTUA& controller,
                          std::array<double, kNumMotors>& positions_rad)
{
    constexpr int kAttempts = 50;
    constexpr int kRetryMs = 20;

    for (int attempt = 0; attempt < kAttempts && !g_should_stop; ++attempt) {
        const std::vector<myactua::MotorStatusSnapshot> status = controller.get_status();
        if (status.size() == static_cast<std::size_t>(kNumMotors)) {
            bool all_comm_ok = true;
            for (int i = 0; i < kNumMotors; ++i) {
                all_comm_ok = all_comm_ok && status[static_cast<std::size_t>(i)].comm_ok;
            }
            if (all_comm_ok) {
                for (int i = 0; i < kNumMotors; ++i) {
                    positions_rad[static_cast<std::size_t>(i)] =
                        myactua::MYACTUA::raw_pos_to_rad(
                            status[static_cast<std::size_t>(i)].position);
                }
                return true;
            }
        }
        sleep_ms(kRetryMs);
    }

    return false;
}

bool wait_all_mode(myactua::MYACTUA& controller,
                   myactua::ControlMode mode,
                   int timeout_ms)
{
    const auto start = Clock::now();
    while (!g_should_stop) {
        const std::vector<myactua::MotorStatusSnapshot> status = controller.get_status();
        bool all_ok = status.size() == static_cast<std::size_t>(kNumMotors);
        for (int i = 0; all_ok && i < kNumMotors; ++i) {
            const auto& motor = status[static_cast<std::size_t>(i)];
            all_ok = motor.comm_ok && motor.op_mode == mode;
        }
        if (all_ok) {
            return true;
        }

        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                Clock::now() - start).count() >= timeout_ms) {
            return false;
        }
        sleep_ms(20);
    }

    return false;
}

bool wait_all_running(myactua::MYACTUA& controller,
                      myactua::ControlMode mode,
                      int timeout_ms)
{
    const auto start = Clock::now();
    while (!g_should_stop) {
        const std::vector<myactua::MotorStatusSnapshot> status = controller.get_status();
        bool all_ok = status.size() == static_cast<std::size_t>(kNumMotors);
        for (int i = 0; all_ok && i < kNumMotors; ++i) {
            const auto& motor = status[static_cast<std::size_t>(i)];
            all_ok = motor.comm_ok &&
                     motor.op_mode == mode &&
                     myactua::is_operation_enabled(motor.status_word);
        }
        if (all_ok) {
            return true;
        }

        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                Clock::now() - start).count() >= timeout_ms) {
            return false;
        }
        sleep_ms(20);
    }

    return false;
}

void send_mode_all(myactua::MYACTUA& controller, myactua::ControlMode mode)
{
    for (int i = 0; i < kNumMotors; ++i) {
        controller.send_command(myactua::ControlCommand::SetMode(mode, i));
    }
}

bool stream_target_for(myactua::MYACTUA& controller,
                       const std::array<double, kNumMotors>& target_rad,
                       int duration_ms)
{
    const auto start = Clock::now();
    auto next = start;
    const auto end = start + std::chrono::milliseconds(duration_ms);

    while (!g_should_stop && Clock::now() < end) {
        send_mit_targets(controller, target_rad);
        next += std::chrono::milliseconds(kCommandPeriodMs);
        std::this_thread::sleep_until(next);
    }

    return !g_should_stop;
}

bool ramp_to_target(myactua::MYACTUA& controller,
                    const std::array<double, kNumMotors>& start_rad,
                    const std::array<double, kNumMotors>& target_rad,
                    int duration_ms)
{
    const auto start = Clock::now();
    auto next = start;
    const auto duration = std::chrono::milliseconds(duration_ms);
    std::array<double, kNumMotors> command = start_rad;

    while (!g_should_stop) {
        const auto now = Clock::now();
        const auto elapsed = now - start;
        if (elapsed >= duration) {
            break;
        }

        const double alpha = std::chrono::duration<double>(elapsed).count() /
                             std::chrono::duration<double>(duration).count();
        const double blend = alpha * alpha * (3.0 - 2.0 * alpha);
        for (int i = 0; i < kNumMotors; ++i) {
            command[static_cast<std::size_t>(i)] =
                start_rad[static_cast<std::size_t>(i)] +
                (target_rad[static_cast<std::size_t>(i)] -
                 start_rad[static_cast<std::size_t>(i)]) * blend;
        }

        send_mit_targets(controller, command);
        next += std::chrono::milliseconds(kCommandPeriodMs);
        std::this_thread::sleep_until(next);
    }

    if (!g_should_stop) {
        send_mit_targets(controller, target_rad);
    }
    return !g_should_stop;
}

bool send_measured_step(myactua::MYACTUA& controller,
                        const std::array<double, kNumMotors>& target_rad,
                        int trial,
                        std::vector<StepEvent>& events)
{
    StepEvent event;
    event.trial = trial;
    event.target_rad = target_rad;
    if (!read_motor_positions(controller, event.baseline_rad)) {
        std::cerr << "[error] failed to read baseline motor positions\n";
        return false;
    }

    event.command_time = Clock::now();
    send_mit_targets(controller, target_rad);
    events.push_back(event);

    auto next = event.command_time + std::chrono::milliseconds(kCommandPeriodMs);
    const auto end = event.command_time + std::chrono::milliseconds(kStepHoldMs);
    while (!g_should_stop && Clock::now() < end) {
        std::this_thread::sleep_until(next);
        if (Clock::now() >= end) {
            break;
        }
        send_mit_targets(controller, target_rad);
        next += std::chrono::milliseconds(kCommandPeriodMs);
    }

    return !g_should_stop;
}

void safe_stop(myactua::MYACTUA& controller, bool started)
{
    g_recorder.set_recording(false);
    if (started) {
        controller.send_command(myactua::ControlCommand::Stop());
        force_sleep_ms(500);
        controller.shutdown();
    }
    controller.set_status_callback({});
}

struct MotorLatencyStats {
    int skipped = 0;
    int missed = 0;
    std::vector<double> latency_ms;
};

void analyze_and_print_results(const std::vector<StepEvent>& events)
{
    std::array<MotorLatencyStats, kNumMotors> stats;
    const std::size_t sample_count = g_recorder.sample_count();

    for (const StepEvent& event : events) {
        for (int motor_id = 0; motor_id < kNumMotors; ++motor_id) {
            const double baseline =
                event.baseline_rad[static_cast<std::size_t>(motor_id)];
            const double target =
                event.target_rad[static_cast<std::size_t>(motor_id)];
            const double delta = target - baseline;
            const double abs_delta = std::abs(delta);

            if (!std::isfinite(delta) || abs_delta <= kMinimumDeltaRad) {
                ++stats[static_cast<std::size_t>(motor_id)].skipped;
                continue;
            }

            const double direction = delta >= 0.0 ? 1.0 : -1.0;
            const double threshold = abs_delta * kResponseThresholdFraction;
            bool found = false;

            for (std::size_t sample_index = 0; sample_index < sample_count; ++sample_index) {
                const Sample& sample = g_recorder.sample(sample_index);
                if (!sample.valid || !sample.comm_ok[static_cast<std::size_t>(motor_id)]) {
                    continue;
                }
                if (sample.time < event.command_time) {
                    continue;
                }

                const double elapsed_ms =
                    std::chrono::duration<double, std::milli>(
                        sample.time - event.command_time).count();
                if (elapsed_ms > static_cast<double>(kResponseTimeoutMs)) {
                    break;
                }

                const double response =
                    direction *
                    (sample.position_rad[static_cast<std::size_t>(motor_id)] - baseline);
                if (response >= threshold) {
                    stats[static_cast<std::size_t>(motor_id)].latency_ms.push_back(elapsed_ms);
                    found = true;
                    break;
                }
            }

            if (!found) {
                ++stats[static_cast<std::size_t>(motor_id)].missed;
            }
        }
    }

    std::cout << "\n========== motor response latency summary ==========\n";
    std::cout << "samples=" << sample_count
              << " events=" << events.size()
              << " callback_overflow=" << (g_recorder.overflowed() ? "yes" : "no")
              << "\n";
    std::cout << "metric: first feedback movement >= "
              << kResponseThresholdFraction * 100.0
              << "% of physical motor step, timeout="
              << kResponseTimeoutMs << " ms\n\n";

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "motor  trials  measured  missed  skipped  mean_ms  min_ms  p95_ms  max_ms\n";
    for (int motor_id = 0; motor_id < kNumMotors; ++motor_id) {
        auto& motor_stats = stats[static_cast<std::size_t>(motor_id)];
        auto& values = motor_stats.latency_ms;
        std::sort(values.begin(), values.end());

        const int measured = static_cast<int>(values.size());
        const int trials = measured + motor_stats.missed + motor_stats.skipped;
        std::cout << std::setw(5) << motor_id
                  << std::setw(8) << trials
                  << std::setw(10) << measured
                  << std::setw(8) << motor_stats.missed
                  << std::setw(9) << motor_stats.skipped;

        if (values.empty()) {
            std::cout << "      n/a     n/a     n/a     n/a\n";
            continue;
        }

        const double sum = std::accumulate(values.begin(), values.end(), 0.0);
        const double mean = sum / static_cast<double>(values.size());
        const double min_value = values.front();
        const double max_value = values.back();
        const std::size_t p95_index =
            std::min<std::size_t>((values.size() * 95 + 99) / 100 - 1,
                                  values.size() - 1);
        const double p95 = values[p95_index];

        std::cout << std::setw(9) << mean
                  << std::setw(8) << min_value
                  << std::setw(8) << p95
                  << std::setw(8) << max_value << "\n";
    }
    std::cout << std::defaultfloat;
}

}  // namespace

int main()
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    TargetBuilder target_builder;
    std::array<double, kNumMotors> stand_motor_rad{};
    if (!target_builder.build(kStandPoseRad, stand_motor_rad)) {
        std::cerr << "[error] failed to build stand motor targets\n";
        return -1;
    }

    auto adapter = std::make_shared<myactua::EthercatAdapterIGH>();
    myactua::MYACTUA controller(adapter, kNumMotors);
    bool controller_started = false;

    controller.set_print_info({});
    controller.set_status_callback([](const std::vector<myactua::MotorStatusSnapshot>& status) {
        g_recorder.record(status);
    });

    std::cout << "\n========== 50Hz motor response latency test ==========\n";
    std::cout << "[init] connect to " << kIfName << "\n";
    if (!controller.connect(kIfName)) {
        std::cerr << "[error] failed to connect EtherCAT interface\n";
        return -1;
    }

    if (!controller.wait_all_slaves_ready(
            kWaitReadyTimeoutMs, kWaitReadyPollMs, [] { return g_should_stop != 0; })) {
        std::cerr << "[error] not all slaves reached OP within "
                  << kWaitReadyTimeoutMs << " ms\n";
        return -1;
    }

    controller.start();
    controller_started = true;

    std::cout << "[flow] stop all motors before mode switch\n";
    controller.send_command(myactua::ControlCommand::Stop());
    sleep_ms(kWarmupMs);

    std::cout << "[flow] switch all motors to PVT/MIT\n";
    send_mode_all(controller, myactua::ControlMode::PVT);
    sleep_ms(kModeSwitchWaitMs);
    if (!wait_all_mode(controller, myactua::ControlMode::PVT, 4000)) {
        std::cerr << "[error] not all motors reached PVT/MIT mode\n";
        safe_stop(controller, controller_started);
        return -1;
    }

    std::array<double, kNumMotors> current_motor_rad{};
    if (!read_motor_positions(controller, current_motor_rad)) {
        std::cerr << "[error] failed to read initial motor positions\n";
        safe_stop(controller, controller_started);
        return -1;
    }

    std::cout << "[flow] preload current-position MIT setpoints\n";
    send_mit_targets(controller, current_motor_rad);
    sleep_ms(200);

    std::cout << "[flow] restart all motors\n";
    controller.send_command(myactua::ControlCommand::Restart());
    sleep_ms(kRestartWaitMs);
    if (!wait_all_running(controller, myactua::ControlMode::PVT, 4000)) {
        std::cerr << "[error] not all motors reached operation enabled in PVT/MIT mode\n";
        safe_stop(controller, controller_started);
        return -1;
    }

    std::cout << "[flow] move to stand_pose at 50 Hz\n";
    if (!ramp_to_target(controller, current_motor_rad, stand_motor_rad, kMoveToStandMs)) {
        safe_stop(controller, controller_started);
        return g_should_stop ? 130 : 0;
    }
    if (!target_builder.build(kStandPoseRad, stand_motor_rad)) {
        std::cerr << "[error] failed to rebuild stand motor targets\n";
        safe_stop(controller, controller_started);
        return -1;
    }
    if (!stream_target_for(controller, stand_motor_rad, kStandHoldMs)) {
        safe_stop(controller, controller_started);
        return g_should_stop ? 130 : 0;
    }

    std::vector<StepEvent> events;
    events.reserve(kTrials);
    g_recorder.reset();
    g_recorder.set_recording(true);

    std::cout << "[test] run " << kTrials
              << " measured 50Hz step trials, all 12 model DOF together\n";
    for (int trial = 0; trial < kTrials && !g_should_stop; ++trial) {
        if (!target_builder.build(kStandPoseRad, stand_motor_rad)) {
            std::cerr << "[error] failed to rebuild stand motor targets before trial "
                      << trial << "\n";
            break;
        }
        if (!stream_target_for(controller, stand_motor_rad, kPreStepHoldMs)) {
            break;
        }

        const std::array<double, kNumMotors> step_model_pose =
            make_step_model_pose(trial);
        std::array<double, kNumMotors> step_motor_rad{};
        if (!target_builder.build(step_model_pose, step_motor_rad)) {
            std::cerr << "[error] failed to build step motor targets at trial "
                      << trial << "\n";
            break;
        }

        if (!send_measured_step(controller, step_motor_rad, trial, events)) {
            break;
        }

        if (!target_builder.build(kStandPoseRad, stand_motor_rad)) {
            std::cerr << "[error] failed to rebuild stand motor targets after trial "
                      << trial << "\n";
            break;
        }
        if (!stream_target_for(controller, stand_motor_rad, kReturnHoldMs)) {
            break;
        }
    }

    g_recorder.set_recording(false);
    force_sleep_ms(40);

    std::cout << "[flow] stop all motors\n";
    controller.send_command(myactua::ControlCommand::Stop());
    force_sleep_ms(500);
    controller.shutdown();
    controller.set_status_callback({});
    controller_started = false;

    analyze_and_print_results(events);
    return g_should_stop ? 130 : 0;
}
