#include "motor_base/command_types.hpp"
#include "protocol/ethercat/ethercat_adapter.hpp"
#include "driver/myact/motor_control.hpp"
#include "driver/myact/motor_units.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <sched.h>
#include <thread>
#include <vector>

namespace {

class FakeAdapter : public myactua::EthercatAdapter {
public:
    explicit FakeAdapter(int motor_count)
        : motor_count_(motor_count)
    {
        default_health_.master_link_up = true;
        default_health_.wc_state = EC_WC_COMPLETE;
        default_health_.working_counter = static_cast<unsigned int>(motor_count_);
        current_health_ = default_health_;
        for (auto& rx : rx_) {
            rx = {};
            rx.status_word = myactua::BIT_READY_TO_SWITCH_ON;
            rx.op_mode = static_cast<int8_t>(myactua::MyactControlMode::CSP);
        }
    }

    bool init(const char*) override { return true; }

    void receive_physical() override
    {
        const std::thread::id thread_id = std::this_thread::get_id();
        {
            std::lock_guard<std::mutex> lock(thread_ids_mutex_);
            bool known = false;
            for (const auto& id : receiver_thread_ids_) {
                if (id == thread_id) {
                    known = true;
                    break;
                }
            }
            if (!known) {
                receiver_thread_ids_.push_back(thread_id);
            }
        }

        std::lock_guard<std::mutex> lock(health_mutex_);
        if (!health_script_.empty()) {
            current_health_ = health_script_.front();
            health_script_.pop_front();
        } else {
            current_health_ = default_health_;
        }
    }

    void send_physical() override
    {
        cycles_.fetch_add(1, std::memory_order_relaxed);
        cycles_cv_.notify_all();
    }

    void send(int index, const myactua::TxPDO& pdo) override
    {
        if (index >= 0 && index < motor_count_) {
            std::lock_guard<std::mutex> lock(tx_mutex_);
            tx_[static_cast<std::size_t>(index)] = pdo;
        }
    }

    myactua::RxPDO receive(int index) override
    {
        if (index >= 0 && index < motor_count_) {
            return rx_[static_cast<std::size_t>(index)];
        }
        return {};
    }

    bool is_configured(int index) override
    {
        return index >= 0 && index < motor_count_;
    }

    myactua::EthercatBusHealthSnapshot get_bus_health() const override
    {
        std::lock_guard<std::mutex> lock(health_mutex_);
        return current_health_;
    }

    std::uint64_t cycles() const
    {
        return cycles_.load(std::memory_order_relaxed);
    }

    bool wait_for_cycles(std::uint64_t target_cycles,
                         std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock(cycles_mutex_);
        return cycles_cv_.wait_for(lock, timeout, [this, target_cycles]() {
            return cycles_.load(std::memory_order_relaxed) >= target_cycles;
        });
    }

    std::size_t receiver_thread_count() const
    {
        std::lock_guard<std::mutex> lock(thread_ids_mutex_);
        return receiver_thread_ids_.size();
    }

    void set_health_script(
        const std::vector<myactua::EthercatBusHealthSnapshot>& script)
    {
        std::lock_guard<std::mutex> lock(health_mutex_);
        health_script_.clear();
        for (const auto& health : script) {
            health_script_.push_back(health);
        }
    }

    void set_rx_position(int index, int32_t raw_position)
    {
        if (index >= 0 && index < motor_count_) {
            rx_[static_cast<std::size_t>(index)].pos = raw_position;
        }
    }

    void set_rx_status_word(int index, uint16_t status_word)
    {
        if (index >= 0 && index < motor_count_) {
            rx_[static_cast<std::size_t>(index)].status_word = status_word;
        }
    }

    void set_rx_mode(int index, myactua::MyactControlMode mode)
    {
        if (index >= 0 && index < motor_count_) {
            rx_[static_cast<std::size_t>(index)].op_mode =
                static_cast<int8_t>(mode);
        }
    }

    myactua::TxPDO last_tx(int index) const
    {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        if (index >= 0 && index < motor_count_) {
            return tx_[static_cast<std::size_t>(index)];
        }
        return {};
    }

private:
    int motor_count_;
    std::array<myactua::RxPDO, motor_base::kMaxMotorCommandSetpoints> rx_{};
    std::array<myactua::TxPDO, motor_base::kMaxMotorCommandSetpoints> tx_{};
    std::atomic<std::uint64_t> cycles_{0};
    mutable std::mutex cycles_mutex_;
    std::condition_variable cycles_cv_;
    mutable std::mutex thread_ids_mutex_;
    std::vector<std::thread::id> receiver_thread_ids_;
    mutable std::mutex tx_mutex_;
    mutable std::mutex health_mutex_;
    myactua::EthercatBusHealthSnapshot default_health_;
    myactua::EthercatBusHealthSnapshot current_health_;
    std::deque<myactua::EthercatBusHealthSnapshot> health_script_;
};

bool expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "[motor_realtime_channel_test] " << message << "\n";
        return false;
    }
    return true;
}

myactua::EthercatBusHealthSnapshot health(
    bool link_up,
    ec_wc_state_t wc_state,
    unsigned int working_counter = 1)
{
    myactua::EthercatBusHealthSnapshot snapshot;
    snapshot.master_link_up = link_up;
    snapshot.wc_state = wc_state;
    snapshot.working_counter = working_counter;
    return snapshot;
}

uint16_t operation_enabled_status_word()
{
    return myactua::BIT_READY_TO_SWITCH_ON |
           myactua::BIT_SWITCHED_ON |
           myactua::BIT_OPERATION_ENABLED;
}

void append_health(
    std::vector<myactua::EthercatBusHealthSnapshot>& script,
    int count,
    const myactua::EthercatBusHealthSnapshot& snapshot)
{
    for (int i = 0; i < count; ++i) {
        script.push_back(snapshot);
    }
}

myactua::MYACTUA::Options test_options()
{
    myactua::MYACTUA::Options options;
    options.command_queue_capacity = 8;
    options.discrete_queue_capacity_per_motor = 4;
    options.rt_event_queue_capacity = 64;
    options.max_commands_per_cycle = 8;
    options.status_publish_period_ms = 1;
    options.rt_priority = 0;
    options.rt_period_ns = 1000000;
    return options;
}

bool expect_start(myactua::MYACTUA& controller, const char* message)
{
    if (!expect(controller.start(), message)) {
        controller.shutdown();
        return false;
    }
    return true;
}

} // namespace

int main()
{
    myactua::MYACTUA::Options options;
    options.command_queue_capacity = 3;
    options.discrete_queue_capacity_per_motor = 1;
    options.rt_event_queue_capacity = 32;
    options.max_commands_per_cycle = 8;
    options.status_publish_period_ms = 1;
    options.rt_priority = 0;

    auto adapter = std::make_shared<FakeAdapter>(1);
    myactua::MYACTUA controller(adapter, 1, options);

    std::vector<double> too_many(motor_base::kMaxMotorCommandSetpoints + 1, 0.0);
    const motor_base::CommandSubmitResult invalid_setpoint_result =
        controller.send_command(
            motor_base::ControlCommand::set_position_targets_rad(too_many));
    if (!expect(
            invalid_setpoint_result.status ==
                motor_base::CommandSubmitStatus::INVALID_PAYLOAD,
            "oversized setpoint payload should be rejected")) {
        return 1;
    }
    if (!expect(!invalid_setpoint_result.command_id.has_value(),
                "rejected setpoint should not carry command_id")) {
        return 1;
    }

    const motor_base::CommandSubmitResult setpoint_result =
        controller.send_command(
            motor_base::ControlCommand::set_velocity_targets_rad_s({0.0}));
    if (!expect(
            setpoint_result.status == motor_base::CommandSubmitStatus::ACCEPTED,
            "mode-dependent setpoint validation should be deferred to RT execution")) {
        return 1;
    }
    if (!expect(!setpoint_result.command_id.has_value(),
                "accepted setpoint should not carry command_id")) {
        return 1;
    }

    const motor_base::CommandSubmitResult stop_result =
        controller.send_command(motor_base::ControlCommand::stop());
    if (!expect(
            stop_result.status == motor_base::CommandSubmitStatus::ACCEPTED,
            "first command should be accepted")) {
        return 1;
    }
    if (!expect(stop_result.command_id.has_value(),
                "accepted STOP should carry command_id")) {
        return 1;
    }

    const motor_base::CommandSubmitResult restart_result =
        controller.send_command(motor_base::ControlCommand::restart());
    if (!expect(
            restart_result.status == motor_base::CommandSubmitStatus::ACCEPTED,
            "second command should be accepted")) {
        return 1;
    }
    if (!expect(restart_result.command_id.has_value(),
                "accepted RESTART should carry command_id")) {
        return 1;
    }
    if (!expect(
            controller.get_discrete_command_result(*restart_result.command_id) ==
                motor_base::DiscreteCommandResult::PENDING,
            "accepted RESTART should be pending before RT consumes it")) {
        return 1;
    }
    if (!expect(
            controller.get_discrete_command_result(999999) ==
                motor_base::DiscreteCommandResult::UNKNOWN,
            "unknown command_id should query as UNKNOWN")) {
        return 1;
    }

    const motor_base::CommandSubmitResult queue_full_result =
        controller.send_command(
            motor_base::ControlCommand::set_mode(
                motor_base::MotorControlMode::POSITION));
    if (!expect(
            queue_full_result.status ==
                motor_base::CommandSubmitStatus::QUEUE_FULL,
            "bounded command queue should report full")) {
        return 1;
    }
    if (!expect(!queue_full_result.command_id.has_value(),
                "queue-full command should not expose command_id")) {
        return 1;
    }

    std::atomic<int> status_callbacks{0};
    std::atomic<int> diagnostics_callbacks{0};
    std::atomic<int> discrete_queue_full_events{0};
    std::atomic<int> status_channel_busy_events{0};
    controller.set_status_callback(
        [&status_callbacks](const std::vector<motor_base::MotorStatusSnapshot>&) {
            status_callbacks.fetch_add(1, std::memory_order_relaxed);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        });
    controller.set_myact_diagnostics_callback(
        [&diagnostics_callbacks](const std::vector<myactua::MotorState>&) {
            diagnostics_callbacks.fetch_add(1, std::memory_order_relaxed);
        });
    controller.set_event_callback(
        [&discrete_queue_full_events,
         &status_channel_busy_events](const motor_base::RtEvent& event) {
            if (event.type == motor_base::RtEventType::DISCRETE_QUEUE_FULL) {
                discrete_queue_full_events.fetch_add(1, std::memory_order_relaxed);
            }
            if (event.type == motor_base::RtEventType::STATUS_CHANNEL_BUSY) {
                status_channel_busy_events.fetch_add(1, std::memory_order_relaxed);
            }
        });

    if (!expect(controller.start(), "first start should succeed in non-RT test mode")) {
        return 1;
    }
    if (!expect(controller.is_realtime_scheduling_ready(),
                "non-RT test mode should satisfy the scheduling prerequisite")) {
        controller.shutdown();
        return 1;
    }
    if (!expect(controller.start(), "second start should remain successful")) {
        controller.shutdown();
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(80));

    if (!expect(adapter->cycles() > 30,
                "slow status callback should not block the realtime loop")) {
        controller.shutdown();
        return 1;
    }
    if (!expect(adapter->receiver_thread_count() == 1,
                "double start should keep a single realtime thread")) {
        controller.shutdown();
        return 1;
    }
    if (!expect(status_callbacks.load(std::memory_order_relaxed) > 0,
                "status callback should be called by publisher thread")) {
        controller.shutdown();
        return 1;
    }
    if (!expect(diagnostics_callbacks.load(std::memory_order_relaxed) > 0,
                "diagnostics callback should be called by publisher thread")) {
        controller.shutdown();
        return 1;
    }
    std::array<motor_base::MotorStatusSnapshot,
               motor_base::kMaxMotorCommandSetpoints> policy_feedback;
    if (!expect(controller.try_consume_policy_feedback(policy_feedback),
                "policy feedback should be available")) {
        controller.shutdown();
        return 1;
    }
    if (!expect(policy_feedback[0].host_timestamp_ns != 0,
                "policy feedback timestamp must be populated by RT cycle")) {
        controller.shutdown();
        return 1;
    }
    const std::vector<motor_base::MotorStatusSnapshot> status =
        controller.get_status();
    if (!expect(!status.empty() && status[0].host_timestamp_ns != 0,
                "status snapshot timestamp must be populated by RT cycle")) {
        controller.shutdown();
        return 1;
    }
    if (!expect(discrete_queue_full_events.load(std::memory_order_relaxed) > 0,
                "discrete queue overflow should emit an RT event")) {
        controller.shutdown();
        return 1;
    }
    if (!expect(
            controller.get_discrete_command_result(*restart_result.command_id) ==
                motor_base::DiscreteCommandResult::FAILED,
            "per-motor discrete queue overflow should mark command_id failed")) {
        controller.shutdown();
        return 1;
    }
    if (!expect(status_channel_busy_events.load(std::memory_order_relaxed) == 0,
                "latest-value status supersedes should not emit busy events")) {
        controller.shutdown();
        return 1;
    }

    std::thread callback_toggler([&controller]() {
        for (int i = 0; i < 100; ++i) {
            controller.set_status_callback({});
            controller.set_status_callback(
                [](const std::vector<motor_base::MotorStatusSnapshot>&) {});
            controller.set_myact_diagnostics_callback({});
            controller.set_myact_diagnostics_callback(
                [](const std::vector<myactua::MotorState>&) {});
        }
    });
    callback_toggler.join();

    controller.shutdown();
    controller.shutdown();

    {
        myactua::MYACTUA::Options wrap_options = test_options();
        wrap_options.command_queue_capacity = 70;
        auto wrap_adapter = std::make_shared<FakeAdapter>(1);
        myactua::MYACTUA wrap_controller(wrap_adapter, 1, wrap_options);

        const motor_base::CommandSubmitResult first_result =
            wrap_controller.send_command(motor_base::ControlCommand::stop());
        if (!expect(
                first_result.status == motor_base::CommandSubmitStatus::ACCEPTED &&
                    first_result.command_id.has_value(),
                "first tracked command should submit before result-table wrap")) {
            return 1;
        }

        for (int i = 0; i < 64; ++i) {
            const motor_base::CommandSubmitResult result =
                wrap_controller.send_command(motor_base::ControlCommand::stop());
            if (!expect(
                    result.status == motor_base::CommandSubmitStatus::ACCEPTED &&
                        result.command_id.has_value(),
                    "tracked command should submit during result-table wrap")) {
                return 1;
            }
        }

        if (!expect(
                wrap_controller.get_discrete_command_result(*first_result.command_id) ==
                    motor_base::DiscreteCommandResult::UNKNOWN,
                "overwritten command_id should query as UNKNOWN")) {
            return 1;
        }
    }

    {
        myactua::MYACTUA::Options failing_options = test_options();
        const int max_fifo_priority = sched_get_priority_max(SCHED_FIFO);
        if (!expect(max_fifo_priority > 0,
                    "SCHED_FIFO max priority should be available")) {
            return 1;
        }
        failing_options.rt_priority = max_fifo_priority + 1;

        auto failing_adapter = std::make_shared<FakeAdapter>(1);
        myactua::MYACTUA failing_controller(failing_adapter, 1, failing_options);
        if (!expect(!failing_controller.start(),
                    "invalid realtime priority should make start return false")) {
            failing_controller.shutdown();
            return 1;
        }
        if (!expect(!failing_controller.is_running(),
                    "failed realtime scheduling should stop the RT thread")) {
            failing_controller.shutdown();
            return 1;
        }
        if (!expect(!failing_controller.is_realtime_scheduling_ready(),
                    "failed realtime scheduling should leave rt ready false")) {
            failing_controller.shutdown();
            return 1;
        }
        if (!expect(
                failing_controller.send_command(
                    motor_base::ControlCommand::restart()).status ==
                    motor_base::CommandSubmitStatus::INVALID_COMMAND,
                "RESTART should be rejected when required RT scheduling is inactive")) {
            failing_controller.shutdown();
            return 1;
        }
        if (!expect(
                failing_controller.send_command(
                    motor_base::ControlCommand::set_position_targets_rad({0.0})).status ==
                    motor_base::CommandSubmitStatus::INVALID_COMMAND,
                "setpoint should be rejected when required RT scheduling is inactive")) {
            failing_controller.shutdown();
            return 1;
        }
        if (!expect(
                failing_controller.send_command(
                    motor_base::ControlCommand::stop()).status ==
                    motor_base::CommandSubmitStatus::ACCEPTED,
                "STOP should remain accepted when RT scheduling is inactive")) {
            failing_controller.shutdown();
            return 1;
        }
        if (!expect(
                failing_controller.send_command(
                    motor_base::ControlCommand::set_mode(
                        motor_base::MotorControlMode::POSITION)).status ==
                    motor_base::CommandSubmitStatus::ACCEPTED,
                "SET_MODE should remain accepted when RT scheduling is inactive")) {
            failing_controller.shutdown();
            return 1;
        }
        failing_controller.shutdown();
    }

    {
        auto adapter = std::make_shared<FakeAdapter>(1);
        adapter->set_rx_status_word(0, operation_enabled_status_word());
        adapter->set_rx_mode(0, myactua::MyactControlMode::CSP);

        myactua::MYACTUA mode_controller(adapter, 1, test_options());
        std::atomic<int> reject_events{0};
        std::atomic<int> reject_motor{-2};
        std::atomic<int> reject_reason{0};
        std::atomic<uint32_t> reject_value{0};
        mode_controller.set_event_callback(
            [&reject_events, &reject_motor, &reject_reason, &reject_value](
                const motor_base::RtEvent& event) {
                if (event.type ==
                    motor_base::RtEventType::SETPOINT_COMMAND_REJECTED) {
                    reject_events.fetch_add(1, std::memory_order_relaxed);
                    reject_motor.store(event.motor_index, std::memory_order_relaxed);
                    reject_reason.store(event.reason, std::memory_order_relaxed);
                    reject_value.store(event.value, std::memory_order_relaxed);
                }
            });

        if (!expect_start(mode_controller,
                          "target-mode-not-confirmed controller should start")) {
            return 1;
        }
        if (!expect(
                mode_controller.send_command(
                    motor_base::ControlCommand::set_mode(
                        motor_base::MotorControlMode::IMPEDANCE)).status ==
                    motor_base::CommandSubmitStatus::ACCEPTED,
                "SET_MODE should be accepted before confirmed-mode test")) {
            mode_controller.shutdown();
            return 1;
        }
        if (!expect(adapter->wait_for_cycles(6, std::chrono::seconds(1)),
                    "SET_MODE should be processed before confirmed-mode test")) {
            mode_controller.shutdown();
            return 1;
        }

        constexpr int32_t target_raw = 2222;
        std::vector<motor_base::ImpedanceSetpoint> setpoints;
        setpoints.emplace_back(
            static_cast<double>(target_raw) * myactua::kRawPosToRad,
            0.0,
            0.0,
            10.0,
            1.0);
        if (!expect(
                mode_controller.send_command(
                    motor_base::ControlCommand::set_impedance_targets(setpoints)).status ==
                    motor_base::CommandSubmitStatus::ACCEPTED,
                "setpoint should enqueue for RT confirmed-mode validation")) {
            mode_controller.shutdown();
            return 1;
        }
        if (!expect(adapter->wait_for_cycles(12, std::chrono::seconds(1)),
                    "unconfirmed target-mode setpoint scenario should run")) {
            mode_controller.shutdown();
            return 1;
        }
        mode_controller.shutdown();

        const myactua::TxPDO tx = adapter->last_tx(0);
        if (!expect(reject_events.load(std::memory_order_relaxed) == 1,
                    "target mode without observed confirmation should reject setpoint in RT")) {
            return 1;
        }
        if (!expect(reject_motor.load(std::memory_order_relaxed) == 0 &&
                        reject_reason.load(std::memory_order_relaxed) ==
                            static_cast<int>(
                                motor_base::SetpointRejectReason::MODE_NOT_CONFIRMED) &&
                        reject_value.load(std::memory_order_relaxed) ==
                            static_cast<uint32_t>(
                                motor_base::SetpointCommandType::IMPEDANCE_TARGETS),
                    "setpoint rejection event should describe the unconfirmed motor and type")) {
            return 1;
        }
        if (!expect(tx.target_pos != target_raw,
                    "rejected impedance setpoint should not update target position")) {
            return 1;
        }
    }

    {
        auto adapter = std::make_shared<FakeAdapter>(1);
        adapter->set_rx_status_word(0, myactua::BIT_READY_TO_SWITCH_ON);
        adapter->set_rx_mode(0, myactua::MyactControlMode::CSP);

        myactua::MYACTUA stopped_controller(adapter, 1, test_options());
        std::atomic<int> reject_events{0};
        stopped_controller.set_event_callback(
            [&reject_events](const motor_base::RtEvent& event) {
                if (event.type ==
                    motor_base::RtEventType::SETPOINT_COMMAND_REJECTED) {
                    reject_events.fetch_add(1, std::memory_order_relaxed);
                }
            });

        if (!expect_start(stopped_controller,
                          "not-running confirmed-mode controller should start")) {
            return 1;
        }
        if (!expect(adapter->wait_for_cycles(4, std::chrono::seconds(1)),
                    "not-running confirmed-mode scenario should settle")) {
            stopped_controller.shutdown();
            return 1;
        }

        constexpr int32_t target_raw = 3333;
        if (!expect(
                stopped_controller.send_command(
                    motor_base::ControlCommand::set_position_targets_rad(
                        {static_cast<double>(target_raw) * myactua::kRawPosToRad})).status ==
                    motor_base::CommandSubmitStatus::ACCEPTED,
                "setpoint should enqueue when observed mode matches but motor is not running")) {
            stopped_controller.shutdown();
            return 1;
        }
        if (!expect(adapter->wait_for_cycles(10, std::chrono::seconds(1)),
                    "not-running setpoint rejection scenario should run")) {
            stopped_controller.shutdown();
            return 1;
        }
        stopped_controller.shutdown();

        if (!expect(reject_events.load(std::memory_order_relaxed) == 1,
                    "matching observed mode without RUNNING state should reject setpoint")) {
            return 1;
        }
        if (!expect(adapter->last_tx(0).target_pos != target_raw,
                    "not-running rejection should not update target position")) {
            return 1;
        }
    }

    {
        auto adapter = std::make_shared<FakeAdapter>(1);
        adapter->set_rx_status_word(0, operation_enabled_status_word());
        adapter->set_rx_mode(0, myactua::MyactControlMode::CSP);

        myactua::MYACTUA running_controller(adapter, 1, test_options());
        std::atomic<int> reject_events{0};
        running_controller.set_event_callback(
            [&reject_events](const motor_base::RtEvent& event) {
                if (event.type ==
                    motor_base::RtEventType::SETPOINT_COMMAND_REJECTED) {
                    reject_events.fetch_add(1, std::memory_order_relaxed);
                }
            });

        if (!expect_start(running_controller,
                          "confirmed-running setpoint controller should start")) {
            return 1;
        }
        const motor_base::CommandSubmitResult restart_submit =
            running_controller.send_command(motor_base::ControlCommand::restart());
        if (!expect(
                restart_submit.status == motor_base::CommandSubmitStatus::ACCEPTED &&
                    restart_submit.command_id.has_value(),
                "RESTART should be accepted with command_id")) {
            running_controller.shutdown();
            return 1;
        }
        if (!expect(
                running_controller.get_discrete_command_result(*restart_submit.command_id) ==
                    motor_base::DiscreteCommandResult::PENDING,
                "RESTART command_id should start pending")) {
            running_controller.shutdown();
            return 1;
        }
        if (!expect(adapter->wait_for_cycles(60, std::chrono::seconds(1)),
                    "confirmed-running controller should reach RUNNING")) {
            running_controller.shutdown();
            return 1;
        }
        if (!expect(
                running_controller.get_discrete_command_result(*restart_submit.command_id) ==
                    motor_base::DiscreteCommandResult::SUCCEEDED,
                "RESTART command_id should succeed after control_ready")) {
            running_controller.shutdown();
            return 1;
        }

        constexpr int32_t target_raw = 4444;
        if (!expect(
                running_controller.send_command(
                    motor_base::ControlCommand::set_position_targets_rad(
                        {static_cast<double>(target_raw) * myactua::kRawPosToRad})).status ==
                    motor_base::CommandSubmitStatus::ACCEPTED,
                "setpoint should enqueue when all motors are confirmed running")) {
            running_controller.shutdown();
            return 1;
        }
        if (!expect(adapter->wait_for_cycles(adapter->cycles() + 12,
                                             std::chrono::seconds(1)),
                    "confirmed-running setpoint scenario should run")) {
            running_controller.shutdown();
            return 1;
        }
        running_controller.shutdown();

        if (!expect(reject_events.load(std::memory_order_relaxed) == 0,
                    "confirmed running mode should not reject setpoint")) {
            return 1;
        }
        if (!expect(adapter->last_tx(0).target_pos == target_raw,
                    "confirmed running setpoint should update target position")) {
            return 1;
        }
    }

    {
        auto adapter = std::make_shared<FakeAdapter>(2);
        adapter->set_rx_status_word(0, operation_enabled_status_word());
        adapter->set_rx_status_word(1, operation_enabled_status_word());
        adapter->set_rx_mode(0, myactua::MyactControlMode::CSP);
        adapter->set_rx_mode(1, myactua::MyactControlMode::CSP);

        myactua::MYACTUA all_ready_controller(adapter, 2, test_options());
        if (!expect_start(all_ready_controller,
                          "all-ready restart controller should start")) {
            return 1;
        }
        const motor_base::CommandSubmitResult restart_submit =
            all_ready_controller.send_command(motor_base::ControlCommand::restart());
        if (!expect(
                restart_submit.status == motor_base::CommandSubmitStatus::ACCEPTED &&
                    restart_submit.command_id.has_value(),
                "all-motors RESTART should be accepted with command_id")) {
            all_ready_controller.shutdown();
            return 1;
        }
        if (!expect(adapter->wait_for_cycles(60, std::chrono::seconds(1)),
                    "all-ready restart controller should run")) {
            all_ready_controller.shutdown();
            return 1;
        }
        if (!expect(
                all_ready_controller.get_discrete_command_result(
                    *restart_submit.command_id) ==
                    motor_base::DiscreteCommandResult::SUCCEEDED,
                "all-motors RESTART should succeed after every motor is ready")) {
            all_ready_controller.shutdown();
            return 1;
        }
        all_ready_controller.shutdown();
    }

    {
        auto adapter = std::make_shared<FakeAdapter>(2);
        adapter->set_rx_status_word(0, operation_enabled_status_word());
        adapter->set_rx_status_word(1, myactua::BIT_READY_TO_SWITCH_ON);
        adapter->set_rx_mode(0, myactua::MyactControlMode::CSP);
        adapter->set_rx_mode(1, myactua::MyactControlMode::CSP);

        myactua::MYACTUA partial_controller(adapter, 2, test_options());
        std::atomic<int> reject_events{0};
        std::atomic<int> reject_motor{-2};
        partial_controller.set_event_callback(
            [&reject_events, &reject_motor](const motor_base::RtEvent& event) {
                if (event.type ==
                    motor_base::RtEventType::SETPOINT_COMMAND_REJECTED) {
                    reject_events.fetch_add(1, std::memory_order_relaxed);
                    reject_motor.store(event.motor_index, std::memory_order_relaxed);
                }
            });

        if (!expect_start(partial_controller,
                          "partial-frame setpoint controller should start")) {
            return 1;
        }
        const motor_base::CommandSubmitResult partial_restart_submit =
            partial_controller.send_command(motor_base::ControlCommand::restart());
        if (!expect(
                partial_restart_submit.status ==
                    motor_base::CommandSubmitStatus::ACCEPTED &&
                    partial_restart_submit.command_id.has_value(),
                "partial-frame RESTART should be accepted with command_id")) {
            partial_controller.shutdown();
            return 1;
        }
        if (!expect(adapter->wait_for_cycles(6, std::chrono::seconds(1)),
                    "partial-frame controller should process restart")) {
            partial_controller.shutdown();
            return 1;
        }
        if (!expect(
                partial_controller.get_discrete_command_result(
                    *partial_restart_submit.command_id) ==
                    motor_base::DiscreteCommandResult::PENDING,
                "all-motors RESTART should stay pending until every target motor is ready")) {
            partial_controller.shutdown();
            return 1;
        }

        constexpr int32_t target0_raw = 5555;
        constexpr int32_t target1_raw = 6666;
        if (!expect(
                partial_controller.send_command(
                    motor_base::ControlCommand::set_position_targets_rad(
                        {static_cast<double>(target0_raw) * myactua::kRawPosToRad,
                         static_cast<double>(target1_raw) * myactua::kRawPosToRad})).status ==
                    motor_base::CommandSubmitStatus::ACCEPTED,
                "partial-frame setpoint should enqueue for RT validation")) {
            partial_controller.shutdown();
            return 1;
        }
        if (!expect(adapter->wait_for_cycles(12, std::chrono::seconds(1)),
                    "partial-frame rejection scenario should run")) {
            partial_controller.shutdown();
            return 1;
        }
        partial_controller.shutdown();

        if (!expect(reject_events.load(std::memory_order_relaxed) == 1 &&
                        reject_motor.load(std::memory_order_relaxed) == 1,
                    "first unconfirmed motor should reject the whole setpoint frame")) {
            return 1;
        }
        if (!expect(adapter->last_tx(0).target_pos != target0_raw &&
                        adapter->last_tx(1).target_pos != target1_raw,
                    "rejected setpoint frame should not partially update targets")) {
            return 1;
        }
    }

      {
          auto adapter = std::make_shared<FakeAdapter>(1);
          std::vector<myactua::EthercatBusHealthSnapshot> script;
          append_health(script, 9, health(true, EC_WC_INCOMPLETE, 0));
          adapter->set_health_script(script);

          myactua::MYACTUA watchdog_controller(adapter, 1, test_options());
          std::atomic<int> fault_events{0};
          watchdog_controller.set_event_callback(
              [&fault_events](const motor_base::RtEvent& event) {
                  if (event.type == motor_base::RtEventType::COMM_WATCHDOG_FAULT) {
                      fault_events.fetch_add(1, std::memory_order_relaxed);
                  }
              });
          if (!expect_start(watchdog_controller,
                            "watchdog 9-bad-cycle controller should start")) {
              return 1;
          }
          if (!expect(adapter->wait_for_cycles(20, std::chrono::seconds(1)),
                      "watchdog 9-bad-cycle scenario should complete")) {
              watchdog_controller.shutdown();
              return 1;
          }
          watchdog_controller.shutdown();
          if (!expect(fault_events.load(std::memory_order_relaxed) == 0,
                      "9 consecutive bad process-data cycles should not latch")) {
              return 1;
          }
      }

      {
          auto adapter = std::make_shared<FakeAdapter>(1);
          std::vector<myactua::EthercatBusHealthSnapshot> script;
          append_health(script, 5, health(true, EC_WC_INCOMPLETE, 0));
          append_health(script, 1, health(true, EC_WC_COMPLETE, 1));
          append_health(script, 5, health(true, EC_WC_INCOMPLETE, 0));
          adapter->set_health_script(script);

          myactua::MYACTUA watchdog_controller(adapter, 1, test_options());
          std::atomic<int> fault_events{0};
          watchdog_controller.set_event_callback(
              [&fault_events](const motor_base::RtEvent& event) {
                  if (event.type == motor_base::RtEventType::COMM_WATCHDOG_FAULT) {
                      fault_events.fetch_add(1, std::memory_order_relaxed);
                  }
              });
          if (!expect_start(watchdog_controller,
                            "watchdog reset-counter controller should start")) {
              return 1;
          }
          if (!expect(adapter->wait_for_cycles(20, std::chrono::seconds(1)),
                      "watchdog reset-counter scenario should complete")) {
              watchdog_controller.shutdown();
              return 1;
          }
          watchdog_controller.shutdown();
          if (!expect(fault_events.load(std::memory_order_relaxed) == 0,
                      "a good process-data cycle should reset the fail counter")) {
              return 1;
          }
      }

      {
          auto adapter = std::make_shared<FakeAdapter>(1);
          std::vector<myactua::EthercatBusHealthSnapshot> script;
          append_health(script, 10, health(true, EC_WC_INCOMPLETE, 0));
          adapter->set_health_script(script);

          myactua::MYACTUA watchdog_controller(adapter, 1, test_options());
          std::atomic<int> fault_events{0};
          std::atomic<int> last_reason{0};
          watchdog_controller.set_event_callback(
              [&fault_events, &last_reason](
                  const motor_base::RtEvent& event) {
                  if (event.type == motor_base::RtEventType::COMM_WATCHDOG_FAULT) {
                      fault_events.fetch_add(1, std::memory_order_relaxed);
                      last_reason.store(event.reason, std::memory_order_relaxed);
                  }
              });
          if (!expect_start(watchdog_controller,
                            "watchdog WKC fault controller should start")) {
              return 1;
          }
          if (!expect(adapter->wait_for_cycles(24, std::chrono::seconds(1)),
                      "watchdog WKC fault scenario should complete")) {
              watchdog_controller.shutdown();
              return 1;
          }
          if (!expect(
                  watchdog_controller.send_command(
                      motor_base::ControlCommand::set_position_targets_rad({0.0})).status ==
                      motor_base::CommandSubmitStatus::INVALID_COMMAND,
                  "setpoint commands should be rejected while communication fault is latched")) {
              watchdog_controller.shutdown();
              return 1;
          }
          if (!expect(
                  watchdog_controller.send_command(
                      motor_base::ControlCommand::set_mode(
                          motor_base::MotorControlMode::POSITION)).status ==
                      motor_base::CommandSubmitStatus::INVALID_COMMAND,
                  "SET_MODE should be rejected while communication fault is latched")) {
              watchdog_controller.shutdown();
              return 1;
          }
          watchdog_controller.shutdown();

          const myactua::TxPDO tx = adapter->last_tx(0);
          if (!expect(fault_events.load(std::memory_order_relaxed) == 1,
                      "10 consecutive WKC failures should latch once")) {
              return 1;
          }
          if (!expect(last_reason.load(std::memory_order_relaxed) ==
                          static_cast<int>(myactua::MyactCommunicationFaultReason::WkcIncomplete),
                      "WKC fault should report WkcIncomplete reason")) {
              return 1;
          }
          if (!expect(tx.control_word == myactua::CMD_QUICK_STOP &&
                          tx.target_vel == 0 &&
                          tx.target_torque == 0 &&
                          tx.pvt_kp == 0 &&
                          tx.pvt_kd == 0,
                      "latched communication fault should output safe quick-stop PDO")) {
              return 1;
          }
      }

      {
          auto adapter = std::make_shared<FakeAdapter>(1);
          std::vector<myactua::EthercatBusHealthSnapshot> script;
          append_health(script, 10, health(false, EC_WC_COMPLETE, 0));
          adapter->set_health_script(script);

          myactua::MYACTUA watchdog_controller(adapter, 1, test_options());
          std::atomic<int> last_reason{0};
          watchdog_controller.set_event_callback(
              [&last_reason](const motor_base::RtEvent& event) {
                  if (event.type == motor_base::RtEventType::COMM_WATCHDOG_FAULT) {
                      last_reason.store(event.reason, std::memory_order_relaxed);
                  }
              });
          if (!expect_start(watchdog_controller,
                            "watchdog link-down controller should start")) {
              return 1;
          }
          if (!expect(adapter->wait_for_cycles(12, std::chrono::seconds(1)),
                      "watchdog link-down scenario should complete")) {
              watchdog_controller.shutdown();
              return 1;
          }
          watchdog_controller.shutdown();
          if (!expect(last_reason.load(std::memory_order_relaxed) ==
                          static_cast<int>(myactua::MyactCommunicationFaultReason::LinkDown),
                      "link-down fault should report LinkDown reason")) {
              return 1;
          }
      }

      {
          auto adapter = std::make_shared<FakeAdapter>(1);
          std::vector<myactua::EthercatBusHealthSnapshot> script;
          append_health(script, 10, health(true, EC_WC_INCOMPLETE, 0));
          adapter->set_health_script(script);

          myactua::MYACTUA watchdog_controller(adapter, 1, test_options());
          if (!expect_start(watchdog_controller,
                            "single-motor restart controller should start")) {
              return 1;
          }
          if (!expect(adapter->wait_for_cycles(12, std::chrono::seconds(1)),
                      "single-motor restart scenario should reach fault")) {
              watchdog_controller.shutdown();
              return 1;
          }
          const motor_base::CommandSubmitResult restart_submit =
              watchdog_controller.send_command(motor_base::ControlCommand::restart(0));
          if (!expect(
                  restart_submit.status == motor_base::CommandSubmitStatus::ACCEPTED &&
                      restart_submit.command_id.has_value(),
                  "single-motor RESTART under fault should submit with command_id")) {
              watchdog_controller.shutdown();
              return 1;
          }
          if (!expect(adapter->wait_for_cycles(adapter->cycles() + 32,
                                               std::chrono::seconds(1)),
                      "single-motor restart scenario should continue running")) {
              watchdog_controller.shutdown();
              return 1;
          }
          if (!expect(
                  watchdog_controller.get_discrete_command_result(
                      *restart_submit.command_id) ==
                      motor_base::DiscreteCommandResult::FAILED,
                  "single-motor RESTART under latched fault should fail")) {
              watchdog_controller.shutdown();
              return 1;
          }
          watchdog_controller.shutdown();
          if (!expect(adapter->last_tx(0).control_word == myactua::CMD_QUICK_STOP,
                      "RESTART(i) should keep communication fault latched")) {
              return 1;
          }
      }

      {
          auto adapter = std::make_shared<FakeAdapter>(1);
          std::vector<myactua::EthercatBusHealthSnapshot> script;
          append_health(script, 10, health(true, EC_WC_INCOMPLETE, 0));
          adapter->set_health_script(script);

          myactua::MYACTUA watchdog_controller(adapter, 1, test_options());
          std::atomic<int> fault_events{0};
          watchdog_controller.set_event_callback(
              [&fault_events](const motor_base::RtEvent& event) {
                  if (event.type == motor_base::RtEventType::COMM_WATCHDOG_FAULT) {
                      fault_events.fetch_add(1, std::memory_order_relaxed);
                  }
              });
          if (!expect_start(watchdog_controller,
                            "RESTART(-1) latch controller should start")) {
              return 1;
          }
          if (!expect(adapter->wait_for_cycles(12, std::chrono::seconds(1)),
                      "RESTART(-1) latch scenario should reach fault")) {
              watchdog_controller.shutdown();
              return 1;
          }

          std::vector<myactua::EthercatBusHealthSnapshot> recovery_script;
          append_health(recovery_script, 20, health(true, EC_WC_COMPLETE, 1));
          adapter->set_health_script(recovery_script);
          watchdog_controller.send_command(motor_base::ControlCommand::restart());
          if (!expect(adapter->wait_for_cycles(28, std::chrono::seconds(1)),
                      "healthy cycles after RESTART(-1) should continue running")) {
              watchdog_controller.shutdown();
              return 1;
          }
          watchdog_controller.shutdown();
          if (!expect(fault_events.load(std::memory_order_relaxed) == 1 &&
                          adapter->last_tx(0).control_word == myactua::CMD_QUICK_STOP,
                      "RESTART(-1) and healthy cycles should keep communication fault latched")) {
              return 1;
          }
      }

      {
          auto adapter = std::make_shared<FakeAdapter>(1);
          std::vector<myactua::EthercatBusHealthSnapshot> script;
          append_health(script, 10, health(true, EC_WC_INCOMPLETE, 0));
          adapter->set_health_script(script);

          myactua::MYACTUA watchdog_controller(adapter, 1, test_options());
          if (!expect_start(watchdog_controller,
                            "RT restart latch controller should start")) {
              return 1;
          }
          if (!expect(adapter->wait_for_cycles(12, std::chrono::seconds(1)),
                      "RT restart latch scenario should reach fault")) {
              watchdog_controller.shutdown();
              return 1;
          }
          watchdog_controller.shutdown();

          if (!expect_start(watchdog_controller,
                            "RT restart latch controller should restart")) {
              return 1;
          }
          if (!expect(adapter->wait_for_cycles(18, std::chrono::seconds(1)),
                      "RT restart latch scenario should run again")) {
              watchdog_controller.shutdown();
              return 1;
          }
          watchdog_controller.shutdown();

          const myactua::TxPDO tx = adapter->last_tx(0);
          if (!expect(tx.control_word == myactua::CMD_QUICK_STOP,
                      "RT thread restart should not clear communication latch")) {
              return 1;
          }
      }

      return 0;
  }
