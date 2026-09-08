#include "motor_base/command_types.hpp"
#include "protocol/ethercat/ethercat_adapter.hpp"
#include "driver/myact/myact_motor_controller.hpp"
#include "driver/myact/motor_units.hpp"
#include "tool/tool.hpp"

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

motor_base::ControlCommand timed_policy_position(double position,
                                                  std::uint64_t policy_seq = 1)
{
    motor_base::ControlCommand command =
        motor_base::ControlCommand::set_position_targets_rad({position});
    const std::int64_t now_ns = robot_base::monotonic_now_ns();
    command.timing.source_policy_seq = policy_seq;
    command.timing.produced_at_ns = now_ns;
    command.timing.valid_until_ns = now_ns + 1'000'000'000LL;
    return command;
}

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

myactua::MyActMotorController::Options test_options()
{
    myactua::MyActMotorController::Options options;
    options.command_queue_capacity = 8;
    options.discrete_queue_capacity_per_motor = 4;
    options.rt_event_queue_capacity = 64;
    options.max_commands_per_cycle = 8;
    options.status_publish_period_ms = 1;
    options.rt_priority = 0;
    options.rt_period_ns = 1000000;
    options.setpoint_timeout_ns = 1'000'000'000;
    return options;
}

bool expect_start(myactua::MyActMotorController& controller, const char* message)
{
    if (!expect(controller.start(), message)) {
        controller.shutdown();
        return false;
    }
    return true;
}

class RecordingMotorController : public motor_base::MotorControllerBase {
public:
    explicit RecordingMotorController(
        const motor_base::MotorControllerBase::RealtimeOptions& options)
        : MotorControllerBase(1, options)
    {
    }

    bool wait_all_motors_ready(int, int) const override { return true; }

    void set_print_info(const std::vector<int>&) override {}

    bool wait_for_cycles(std::uint64_t target_cycles,
                         std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [this, target_cycles]() {
            return cycles_ >= target_cycles;
        });
    }

    bool wait_for_applied_count(std::size_t target_count,
                                std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [this, target_count]() {
            return applied_setpoints_.size() >= target_count;
        });
    }

    std::uint64_t cycles() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return cycles_;
    }

    std::vector<motor_base::ControlCommand> applied_setpoints() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return applied_setpoints_;
    }

    std::size_t safety_stop_count() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return safety_stop_count_;
    }

protected:
    bool connect_impl(const char*) override { return true; }

    void realtime_cycle_callback() override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++cycles_;
        cv_.notify_all();
    }

    void apply_setpoint_command_impl(
        const motor_base::ControlCommand& cmd) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        applied_setpoints_.push_back(cmd);
        cv_.notify_all();
    }

    void apply_discrete_command_impl(
        int,
        const motor_base::DiscreteCommand& cmd) override
    {
        if (cmd.type == motor_base::DiscreteCommandType::STOP) {
            std::lock_guard<std::mutex> lock(mutex_);
            ++safety_stop_count_;
            cv_.notify_all();
        }
    }

    motor_base::DiscreteCommandEvaluation evaluate_discrete_command_impl(
        int,
        const motor_base::DiscreteCommand&) const override
    {
        return motor_base::DiscreteCommandEvaluation::SATISFIED;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::uint64_t cycles_{0};
    std::vector<motor_base::ControlCommand> applied_setpoints_;
    std::size_t safety_stop_count_{0};
};

motor_base::MotorControllerBase::RealtimeOptions latest_channel_test_options()
{
    motor_base::MotorControllerBase::RealtimeOptions options;
    options.command_queue_capacity = 4;
    options.discrete_queue_capacity_per_motor = 4;
    options.max_commands_per_cycle = 4;
    options.rt_period_ns = 50000000;
    options.rt_priority = 0;
    options.setpoint_timeout_ns = 200'000'000;
    options.rt_event_queue_capacity = 16;
    options.status_publish_period_ms = 1;
    return options;
}

} // namespace

int main()
{
    myactua::MyActMotorController::Options options;
    options.command_queue_capacity = 3;
    options.discrete_queue_capacity_per_motor = 1;
    options.rt_event_queue_capacity = 32;
    options.max_commands_per_cycle = 8;
    options.status_publish_period_ms = 1;
    options.rt_priority = 0;

    auto adapter = std::make_shared<FakeAdapter>(1);
    myactua::MyActMotorController controller(adapter, 1, options);

    const motor_base::CommandSubmitResult setpoint_as_discrete_result =
        controller.send_discrete_command(
            motor_base::ControlCommand::set_velocity_targets_rad_s({0.0}));
    if (!expect(
            setpoint_as_discrete_result.status ==
                motor_base::CommandSubmitStatus::INVALID_COMMAND,
            "discrete command API should reject setpoint commands")) {
        return 1;
    }

    const motor_base::CommandSubmitResult inactive_debug_result =
        controller.send_debug_setpoint(
            motor_base::ControlCommand::set_velocity_targets_rad_s({0.0}));
    if (!expect(
            inactive_debug_result.status ==
                motor_base::CommandSubmitStatus::SOURCE_INACTIVE,
            "debug setpoint should be rejected while policy source is active")) {
        return 1;
    }

    controller.set_active_setpoint_source(motor_base::SetpointSource::DEBUG);

    std::vector<double> too_many(motor_base::kMaxMotorCommandSetpoints + 1, 0.0);
    const motor_base::CommandSubmitResult invalid_setpoint_result =
        controller.send_debug_setpoint(
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
        controller.send_debug_setpoint(
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
        controller.send_discrete_command(motor_base::ControlCommand::stop());
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
        controller.send_discrete_command(motor_base::ControlCommand::restart());
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

    const motor_base::CommandSubmitResult set_mode_result =
        controller.send_discrete_command(motor_base::ControlCommand::set_mode(
                motor_base::MotorControlMode::POSITION));
    if (!expect(
            set_mode_result.status == motor_base::CommandSubmitStatus::ACCEPTED,
            "third discrete command should fill the bounded command queue")) {
        return 1;
    }
    if (!expect(set_mode_result.command_id.has_value(),
                "accepted SET_MODE should carry command_id")) {
        return 1;
    }

    const motor_base::CommandSubmitResult queue_full_result =
        controller.send_discrete_command(motor_base::ControlCommand::set_mode(
                motor_base::MotorControlMode::VELOCITY));
    if (!expect(
            queue_full_result.status ==
                motor_base::CommandSubmitStatus::QUEUE_FULL,
            "bounded discrete command queue should report full")) {
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
    if (!expect(controller.try_consume_latest_status_policy(policy_feedback),
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
        RecordingMotorController policy_controller(latest_channel_test_options());
        if (!expect(
                policy_controller.send_debug_setpoint(
                    motor_base::ControlCommand::set_position_targets_rad(
                        {1.0})).status ==
                    motor_base::CommandSubmitStatus::SOURCE_INACTIVE,
                "debug setpoint should be inactive when policy source is selected")) {
            return 1;
        }
        if (!expect(policy_controller.start(),
                    "default policy setpoint controller should start")) {
            return 1;
        }
        if (!expect(policy_controller.wait_for_cycles(1, std::chrono::seconds(1)),
                    "default policy controller should complete initial cycle")) {
            policy_controller.shutdown();
            return 1;
        }
        if (!expect(
                policy_controller.send_policy_setpoint(
                    motor_base::ControlCommand::set_position_targets_rad(
                        {1.5})).status ==
                    motor_base::CommandSubmitStatus::INVALID_COMMAND,
                "policy setpoint without freshness metadata should be rejected")) {
            policy_controller.shutdown();
            return 1;
        }
        if (!expect(
                policy_controller.send_policy_setpoint(
                    timed_policy_position(2.0)).status ==
                    motor_base::CommandSubmitStatus::ACCEPTED,
                "policy setpoint should publish when policy source is active")) {
            policy_controller.shutdown();
            return 1;
        }
        if (!expect(policy_controller.wait_for_applied_count(
                        1,
                        std::chrono::seconds(1)),
                    "policy setpoint should be consumed by default")) {
            policy_controller.shutdown();
            return 1;
        }
        policy_controller.shutdown();

        const std::vector<motor_base::ControlCommand> applied =
            policy_controller.applied_setpoints();
        if (!expect(applied.size() == 1 && applied[0].setpoints[0] == 2.0,
                    "default source should consume only policy setpoints")) {
            return 1;
        }
    }

    {
        RecordingMotorController debug_controller(latest_channel_test_options());
        debug_controller.set_active_setpoint_source(motor_base::SetpointSource::DEBUG);
        if (!expect(
                debug_controller.send_debug_setpoint(
                    motor_base::ControlCommand::set_position_targets_rad(
                        {5.0})).status ==
                    motor_base::CommandSubmitStatus::ACCEPTED,
                "debug setpoint before start should publish to debug channel")) {
            return 1;
        }
        if (!expect(debug_controller.start(),
                    "debug setpoint controller should start")) {
            return 1;
        }
        if (!expect(debug_controller.wait_for_cycles(2, std::chrono::seconds(1)),
                    "debug controller should run after start reset")) {
            debug_controller.shutdown();
            return 1;
        }
        if (!expect(debug_controller.applied_setpoints().empty(),
                    "start should clear stale debug setpoint data")) {
            debug_controller.shutdown();
            return 1;
        }

        debug_controller.set_active_setpoint_source(motor_base::SetpointSource::POLICY);
        if (!expect(
                debug_controller.send_policy_setpoint(
                    motor_base::ControlCommand::set_position_targets_rad(
                        {6.0})).status ==
                    motor_base::CommandSubmitStatus::SOURCE_INACTIVE,
                "running setpoint source switch should not activate policy source")) {
            debug_controller.shutdown();
            return 1;
        }
        if (!expect(
                debug_controller.send_debug_setpoint(
                    motor_base::ControlCommand::set_position_targets_rad(
                        {7.0})).status ==
                    motor_base::CommandSubmitStatus::ACCEPTED,
                "debug setpoint should remain active after ignored runtime switch")) {
            debug_controller.shutdown();
            return 1;
        }
        if (!expect(
                debug_controller.send_debug_setpoint(
                    motor_base::ControlCommand::set_position_targets_rad(
                        {8.0})).status ==
                    motor_base::CommandSubmitStatus::ACCEPTED,
                "newer debug setpoint should overwrite older debug setpoint")) {
            debug_controller.shutdown();
            return 1;
        }
        if (!expect(debug_controller.wait_for_applied_count(
                        1,
                        std::chrono::seconds(1)),
                    "debug latest setpoint should be consumed")) {
            debug_controller.shutdown();
            return 1;
        }
        if (!expect(debug_controller.wait_for_cycles(
                        debug_controller.cycles() + 2,
                        std::chrono::seconds(1)),
                    "debug latest setpoint should not repeat without new data")) {
            debug_controller.shutdown();
            return 1;
        }
        debug_controller.shutdown();

        const std::vector<motor_base::ControlCommand> applied =
            debug_controller.applied_setpoints();
        if (!expect(applied.size() == 1 && applied[0].setpoints[0] == 8.0,
                    "debug channel should apply only the latest active-source command")) {
            return 1;
        }
    }

    {
        RecordingMotorController debug_only_controller(latest_channel_test_options());
        debug_only_controller.set_active_setpoint_source(motor_base::SetpointSource::DEBUG);
        if (!expect(debug_only_controller.start(),
                    "debug-only setpoint controller should start")) {
            return 1;
        }

        if (!expect(
                debug_only_controller.send_policy_setpoint(
                    motor_base::ControlCommand::set_position_targets_rad(
                        {3.0})).status ==
                    motor_base::CommandSubmitStatus::SOURCE_INACTIVE,
                "policy setpoint should be inactive when debug source is selected")) {
            debug_only_controller.shutdown();
            return 1;
        }
        if (!expect(
                debug_only_controller.send_debug_setpoint(
                    motor_base::ControlCommand::set_position_targets_rad(
                        {4.0})).status ==
                    motor_base::CommandSubmitStatus::ACCEPTED,
                "debug setpoint should publish when debug source is active")) {
            debug_only_controller.shutdown();
            return 1;
        }
        if (!expect(debug_only_controller.wait_for_applied_count(
                        1,
                        std::chrono::seconds(1)),
                    "only debug source should be consumed")) {
            debug_only_controller.shutdown();
            return 1;
        }
        debug_only_controller.shutdown();

        const std::vector<motor_base::ControlCommand> applied =
            debug_only_controller.applied_setpoints();
        if (!expect(applied.size() == 1 && applied[0].setpoints[0] == 4.0,
                    "selected debug source should be the only applied setpoint")) {
            return 1;
        }
    }

    {
        myactua::MyActMotorController::Options wrap_options = test_options();
        wrap_options.command_queue_capacity = 70;
        auto wrap_adapter = std::make_shared<FakeAdapter>(1);
        myactua::MyActMotorController wrap_controller(wrap_adapter, 1, wrap_options);

        const motor_base::CommandSubmitResult first_result =
            wrap_controller.send_discrete_command(motor_base::ControlCommand::stop());
        if (!expect(
                first_result.status == motor_base::CommandSubmitStatus::ACCEPTED &&
                    first_result.command_id.has_value(),
                "first tracked command should submit before result-table wrap")) {
            return 1;
        }

        for (int i = 0; i < 64; ++i) {
            const motor_base::CommandSubmitResult result =
                wrap_controller.send_discrete_command(motor_base::ControlCommand::stop());
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
        myactua::MyActMotorController::Options failing_options = test_options();
        const int max_fifo_priority = sched_get_priority_max(SCHED_FIFO);
        if (!expect(max_fifo_priority > 0,
                    "SCHED_FIFO max priority should be available")) {
            return 1;
        }
        failing_options.rt_priority = max_fifo_priority + 1;

        auto failing_adapter = std::make_shared<FakeAdapter>(1);
        myactua::MyActMotorController failing_controller(failing_adapter, 1, failing_options);
        failing_controller.set_active_setpoint_source(motor_base::SetpointSource::DEBUG);
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
                failing_controller.send_discrete_command(motor_base::ControlCommand::restart()).status ==
                    motor_base::CommandSubmitStatus::INVALID_COMMAND,
                "RESTART should be rejected when required RT scheduling is inactive")) {
            failing_controller.shutdown();
            return 1;
        }
        if (!expect(
                failing_controller.send_debug_setpoint(
                    motor_base::ControlCommand::set_position_targets_rad({0.0})).status ==
                    motor_base::CommandSubmitStatus::INVALID_COMMAND,
                "setpoint should be rejected when required RT scheduling is inactive")) {
            failing_controller.shutdown();
            return 1;
        }
        if (!expect(
                failing_controller.send_discrete_command(motor_base::ControlCommand::stop()).status ==
                    motor_base::CommandSubmitStatus::ACCEPTED,
                "STOP should remain accepted when RT scheduling is inactive")) {
            failing_controller.shutdown();
            return 1;
        }
        if (!expect(
                failing_controller.send_discrete_command(motor_base::ControlCommand::set_mode(
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

        myactua::MyActMotorController mode_controller(adapter, 1, test_options());
        mode_controller.set_active_setpoint_source(motor_base::SetpointSource::DEBUG);
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
                mode_controller.send_discrete_command(motor_base::ControlCommand::set_mode(
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
                mode_controller.send_debug_setpoint(
                    motor_base::ControlCommand::set_impedance_targets(setpoints)).status ==
                    motor_base::CommandSubmitStatus::ACCEPTED,
                "setpoint should publish for RT confirmed-mode validation")) {
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

        myactua::MyActMotorController stopped_controller(adapter, 1, test_options());
        stopped_controller.set_active_setpoint_source(motor_base::SetpointSource::DEBUG);
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
                stopped_controller.send_debug_setpoint(
                    motor_base::ControlCommand::set_position_targets_rad(
                        {static_cast<double>(target_raw) * myactua::kRawPosToRad})).status ==
                    motor_base::CommandSubmitStatus::ACCEPTED,
                "setpoint should publish when observed mode matches but motor is not running")) {
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

        myactua::MyActMotorController running_controller(adapter, 1, test_options());
        running_controller.set_active_setpoint_source(motor_base::SetpointSource::DEBUG);
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
            running_controller.send_discrete_command(motor_base::ControlCommand::restart());
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
                running_controller.send_debug_setpoint(
                    motor_base::ControlCommand::set_position_targets_rad(
                        {static_cast<double>(target_raw) * myactua::kRawPosToRad})).status ==
                    motor_base::CommandSubmitStatus::ACCEPTED,
                "setpoint should publish when all motors are confirmed running")) {
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

        myactua::MyActMotorController all_ready_controller(adapter, 2, test_options());
        if (!expect_start(all_ready_controller,
                          "all-ready restart controller should start")) {
            return 1;
        }
        const motor_base::CommandSubmitResult restart_submit =
            all_ready_controller.send_discrete_command(motor_base::ControlCommand::restart());
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

        myactua::MyActMotorController partial_controller(adapter, 2, test_options());
        partial_controller.set_active_setpoint_source(motor_base::SetpointSource::DEBUG);
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
            partial_controller.send_discrete_command(motor_base::ControlCommand::restart());
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
                partial_controller.send_debug_setpoint(
                    motor_base::ControlCommand::set_position_targets_rad(
                        {static_cast<double>(target0_raw) * myactua::kRawPosToRad,
                         static_cast<double>(target1_raw) * myactua::kRawPosToRad})).status ==
                    motor_base::CommandSubmitStatus::ACCEPTED,
                "partial-frame setpoint should publish for RT validation")) {
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

          myactua::MyActMotorController watchdog_controller(adapter, 1, test_options());
          watchdog_controller.set_active_setpoint_source(motor_base::SetpointSource::DEBUG);
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

          myactua::MyActMotorController watchdog_controller(adapter, 1, test_options());
          watchdog_controller.set_active_setpoint_source(motor_base::SetpointSource::DEBUG);
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

          myactua::MyActMotorController watchdog_controller(adapter, 1, test_options());
          watchdog_controller.set_active_setpoint_source(motor_base::SetpointSource::DEBUG);
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
                  watchdog_controller.send_debug_setpoint(
                      motor_base::ControlCommand::set_position_targets_rad({0.0})).status ==
                      motor_base::CommandSubmitStatus::INVALID_COMMAND,
                  "setpoint commands should be rejected while communication fault is latched")) {
              watchdog_controller.shutdown();
              return 1;
          }
          if (!expect(
                  watchdog_controller.send_discrete_command(motor_base::ControlCommand::set_mode(
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

          myactua::MyActMotorController watchdog_controller(adapter, 1, test_options());
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

          myactua::MyActMotorController watchdog_controller(adapter, 1, test_options());
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
              watchdog_controller.send_discrete_command(motor_base::ControlCommand::restart(0));
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

          myactua::MyActMotorController watchdog_controller(adapter, 1, test_options());
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
          watchdog_controller.send_discrete_command(motor_base::ControlCommand::restart());
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

          myactua::MyActMotorController watchdog_controller(adapter, 1, test_options());
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

      {
          auto setup_options = test_options();
          setup_options.setpoint_timeout_ns = 10'000'000;

          auto adapter = std::make_shared<FakeAdapter>(1);
          adapter->set_rx_status_word(0, operation_enabled_status_word());
          adapter->set_rx_mode(0, myactua::MyactControlMode::PVT);

          myactua::MyActMotorController setup_controller(
              adapter, 1, setup_options);
          std::atomic<int> timeout_events{0};
          std::atomic<int> timeout_reason{0};
          std::atomic<std::uint32_t> timeout_policy_seq{1};
          setup_controller.set_event_callback(
              [&timeout_events, &timeout_reason, &timeout_policy_seq](
                  const motor_base::RtEvent& event) {
                  if (event.type ==
                      motor_base::RtEventType::SETPOINT_TIMEOUT_FAULT) {
                      timeout_events.fetch_add(1, std::memory_order_relaxed);
                      timeout_reason.store(event.reason, std::memory_order_relaxed);
                      timeout_policy_seq.store(event.value,
                                               std::memory_order_relaxed);
                  }
              });

          if (!expect_start(setup_controller,
                            "startup hold controller should start")) {
              return 1;
          }
          const motor_base::CommandSubmitResult mode_submit =
              setup_controller.send_discrete_command(
                  motor_base::ControlCommand::set_mode(
                      motor_base::MotorControlMode::IMPEDANCE));
          const motor_base::CommandSubmitResult restart_submit =
              setup_controller.send_discrete_command(
                  motor_base::ControlCommand::restart());
          if (!expect(
                  mode_submit.status ==
                          motor_base::CommandSubmitStatus::ACCEPTED &&
                      restart_submit.status ==
                          motor_base::CommandSubmitStatus::ACCEPTED,
                  "startup hold controller should accept mode and restart")) {
              setup_controller.shutdown();
              return 1;
          }
          if (!expect(adapter->wait_for_cycles(60, std::chrono::seconds(1)),
                      "startup hold controller should reach running state")) {
              setup_controller.shutdown();
              return 1;
          }

          constexpr int32_t hold_target_raw = 2468;
          constexpr int32_t hold_kp_raw = 9000;
          constexpr int32_t hold_kd_raw = 700;
          const std::vector<motor_base::ImpedanceSetpoint> hold_setpoints = {
              motor_base::ImpedanceSetpoint(
                  static_cast<double>(hold_target_raw) *
                      myactua::kRawPosToRad,
                  0.0,
                  0.0,
                  static_cast<double>(hold_kp_raw) / 1000.0,
                  static_cast<double>(hold_kd_raw) / 1000.0)};

          for (int i = 0; i < 12; ++i) {
              motor_base::ControlCommand hold_command =
                  motor_base::ControlCommand::set_impedance_targets(
                      hold_setpoints);
              const std::int64_t produced_at_ns =
                  robot_base::monotonic_now_ns();
              hold_command.timing.source_policy_seq = 0;
              hold_command.timing.produced_at_ns = produced_at_ns;
              hold_command.timing.valid_until_ns =
                  produced_at_ns + 10'000'000;
              if (!expect(
                      setup_controller.send_policy_setpoint(hold_command).status ==
                          motor_base::CommandSubmitStatus::ACCEPTED,
                      "fresh startup hold command should be accepted")) {
                  setup_controller.shutdown();
                  return 1;
              }
              if (!expect(
                      adapter->wait_for_cycles(adapter->cycles() + 2,
                                               std::chrono::seconds(1)),
                      "startup hold command should be refreshed before expiry")) {
                  setup_controller.shutdown();
                  return 1;
              }
          }

          const myactua::TxPDO hold_tx = adapter->last_tx(0);
          if (!expect(!setup_controller.safety_stop_latched() &&
                          timeout_events.load(std::memory_order_relaxed) == 0,
                      "refreshed startup hold must not latch STOP")) {
              setup_controller.shutdown();
              return 1;
          }
          if (!expect(hold_tx.target_pos == hold_target_raw &&
                          hold_tx.target_torque == 0 &&
                          hold_tx.pvt_kp == hold_kp_raw &&
                          hold_tx.pvt_kd == hold_kd_raw,
                      "startup hold refresh must preserve the reset command")) {
              setup_controller.shutdown();
              return 1;
          }

          if (!expect(adapter->wait_for_cycles(adapter->cycles() + 15,
                                               std::chrono::seconds(1)),
                      "stopped startup hold producer should reach timeout")) {
              setup_controller.shutdown();
              return 1;
          }
          setup_controller.shutdown();

          if (!expect(setup_controller.safety_stop_latched() &&
                          timeout_events.load(std::memory_order_relaxed) == 1 &&
                          timeout_reason.load(std::memory_order_relaxed) == 1 &&
                          timeout_policy_seq.load(std::memory_order_relaxed) == 0,
                      "expired startup hold should latch reason=1, policy_seq=0")) {
              return 1;
          }
          if (!expect(adapter->last_tx(0).control_word ==
                          myactua::CMD_DISABLE_OPERATION,
                      "expired startup hold should apply STOP output")) {
              return 1;
          }
      }

      {
          auto timing_options = latest_channel_test_options();
          timing_options.rt_period_ns = 1'000'000;
          timing_options.setpoint_timeout_ns = 5'000'000;
          RecordingMotorController timing_controller(timing_options);
          std::atomic<int> timeout_events{0};
          timing_controller.set_event_callback(
              [&timeout_events](const motor_base::RtEvent& event) {
                  if (event.type == motor_base::RtEventType::SETPOINT_TIMEOUT_FAULT) {
                      timeout_events.fetch_add(1, std::memory_order_relaxed);
                  }
              });
          if (!expect(timing_controller.start(),
                      "freshness controller should start")) {
              return 1;
          }

          motor_base::ControlCommand command = timed_policy_position(2.5, 6);
          const std::int64_t now_ns = robot_base::monotonic_now_ns();
          command.timing.produced_at_ns = now_ns + 20'000'000;
          command.timing.valid_until_ns = now_ns + 100'000'000;
          if (!expect(timing_controller.send_policy_setpoint(command).status ==
                          motor_base::CommandSubmitStatus::ACCEPTED,
                      "future freshness command should be accepted at submission")) {
              timing_controller.shutdown();
              return 1;
          }
          if (!expect(timing_controller.wait_for_cycles(3,
                                                         std::chrono::seconds(1)),
                      "freshness controller should continue its RT cycles")) {
              timing_controller.shutdown();
              return 1;
          }

          if (!expect(!timing_controller.safety_stop_latched() &&
                          timing_controller.safety_stop_count() == 0,
                      "invalid incoming metadata should be rejected without latching")) {
              timing_controller.shutdown();
              return 1;
          }

          command = timed_policy_position(2.5, 7);
          const std::int64_t valid_now_ns = robot_base::monotonic_now_ns();
          command.timing.produced_at_ns = valid_now_ns;
          command.timing.valid_until_ns = valid_now_ns + 20'000'000;
          if (!expect(timing_controller.send_policy_setpoint(command).status ==
                          motor_base::CommandSubmitStatus::ACCEPTED,
                      "freshness command should be accepted")) {
              timing_controller.shutdown();
              return 1;
          }
          if (!expect(timing_controller.wait_for_applied_count(
                          1, std::chrono::seconds(1)),
                      "valid command should be applied after metadata rejection")) {
              timing_controller.shutdown();
              return 1;
          }
          if (!expect(timing_controller.send_policy_setpoint(
                          timed_policy_position(2.6, 0)).status ==
                          motor_base::CommandSubmitStatus::INVALID_COMMAND,
                      "policy sequence zero should be rejected after startup")) {
              timing_controller.shutdown();
              return 1;
          }

          motor_base::ControlCommand rollback = timed_policy_position(2.7, 8);
          rollback.timing.produced_at_ns = valid_now_ns - 1'000'000;
          rollback.timing.valid_until_ns = valid_now_ns + 20'000'000;
          if (!expect(timing_controller.send_policy_setpoint(rollback).status ==
                          motor_base::CommandSubmitStatus::ACCEPTED,
                      "timestamp rollback should reach the RT validator")) {
              timing_controller.shutdown();
              return 1;
          }
          const std::uint64_t rollback_target_cycle =
              timing_controller.cycles() + 3;
          if (!expect(timing_controller.wait_for_cycles(rollback_target_cycle,
                                                         std::chrono::seconds(1)),
                      "timestamp rollback should be processed")) {
              timing_controller.shutdown();
              return 1;
          }
          if (!expect(!timing_controller.safety_stop_latched(),
                      "timestamp rollback should not latch the safety stop")) {
              timing_controller.shutdown();
              return 1;
          }

          if (!expect(timing_controller.wait_for_cycles(60,
                                                         std::chrono::seconds(1)),
                      "freshness controller should continue its RT cycles")) {
              timing_controller.shutdown();
              return 1;
          }
          timing_controller.shutdown();

          if (!expect(timing_controller.safety_stop_latched(),
                      "expired command should latch the safety stop")) {
              return 1;
          }
          if (!expect(timing_controller.safety_stop_count() > 0 &&
                          timeout_events.load(std::memory_order_relaxed) >= 3,
                      "expired command should directly apply STOP and emit freshness events")) {
              return 1;
          }
          if (!expect(timing_controller.send_policy_setpoint(
                          timed_policy_position(3.0, 8)).status ==
                          motor_base::CommandSubmitStatus::INVALID_COMMAND,
                      "fresh producer must not clear a latched safety stop")) {
              return 1;
          }
          if (!expect(timing_controller.clear_safety_stop_latch(),
                      "safety latch should require explicit stopped-state clear")) {
              return 1;
          }
          if (!expect(!timing_controller.safety_stop_latched(),
                      "explicit clear should remove the safety latch")) {
              return 1;
          }
      }

      return 0;
  }
