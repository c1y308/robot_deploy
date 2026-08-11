#include "motor_base/command_types.hpp"
#include "ethercat_adapter.hpp"
#include "driver/myact/motor_control.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace {

class FakeAdapter : public myactua::EthercatAdapter {
public:
    explicit FakeAdapter(int motor_count)
        : motor_count_(motor_count)
    {
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
        cycles_.fetch_add(1, std::memory_order_relaxed);
    }

    void send_physical() override {}

    void send(int index, const myactua::TxPDO& pdo) override
    {
        if (index >= 0 && index < motor_count_) {
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

    std::uint64_t cycles() const
    {
        return cycles_.load(std::memory_order_relaxed);
    }

    std::size_t receiver_thread_count() const
    {
        std::lock_guard<std::mutex> lock(thread_ids_mutex_);
        return receiver_thread_ids_.size();
    }

private:
    int motor_count_;
    std::array<myactua::RxPDO, motor_base::kMaxMotorCommandSetpoints> rx_{};
    std::array<myactua::TxPDO, motor_base::kMaxMotorCommandSetpoints> tx_{};
    std::atomic<std::uint64_t> cycles_{0};
    mutable std::mutex thread_ids_mutex_;
    std::vector<std::thread::id> receiver_thread_ids_;
};

bool expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "[motor_realtime_channel_test] " << message << "\n";
        return false;
    }
    return true;
}

} // namespace

int main()
{
    myactua::MYACTUA::Options options;
    options.command_queue_capacity = 2;
    options.discrete_queue_capacity_per_motor = 1;
    options.rt_event_queue_capacity = 32;
    options.max_commands_per_cycle = 8;
    options.status_publish_period_ms = 1;

    auto adapter = std::make_shared<FakeAdapter>(1);
    myactua::MYACTUA controller(adapter, 1, options);

    std::vector<double> too_many(motor_base::kMaxMotorCommandSetpoints + 1, 0.0);
    if (!expect(
            controller.send_command(motor_base::ControlCommand::set_position_targets_rad(too_many)) ==
                motor_base::CommandSubmitResult::INVALID_PAYLOAD,
            "oversized setpoint payload should be rejected")) {
        return 1;
    }

    if (!expect(
            controller.send_command(motor_base::ControlCommand::set_velocity_targets_rad_s({0.0})) ==
                motor_base::CommandSubmitResult::INVALID_COMMAND,
            "setpoint type should be rejected when it does not match target mode")) {
        return 1;
    }

    if (!expect(
            controller.send_command(motor_base::ControlCommand::stop()) ==
                motor_base::CommandSubmitResult::ACCEPTED,
            "first command should be accepted")) {
        return 1;
    }
    if (!expect(
            controller.send_command(motor_base::ControlCommand::restart()) ==
                motor_base::CommandSubmitResult::ACCEPTED,
            "second command should be accepted")) {
        return 1;
    }
    if (!expect(
            controller.send_command(motor_base::ControlCommand::set_mode(motor_base::MotorControlMode::POSITION)) ==
                motor_base::CommandSubmitResult::QUEUE_FULL,
            "bounded command queue should report full")) {
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
        [&diagnostics_callbacks](const std::vector<myactua::MyactDiagnosticsSnapshot>&) {
            diagnostics_callbacks.fetch_add(1, std::memory_order_relaxed);
        });
    controller.set_rt_event_callback(
        [&discrete_queue_full_events,
         &status_channel_busy_events](const motor_base::RtEvent& event) {
            if (event.type == motor_base::RtEventType::DISCRETE_QUEUE_FULL) {
                discrete_queue_full_events.fetch_add(1, std::memory_order_relaxed);
            }
            if (event.type == motor_base::RtEventType::STATUS_CHANNEL_BUSY) {
                status_channel_busy_events.fetch_add(1, std::memory_order_relaxed);
            }
        });

    controller.start();
    controller.start();
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
    if (!expect(discrete_queue_full_events.load(std::memory_order_relaxed) > 0,
                "discrete queue overflow should emit an RT event")) {
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
                [](const std::vector<myactua::MyactDiagnosticsSnapshot>&) {});
        }
    });
    callback_toggler.join();

    controller.shutdown();
    controller.shutdown();
    return 0;
}
