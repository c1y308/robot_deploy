#include "ControlTypes.hpp"
#include "EthercatAdapter.hpp"
#include "motor_control.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
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
            rx.op_mode = myactua::ControlMode::CSP;
        }
    }

    bool init(const char*) override { return true; }

    void receive_physical() override
    {
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

private:
    int motor_count_;
    std::array<myactua::RxPDO, myactua::kMaxMotorCommandSetpoints> rx_{};
    std::array<myactua::TxPDO, myactua::kMaxMotorCommandSetpoints> tx_{};
    std::atomic<std::uint64_t> cycles_{0};
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

    std::vector<double> too_many(myactua::kMaxMotorCommandSetpoints + 1, 0.0);
    if (!expect(
            controller.send_command(myactua::ControlCommand::set_scalar_setpoints(too_many)) ==
                myactua::CommandSubmitResult::INVALID_PAYLOAD,
            "oversized setpoint payload should be rejected")) {
        return 1;
    }

    if (!expect(
            controller.send_command(myactua::ControlCommand::stop()) ==
                myactua::CommandSubmitResult::ACCEPTED,
            "first command should be accepted")) {
        return 1;
    }
    if (!expect(
            controller.send_command(myactua::ControlCommand::restart()) ==
                myactua::CommandSubmitResult::ACCEPTED,
            "second command should be accepted")) {
        return 1;
    }
    if (!expect(
            controller.send_command(myactua::ControlCommand::set_mode(myactua::ControlMode::CSP)) ==
                myactua::CommandSubmitResult::QUEUE_FULL,
            "bounded command queue should report full")) {
        return 1;
    }

    std::atomic<int> status_callbacks{0};
    std::atomic<int> discrete_queue_full_events{0};
    controller.set_status_callback(
        [&status_callbacks](const std::vector<myactua::MotorStatusSnapshot>&) {
            status_callbacks.fetch_add(1, std::memory_order_relaxed);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        });
    controller.set_rt_event_callback(
        [&discrete_queue_full_events](const myactua::RtEvent& event) {
            if (event.type == myactua::RtEventType::DISCRETE_QUEUE_FULL) {
                discrete_queue_full_events.fetch_add(1, std::memory_order_relaxed);
            }
        });

    controller.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(80));

    if (!expect(adapter->cycles() > 30,
                "slow status callback should not block the realtime loop")) {
        controller.shutdown();
        return 1;
    }
    if (!expect(status_callbacks.load(std::memory_order_relaxed) > 0,
                "status callback should be called by publisher thread")) {
        controller.shutdown();
        return 1;
    }
    if (!expect(discrete_queue_full_events.load(std::memory_order_relaxed) > 0,
                "discrete queue overflow should emit an RT event")) {
        controller.shutdown();
        return 1;
    }

    std::thread callback_toggler([&controller]() {
        for (int i = 0; i < 100; ++i) {
            controller.set_status_callback({});
            controller.set_status_callback(
                [](const std::vector<myactua::MotorStatusSnapshot>&) {});
        }
    });
    callback_toggler.join();

    controller.shutdown();
    return 0;
}
