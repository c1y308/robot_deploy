#include "motor_base/command_types.hpp"
#include "ethercat_adapter.hpp"
#include "driver/myact/motor_control.hpp"

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
    options.command_queue_capacity = 2;
    options.discrete_queue_capacity_per_motor = 1;
    options.rt_event_queue_capacity = 32;
    options.max_commands_per_cycle = 8;
    options.status_publish_period_ms = 1;
    options.rt_priority = 0;

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
                    motor_base::ControlCommand::restart()) ==
                    motor_base::CommandSubmitResult::INVALID_COMMAND,
                "RESTART should be rejected when required RT scheduling is inactive")) {
            failing_controller.shutdown();
            return 1;
        }
        if (!expect(
                failing_controller.send_command(
                    motor_base::ControlCommand::set_position_targets_rad({0.0})) ==
                    motor_base::CommandSubmitResult::INVALID_COMMAND,
                "setpoint should be rejected when required RT scheduling is inactive")) {
            failing_controller.shutdown();
            return 1;
        }
        if (!expect(
                failing_controller.send_command(motor_base::ControlCommand::stop()) ==
                    motor_base::CommandSubmitResult::ACCEPTED,
                "STOP should remain accepted when RT scheduling is inactive")) {
            failing_controller.shutdown();
            return 1;
        }
        if (!expect(
                failing_controller.send_command(
                    motor_base::ControlCommand::set_mode(
                        motor_base::MotorControlMode::POSITION)) ==
                    motor_base::CommandSubmitResult::ACCEPTED,
                "SET_MODE should remain accepted when RT scheduling is inactive")) {
            failing_controller.shutdown();
            return 1;
        }
        failing_controller.shutdown();
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
          std::atomic<int> clear_events{0};
          std::atomic<int> last_reason{0};
          watchdog_controller.set_event_callback(
              [&fault_events, &clear_events, &last_reason](
                  const motor_base::RtEvent& event) {
                  if (event.type == motor_base::RtEventType::COMM_WATCHDOG_FAULT) {
                      fault_events.fetch_add(1, std::memory_order_relaxed);
                      last_reason.store(event.reason, std::memory_order_relaxed);
                  }
                  if (event.type == motor_base::RtEventType::COMM_WATCHDOG_CLEARED) {
                      clear_events.fetch_add(1, std::memory_order_relaxed);
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
                      motor_base::ControlCommand::set_position_targets_rad({0.0})) ==
                      motor_base::CommandSubmitResult::INVALID_COMMAND,
                  "setpoint commands should be rejected while communication fault is latched")) {
              watchdog_controller.shutdown();
              return 1;
          }
          if (!expect(
                  watchdog_controller.send_command(
                      motor_base::ControlCommand::set_mode(
                          motor_base::MotorControlMode::POSITION)) ==
                      motor_base::CommandSubmitResult::INVALID_COMMAND,
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
          if (!expect(clear_events.load(std::memory_order_relaxed) == 0,
                      "communication recovery without RESTART(-1) should not clear latch")) {
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
          std::atomic<int> clear_events{0};
          watchdog_controller.set_event_callback(
              [&clear_events](const motor_base::RtEvent& event) {
                  if (event.type == motor_base::RtEventType::COMM_WATCHDOG_CLEARED) {
                      clear_events.fetch_add(1, std::memory_order_relaxed);
                  }
              });
          if (!expect_start(watchdog_controller,
                            "single-motor restart controller should start")) {
              return 1;
          }
          if (!expect(adapter->wait_for_cycles(12, std::chrono::seconds(1)),
                      "single-motor restart scenario should reach fault")) {
              watchdog_controller.shutdown();
              return 1;
          }
          watchdog_controller.send_command(motor_base::ControlCommand::restart(0));
          if (!expect(adapter->wait_for_cycles(32, std::chrono::seconds(1)),
                      "single-motor restart scenario should continue running")) {
              watchdog_controller.shutdown();
              return 1;
          }
          watchdog_controller.shutdown();
          if (!expect(clear_events.load(std::memory_order_relaxed) == 0,
                      "RESTART(i) should not clear a whole-body communication fault")) {
              return 1;
          }
      }

      {
          auto adapter = std::make_shared<FakeAdapter>(1);
          std::vector<myactua::EthercatBusHealthSnapshot> script;
          append_health(script, 10, health(true, EC_WC_INCOMPLETE, 0));
          adapter->set_health_script(script);

          myactua::MYACTUA watchdog_controller(adapter, 1, test_options());
          std::atomic<int> clear_events{0};
          watchdog_controller.set_event_callback(
              [&clear_events](const motor_base::RtEvent& event) {
                  if (event.type == motor_base::RtEventType::COMM_WATCHDOG_CLEARED) {
                      clear_events.fetch_add(1, std::memory_order_relaxed);
                  }
              });
          if (!expect_start(watchdog_controller,
                            "RESTART(-1) recovery controller should start")) {
              return 1;
          }
          if (!expect(adapter->wait_for_cycles(12, std::chrono::seconds(1)),
                      "RESTART(-1) recovery scenario should reach fault")) {
              watchdog_controller.shutdown();
              return 1;
          }

          std::vector<myactua::EthercatBusHealthSnapshot> recovery_script;
          append_health(recovery_script, 9, health(true, EC_WC_COMPLETE, 1));
          append_health(recovery_script, 1, health(true, EC_WC_INCOMPLETE, 0));
          adapter->set_health_script(recovery_script);
          watchdog_controller.send_command(motor_base::ControlCommand::restart());
          if (!expect(adapter->wait_for_cycles(28, std::chrono::seconds(1)),
                      "9-good recovery scenario should continue running")) {
              watchdog_controller.shutdown();
              return 1;
          }
          if (!expect(clear_events.load(std::memory_order_relaxed) == 0,
                      "RESTART(-1) plus only 9 healthy cycles should not clear")) {
              watchdog_controller.shutdown();
              return 1;
          }

          if (!expect(adapter->wait_for_cycles(48, std::chrono::seconds(1)),
                      "10-good recovery scenario should clear latch")) {
              watchdog_controller.shutdown();
              return 1;
          }
          watchdog_controller.shutdown();
          if (!expect(clear_events.load(std::memory_order_relaxed) == 1,
                      "RESTART(-1) plus 10 healthy cycles should clear once")) {
              return 1;
          }
      }

      {
          auto adapter = std::make_shared<FakeAdapter>(1);
          std::vector<myactua::EthercatBusHealthSnapshot> script;
          append_health(script, 10, health(true, EC_WC_INCOMPLETE, 0));
          adapter->set_health_script(script);

          myactua::MYACTUA watchdog_controller(adapter, 1, test_options());
          std::atomic<int> clear_events{0};
          watchdog_controller.set_event_callback(
              [&clear_events](const motor_base::RtEvent& event) {
                  if (event.type == motor_base::RtEventType::COMM_WATCHDOG_CLEARED) {
                      clear_events.fetch_add(1, std::memory_order_relaxed);
                  }
              });
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
          if (!expect(clear_events.load(std::memory_order_relaxed) == 0 &&
                          tx.control_word == myactua::CMD_QUICK_STOP,
                      "RT thread restart should not clear communication latch")) {
              return 1;
          }
      }

      {
          auto adapter = std::make_shared<FakeAdapter>(1);
          constexpr int32_t current_raw = 1234;
          constexpr int32_t stale_raw = 54321;
          adapter->set_rx_position(0, current_raw);
          adapter->set_rx_status_word(
              0,
              myactua::BIT_READY_TO_SWITCH_ON |
                  myactua::BIT_SWITCHED_ON |
                  myactua::BIT_OPERATION_ENABLED);

          myactua::MYACTUA watchdog_controller(adapter, 1, test_options());
          std::atomic<int> clear_events{0};
          watchdog_controller.set_event_callback(
              [&clear_events](const motor_base::RtEvent& event) {
                  if (event.type == motor_base::RtEventType::COMM_WATCHDOG_CLEARED) {
                      clear_events.fetch_add(1, std::memory_order_relaxed);
                  }
              });
          if (!expect_start(watchdog_controller,
                            "stale setpoint controller should start")) {
              return 1;
          }
          watchdog_controller.send_command(motor_base::ControlCommand::restart());
          watchdog_controller.send_command(
              motor_base::ControlCommand::set_position_targets_rad(
                  {myactua::MYACTUA::raw_pos_to_rad(stale_raw)}));
          if (!expect(adapter->wait_for_cycles(6, std::chrono::seconds(1)),
                      "stale setpoint scenario should apply initial setpoint")) {
              watchdog_controller.shutdown();
              return 1;
          }
          if (!expect(adapter->last_tx(0).target_pos == stale_raw,
                      "test setup should apply stale position target before fault")) {
              watchdog_controller.shutdown();
              return 1;
          }

          std::vector<myactua::EthercatBusHealthSnapshot> fault_script;
          append_health(fault_script, 10, health(true, EC_WC_INCOMPLETE, 0));
          adapter->set_health_script(fault_script);
          if (!expect(adapter->wait_for_cycles(20, std::chrono::seconds(1)),
                      "stale setpoint scenario should reach fault")) {
              watchdog_controller.shutdown();
              return 1;
          }
          watchdog_controller.send_command(motor_base::ControlCommand::restart());
          if (!expect(adapter->wait_for_cycles(45, std::chrono::seconds(1)),
                      "stale setpoint scenario should clear fault")) {
              watchdog_controller.shutdown();
              return 1;
          }
          watchdog_controller.shutdown();

          const myactua::TxPDO tx = adapter->last_tx(0);
          if (!expect(clear_events.load(std::memory_order_relaxed) == 1,
                      "stale setpoint scenario should clear once")) {
              return 1;
          }
          if (!expect(tx.target_pos == current_raw && tx.target_pos != stale_raw,
                      "clearing communication fault should reset target to feedback")) {
              return 1;
          }
      }
      return 0;
  }
