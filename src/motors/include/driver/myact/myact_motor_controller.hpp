#pragma once
#include <vector>
#include <memory>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include "motor_base/motor_controller_base.hpp"
#include "protocol/ethercat/ethercat_types.hpp"
#include "protocol/ethercat/ethercat_adapter.hpp"
#include "motor_base/command_types.hpp"
#include "motor_base/motor_status_monitor.hpp"
#include "motor_base/rt_event_dispatcher.hpp"
#include "motor_base/status_channel.hpp"
#include "driver/myact/motor_state.hpp"
#include "driver/myact/myact_types.hpp"

namespace myactua{


/// @brief MYACTUA 品牌电机 EtherCAT 控制器（IGH 协议栈）。
/// 继承 MotorControllerBase，实现 CiA 402 状态机与 PDO 收发。
class MyActMotorController : public motor_base::MotorControllerBase {
public:
    using MyactDiagnosticsCallback = std::function<void(const std::vector<MotorState>&)>;

    struct Options : motor_base::MotorControllerBase::RealtimeOptions {
        uint32_t comm_watchdog_fault_cycles = 10;
        ControlWordCommand comm_fault_control_word = CMD_QUICK_STOP;
    };

    MyActMotorController(std::shared_ptr<EthercatAdapter> adapter, int num_motors);
    MyActMotorController(std::shared_ptr<EthercatAdapter> adapter, int num_motors, Options options);

    ~MyActMotorController();

    bool wait_all_motors_ready(int timeout_ms = 30000, int poll_ms = 100) const override;

    std::vector<MotorState> get_myact_diagnostics();

    void set_myact_diagnostics_callback(MyactDiagnosticsCallback cb);

    void set_print_info(const std::vector<int>& motor_index) override;

private:
    Options options_;
    std::shared_ptr<EthercatAdapter> _adapter;
    std::vector<MotorState> _motors;

    std::atomic<uint64_t> status_channel_busy_count_{0};
    motor_base::LatestStatusChannel<MotorState> diagnostics_channel_;
    motor_base::MotorStatusMonitor<MotorState>  status_monitor_;

    std::atomic<bool> whole_body_fault_latched_{false};
    uint32_t process_data_fail_count_{0};
    MyactCommunicationFaultReason fault_reason_{MyactCommunicationFaultReason::None};
    std::int64_t current_cycle_host_timestamp_ns_{0};
    bool process_data_ok_{false}; // Most recent received cycle, RT thread only.

    bool connect_impl(const char* ifname) override;

    motor_base::CommandSubmitStatus validate_command(
        const motor_base::ControlCommand& cmd) const override;

    void apply_setpoint_command_impl(
        const motor_base::ControlCommand& cmd) override;
    void apply_discrete_command_impl(
        int motor_index,
        const motor_base::DiscreteCommand& cmd) override;
    motor_base::DiscreteCommandEvaluation evaluate_discrete_command_impl(
        int motor_index,
        const motor_base::DiscreteCommand& cmd) const override;

    void discrete_command_failed_callback(
        int motor_index,
        const motor_base::DiscreteCommand& cmd,
        motor_base::DiscreteFailReason reason) override;
    void discrete_queue_full_callback(
        int motor_index,
        const motor_base::ControlCommand& cmd) override;

    bool realtime_start_callback() override;
    void realtime_cycle_callback() override;
    void realtime_stop_callback() noexcept override;

    void update();
    void update_realtime_feedback();
    void update_status_snapshot();
    void update_diagnostics_snapshot();
    void update_communication_watchdog(
        bool process_data_ok,
        const EthercatBusHealthSnapshot& health);
    void latch_communication_fault(
        MyactCommunicationFaultReason reason,
        const EthercatBusHealthSnapshot& health);
    void apply_whole_body_quick_stop();
    void reset_motor_setpoints_to_feedback(MotorState& motor);

    void push_status_channel_busy_event();

    static void event_sink_trampoline(
        void* context,
        const motor_base::RtEvent& event);

    void process_single_motor(MotorState& motor);

    void handle_mode_switching(MotorState& motor);

    static MyactControlMode to_myact_mode(motor_base::MotorControlMode mode);
    static motor_base::MotorControlMode to_motor_control_mode(MyactControlMode mode);
};

} // namespace myactua
