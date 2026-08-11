#pragma once
#include <vector>
#include <memory>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include "motor_base/motor_base.hpp"
#include "ethercat_types.hpp"
#include "ethercat_adapter.hpp"
#include "motor_base/command_types.hpp"
#include "motor_base/motor_status_monitor.hpp"
#include "motor_base/rt_event_dispatcher.hpp"
#include "motor_base/status_channel.hpp"
#include "driver/myact/motor_state.hpp"
#include "driver/myact/myact_types.hpp"

#define LIMIT(VAL, MIN, MAX) ((VAL)<(MIN)?(MIN):((VAL)>(MAX)?(MAX):(VAL)))

namespace myactua{


/// @brief MYACTUA 品牌电机 EtherCAT 控制器（IGH 协议栈）。
/// 继承 MotorControllerBase，实现 CiA 402 状态机与 PDO 收发。
class MYACTUA : public motor_base::MotorControllerBase {
public:
    using MyactDiagnosticsCallback = std::function<void(const std::vector<MyactDiagnosticsSnapshot>&)>;

    struct Options : motor_base::MotorControllerBase::RealtimeOptions {};

    MYACTUA(std::shared_ptr<EthercatAdapter> adapter, int num_motors);
    MYACTUA(std::shared_ptr<EthercatAdapter> adapter, int num_motors, Options options);

    ~MYACTUA();

    bool wait_all_motors_ready(int timeout_ms = 30000,
                               int poll_ms = 100) const override;

    std::vector<MyactDiagnosticsSnapshot> get_myact_diagnostics();

    void set_myact_diagnostics_callback(MyactDiagnosticsCallback cb);

    void set_print_info(const std::vector<int>& motor_index) override;

    /// @name MYACTUA 特有静态工具方法（电机编码器分辨率相关）
    /// @{
    static double raw_pos_to_rad(double raw_pos);
    static double raw_vel_to_rad_s(double raw_vel);
    /// @}

    
private:
    Options options_;
    std::shared_ptr<EthercatAdapter> _adapter;
    std::vector<MotorState> _motors;

    std::atomic<uint64_t> status_channel_busy_count_{0};
    motor_base::LatestStatusChannel<MyactDiagnosticsSnapshot> diagnostics_channel_;
    motor_base::MotorStatusMonitor<MyactDiagnosticsSnapshot> status_monitor_;

    bool connect_impl(const char* ifname) override;

    motor_base::CommandSubmitResult validate_command(
        const motor_base::ControlCommand& cmd) const override;

    void apply_setpoint_command_callback(
        const motor_base::ControlCommand& cmd) override;
    void apply_discrete_command_callback(
        int motor_index,
        const motor_base::DiscreteCommand& cmd) override;
    motor_base::DiscreteCommandEvaluation evaluate_discrete_command_callback(
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
    void update_status_snapshot_rt();
    void update_diagnostics_snapshot_rt();

    void push_status_channel_busy_event_rt();

    static void rt_event_sink_trampoline(
        void* context,
        const motor_base::RtEvent& event);

    void refresh_observed_state(MotorState& motor);

    void process_single_motor(MotorState& motor);

    void handle_mode_switching(MotorState& motor);

    ControlWordCommand get_next_control_word(uint16_t status_word);
    static MyactControlMode to_myact_mode(motor_base::MotorControlMode mode);
    static motor_base::MotorControlMode to_motor_control_mode(MyactControlMode mode);
};

} // namespace myactua
