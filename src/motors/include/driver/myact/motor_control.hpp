#pragma once
#include <vector>
#include <memory>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include "motor_base/MotorControllerBase.hpp"
#include "EthercatTypes.hpp"
#include "EthercatAdapter.hpp"
#include "motor_base/ControlTypes.hpp"
#include "motor_base/MotorStatusMonitor.hpp"
#include "motor_base/RtEventDispatcher.hpp"
#include "driver/myact/motor_state.hpp"
#include "driver/myact/motor_status_channel.hpp"
#include "driver/myact/myact_types.hpp"

#define LIMIT(VAL, MIN, MAX) ((VAL)<(MIN)?(MIN):((VAL)>(MAX)?(MAX):(VAL)))

namespace myactua{


/// @brief MYACTUA 品牌电机 EtherCAT 控制器（IGH 协议栈）。
/// 继承 MotorControllerBase，实现 CiA 402 状态机与 PDO 收发。
class MYACTUA : public motor_base::MotorControllerBase {
public:
    using MyactDiagnosticsCallback =
        std::function<void(const std::vector<MyactDiagnosticsSnapshot>&)>;

    struct Options : motor_base::MotorControllerBase::RealtimeOptions {
        std::size_t rt_event_queue_capacity = 256;
        int status_publish_period_ms = 1;
    };

    MYACTUA(std::shared_ptr<EthercatAdapter> adapter, int num_motors);
    MYACTUA(std::shared_ptr<EthercatAdapter> adapter, int num_motors, Options options);

    ~MYACTUA();

    bool connect(const char* ifname) override;
    bool wait_all_motors_ready(int timeout_ms = 30000, int poll_ms = 100, const std::function<bool()>& should_stop = {}) const override;

    std::vector<motor_base::MotorStatusSnapshot> get_status() override;
    std::vector<double> get_joint_q_rad() override;
    std::vector<double> get_joint_vel_rad_s() override;
    std::vector<double> get_joint_torque_percent() override;
    std::vector<MyactDiagnosticsSnapshot> get_myact_diagnostics();

    void set_status_callback(
        motor_base::MotorControllerBase::StatusCallback cb) override;
    void set_myact_diagnostics_callback(MyactDiagnosticsCallback cb);
    void set_rt_event_callback(
        motor_base::MotorControllerBase::RtEventCallback cb) override;

    void set_print_info(const std::vector<int>& motor_indices) override;

    /// @name MYACTUA 特有静态工具方法（电机编码器分辨率相关）
    /// @{
    static double raw_pos_to_rad(double raw_pos);
    static double raw_vel_to_rad_s(double raw_vel);
    /// @}

    
private:
    Options options_;
    std::shared_ptr<EthercatAdapter> _adapter;
    std::vector<MotorState> _motors;

    std::vector<uint8_t> comm_ok_rt_;

    std::atomic<uint64_t> status_overwrite_count_{0};
    MotorStatusChannel status_channel_;
    LatestStatusChannel<MyactDiagnosticsSnapshot> diagnostics_channel_;
    motor_base::RtEventDispatcher rt_event_dispatcher_;
    motor_base::MotorStatusMonitor<MyactDiagnosticsSnapshot> status_monitor_;

    motor_base::CommandSubmitResult validate_command(
        const motor_base::ControlCommand& cmd) const override;
    void apply_setpoint_command_rt(
        const motor_base::ControlCommand& cmd) override;
    void apply_discrete_command_rt(
        int motor_index,
        const motor_base::DiscreteCommand& cmd) override;
    bool is_discrete_command_satisfied_rt(
        int motor_index,
        const motor_base::DiscreteCommand& cmd) const override;
    bool is_motor_comm_ok_rt(int motor_index) const override;
    bool is_motor_faulted_rt(int motor_index) const override;

    void on_discrete_command_failed_rt(
        int motor_index,
        const motor_base::DiscreteCommand& cmd,
        int reason) override;
    void on_discrete_queue_full_rt(
        int motor_index,
        const motor_base::ControlCommand& cmd) override;
    bool on_realtime_start() override;
    void on_realtime_stop() noexcept override;

    void rt_cycle() override;
    void update();
    void update_status_snapshot_rt();
    void update_diagnostics_snapshot_rt();
    void push_rt_event(const motor_base::RtEvent& event);
    void push_status_overwritten_event_rt();
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
