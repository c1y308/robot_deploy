#include "driver/myact/myact_motor_controller.hpp"
#include "driver/myact/myact_debug_printers.hpp"
#include "driver/myact/motor_units.hpp"
#include "tool/tool.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace myactua{

namespace mb = motor_base;

using robot_base::double_to_i16;
using robot_base::double_to_i32;
using robot_base::fits_i16;
using robot_base::fits_i32;

namespace {
constexpr const char* kDefaultEthercatIfName = "enp8s0";

bool is_ankle_motor_index(int motor_index)
{
    return motor_index == 4 ||
           motor_index == 5 ||
           motor_index == 10 ||
           motor_index == 11;
}

double pos_raw_to_rad_for_motor(int motor_index)
{
    return is_ankle_motor_index(motor_index) ? kAnkleRawPosToRad : kRawPosToRad;
}

double pos_rad_to_raw_for_motor(int motor_index)
{
    return is_ankle_motor_index(motor_index) ? kAnkleRadToRawPos : kRadToRawPos;
}

double vel_raw_to_rad_s_for_motor(int motor_index)
{
    return is_ankle_motor_index(motor_index)
        ? kAnkleRawVelToRadPerSec
        : kRawVelToRadPerSec;
}

double vel_rad_s_to_raw_for_motor(int motor_index)
{
    return is_ankle_motor_index(motor_index)
        ? kAnkleRadPerSecToRawVel
        : kRadPerSecToRawVel;
}

std::size_t checked_motor_count(int num_motors)
{
    if (num_motors < 0 ||
        static_cast<std::size_t>(num_motors) > mb::kMaxMotorCommandSetpoints) {
        throw std::invalid_argument("MYACTUA num_motors exceeds fixed realtime capacity");
    }
    return static_cast<std::size_t>(num_motors);
}

// 返回当前命令类型对应的电机运行模式
MyactControlMode expected_mode_for_setpoint(mb::SetpointCommandType type)
{
    switch (type) {
        case mb::SetpointCommandType::POSITION_TARGETS:
            return MyactControlMode::CSP;
        case mb::SetpointCommandType::VELOCITY_TARGETS:
            return MyactControlMode::CSV;
        case mb::SetpointCommandType::TORQUE_TARGETS:
            return MyactControlMode::CST;
        case mb::SetpointCommandType::IMPEDANCE_TARGETS:
            return MyactControlMode::PVT;
    }
    return MyactControlMode::NONE;
}

bool control_ready_for_mode(const MotorState& motor,
                            MyactControlMode expected_mode,
                            bool terminal_fault)
{
    return !terminal_fault &&
           motor.comm_ok &&
           !(motor.observed.sw_faulted || motor.rx.error != 0) &&
           motor.step == MyactMotorStep::RUNNING &&
           motor.observed.operation_enabled &&
           motor.mode_switch_step == MyactModeSwitchStep::IDLE &&
           motor.observed.observed_mode == expected_mode &&
           motor.desired.mode  == expected_mode &&
           expected_mode != MyactControlMode::NONE;
}

bool control_ready_for_current_target(const MotorState& motor,
                                      bool terminal_fault)
{
    return control_ready_for_mode(
        motor,
        motor.desired.mode,
        terminal_fault);
}

}

MyactControlMode MyActMotorController::to_myact_mode(mb::MotorControlMode mode)
{
    switch (mode) {
        case mb::MotorControlMode::NONE:
            return MyactControlMode::NONE;
        case mb::MotorControlMode::POSITION:
            return MyactControlMode::CSP;
        case mb::MotorControlMode::VELOCITY:
            return MyactControlMode::CSV;
        case mb::MotorControlMode::TORQUE:
            return MyactControlMode::CST;
        case mb::MotorControlMode::IMPEDANCE:
            return MyactControlMode::PVT;
    }
    return MyactControlMode::NONE;
}

mb::MotorControlMode MyActMotorController::to_motor_control_mode(MyactControlMode mode)
{
    switch (mode) {
        case MyactControlMode::NONE:
            return mb::MotorControlMode::NONE;
        case MyactControlMode::CSP:
            return mb::MotorControlMode::POSITION;
        case MyactControlMode::CSV:
            return mb::MotorControlMode::VELOCITY;
        case MyactControlMode::CST:
            return mb::MotorControlMode::TORQUE;
        case MyactControlMode::PVT:
            return mb::MotorControlMode::IMPEDANCE;
    }
    return mb::MotorControlMode::NONE;
}

/* 电机控制器构造函数 */
MyActMotorController::MyActMotorController(std::shared_ptr<EthercatAdapter> adapter, int num_motors)
    : MyActMotorController(std::move(adapter), num_motors, Options())
{
}

MyActMotorController::MyActMotorController(std::shared_ptr<EthercatAdapter> adapter,
                 int num_motors,
                 Options options)
    : mb::MotorControllerBase(checked_motor_count(num_motors), options),
      options_(options),
      _adapter(std::move(adapter))
{
    if (!_adapter) {
        throw std::invalid_argument("MYACTUA requires a non-null EtherCAT adapter");
    }
    if (options_.comm_watchdog_fault_cycles == 0) {
        throw std::invalid_argument(
            "MYACTUA comm_watchdog_fault_cycles must be positive");
    }

    /* 初始化电机状态列表 */
    for (int i = 0; i < num_motors; i++) {
        _motors.emplace_back(i);
    }

    set_event_fallback_printer(print_myact_event);

    diagnostics_channel_.configure(_motors.size(),
                                   options_.status_publish_period_ms,
                                   "motor_diag",
                                   options_.background_thread_options);

    status_monitor_.set_status_provider([this]() { return get_myact_diagnostics(); });
    status_monitor_.set_status_printer(print_myact_status_table);
    status_monitor_.configure_thread(options_.background_thread_options);
    
    _adapter->set_event_sink(this, &MyActMotorController::event_sink_trampoline);
}

/* 析构函数 */
MyActMotorController::~MyActMotorController()
{
    shutdown();
    _adapter->set_event_sink(nullptr, nullptr);
}

/* 连接网卡函数 */
bool MyActMotorController::connect_impl(const char* ifname)
{
    const char* effective_ifname =
        (ifname && ifname[0] != '\0') ? ifname : kDefaultEthercatIfName;
    return _adapter->init(effective_ifname);
}


/// @brief 阻塞等待所有电机进入 OP（每 1ms 检查一次）
/// @param timeout_ms 超时时间 (ms)，0 表示仅检查一次
/// @param poll_ms  日志打印间隔 (ms)
bool MyActMotorController::wait_all_motors_ready(int timeout_ms, int poll_ms) const
{
    if (timeout_ms < 0) {
        throw std::invalid_argument("timeout_ms must be non-negative");
    }
    if (poll_ms <= 0) {
        throw std::invalid_argument("poll_ms must be positive");
    }

    using Clock = std::chrono::steady_clock;

    const auto start_time = Clock::now();
    const auto deadline = start_time + std::chrono::milliseconds(timeout_ms);
    
    auto next_log_time = start_time;
    bool first_check = true;

    while (first_check || (timeout_ms > 0 && Clock::now() < deadline)) {
        first_check = false;

        _adapter->receive_physical();
        _adapter->send_physical();

        int ready_count = 0;
        for (int i = 0; i < static_cast<int>(_motors.size()); ++i) {
            if (_adapter->is_configured(i)) {
                ++ready_count;
            }
        }

        if (ready_count == static_cast<int>(_motors.size())) {
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                Clock::now() - start_time).count();
            std::cout << "[MYACTUA] All slaves ready in "
                      << elapsed_ms << " ms" << std::endl;
            return true;
        }

        const auto now = Clock::now();
        if (now >= next_log_time) {
            std::cout << "[MYACTUA] EtherCAT ready: "
                      << ready_count << "/" << _motors.size() << std::endl;
            next_log_time = now + std::chrono::milliseconds(poll_ms);
        }

        if (timeout_ms == 0) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        Clock::now() - start_time).count();

    std::cout << "[MYACTUA] wait_all_motors_ready timeout after "
              << elapsed_ms << " ms" << std::endl;

    return false;
}


bool MyActMotorController::realtime_start_callback()
{
    process_data_fail_count_ = 0;

    if (!diagnostics_channel_.start()) {
        std::cerr << "[MYACTUA] motor_diag setup failed: "
                  << diagnostics_channel_.last_start_error() << std::endl;
        return false;
    }
    if (status_monitor_.has_print_motor_ids()) {
        if (!status_monitor_.start()) {
            std::cerr << "[MYACTUA] motor_mon setup failed: "
                      << status_monitor_.last_start_error() << std::endl;
            diagnostics_channel_.stop();
            return false;
        }
    }
    
    std::cout << "[MYACTUA] 实时控制线程已启动" << std::endl;
    return true;
}


/* 电机实时控制单周期 */
void MyActMotorController::realtime_cycle_callback()
{
    _adapter->receive_physical();
    current_cycle_host_timestamp_ns_ = robot_base::monotonic_now_ns();
    update();
    _adapter->send_physical();

    update_realtime_feedback();
    update_status_snapshot();
    update_diagnostics_snapshot();
}


void MyActMotorController::realtime_stop_callback() noexcept
{
    status_monitor_.stop();
    diagnostics_channel_.stop();
    std::cout << "[MYACTUA] 实时控制线程已停止" << std::endl;
}


/// @brief 更新 motor.observed，处理通信、故障、模式切换和目标值设置
void MyActMotorController::update()
{
    const EthercatBusHealthSnapshot health = _adapter->get_bus_health();
    const bool process_data_ok =
        health.master_link_up && health.wc_state == EC_WC_COMPLETE;
    process_data_ok_ = process_data_ok;

    /* 接受电机回传数据，并记录当前周期通信状态 */
    for (size_t i = 0; i < _motors.size(); i++)
    {
        const bool comm_ok = _adapter->is_configured(_motors[i].motor_index);
        if (!comm_ok) {
            ++_motors[i].comm_offline_total_count;
        }
        _motors[i].comm_ok = comm_ok;
        
        /* 只接受通信正常的电机数据 */
        if (comm_ok) {
            MotorState& motor = _motors[i];
            motor.rx = _adapter->receive(motor.motor_index);
            const uint16_t sw = motor.rx.status_word;
            motor.observed.observed_mode = static_cast<MyactControlMode>(motor.rx.op_mode);
            motor.observed.operation_enabled = is_operation_enabled(sw);
            motor.observed.sw_faulted = is_fault(sw);
            motor.observed.position_rad =
                static_cast<double>(motor.rx.pos) *
                pos_raw_to_rad_for_motor(motor.motor_index);
            motor.observed.velocity_rad_s =
                static_cast<double>(motor.rx.vel) *
                vel_raw_to_rad_s_for_motor(motor.motor_index);
            motor.observed.torque_percent =
                static_cast<double>(motor.rx.torque) * kRawTorqueToPercent;
        }
    }

    /* 任一轴退出稳定运行状态时，锁存整机保护。主动 STOP、RESTART 和模式切换
       会先改变 desired/step，因此不会被视为意外失能或模式漂移。 */
    for (const auto& motor : _motors) {
        const bool expected_running =
            motor.step == MyactMotorStep::RUNNING &&
            motor.desired.enabled &&
            motor.mode_switch_step == MyactModeSwitchStep::IDLE;
        const bool drive_fault =
            motor.comm_ok &&
            (motor.observed.sw_faulted || motor.rx.error != 0);
        const bool unexpectedly_disabled =
            motor.comm_ok && expected_running &&
            !motor.observed.operation_enabled;
        const bool unexpected_mode =
            motor.comm_ok && expected_running &&
            motor.observed.observed_mode != motor.desired.mode;

        if (!motor.comm_ok || drive_fault || unexpectedly_disabled || unexpected_mode) {
            latch_terminal_fault();
            break;
        }
    }

    update_communication_watchdog(process_data_ok, health);
    if (terminal_fault_latched()) {
        apply_whole_body_quick_stop();
        return;
    }

    /* 未达到 watchdog 锁存阈值时，沿用原有单电机状态机。 */
    for (size_t i = 0; i < _motors.size(); i++)
    {
        if (!_motors[i].comm_ok) {
            continue;
        }
        process_single_motor(_motors[i]);
    }

    /* 最终发送 */
    for (size_t i = 0; i < _motors.size(); i++)
    {
        if (!_motors[i].comm_ok) continue;
        _adapter->send(_motors[i].motor_index, _motors[i].tx);
    }
}


void MyActMotorController::update_communication_watchdog(
    bool process_data_ok,
    const EthercatBusHealthSnapshot& health)
{
    if (terminal_fault_latched()) {
        return;
    }

    if (process_data_ok) {
        process_data_fail_count_ = 0;
        return;
    }

    ++process_data_fail_count_;
    if (process_data_fail_count_ >= options_.comm_watchdog_fault_cycles) {
        const MyactCommunicationFaultReason reason = health.master_link_up
            ? MyactCommunicationFaultReason::WkcIncomplete
            : MyactCommunicationFaultReason::LinkDown;
        latch_communication_fault(reason, health);
    }
}


void MyActMotorController::latch_communication_fault(
    MyactCommunicationFaultReason reason,
    const EthercatBusHealthSnapshot& health)
{
    if (!latch_terminal_fault()) {
        return;
    }

    fault_reason_ = reason;

    mb::RtEvent event;
    event.type = mb::RtEventType::COMM_WATCHDOG_FAULT;
    event.tick = discrete_command_tick();
    event.motor_index = -1;
    event.reason = static_cast<int>(fault_reason_);
    event.value = health.working_counter;
    push_event(event);
}


void MyActMotorController::apply_whole_body_quick_stop()
{
    for (auto& motor : _motors) {
        motor.step = MyactMotorStep::FAULT;
        motor.mode_switch_step = MyactModeSwitchStep::IDLE;
        motor.desired.enabled = false;

        reset_motor_setpoints_to_feedback(motor);
        motor.tx.control_word = options_.comm_fault_control_word;
        motor.tx.op_mode = static_cast<int8_t>(motor.desired.mode);

        _adapter->send(motor.motor_index, motor.tx);
    }
}


void MyActMotorController::reset_motor_setpoints_to_feedback(MotorState& motor)
{
    const double position_rad = motor.observed.position_rad;

    motor.desired.position_rad = position_rad;
    motor.desired.velocity_rad_s = 0.0;
    motor.desired.torque = 0.0;
    motor.desired.impedance_setpoint =
        mb::ImpedanceSetpoint(position_rad, 0.0, 0.0, 0.0, 0.0);

    motor.tx.target_pos = motor.rx.pos;
    motor.tx.target_vel = 0;
    motor.tx.target_torque = 0;
    motor.tx.pvt_kp = 0;
    motor.tx.pvt_kd = 0;
}


/// @brief 处理单个电机的状态机逻辑
void MyActMotorController::process_single_motor(MotorState& motor)
{
    const auto& desired   = motor.desired;
    const uint16_t sw = motor.rx.status_word;

    /* 处理故障状态 */
    if (motor.observed.sw_faulted || motor.rx.error != 0) {
        motor.step = MyactMotorStep::FAULT;
        motor.mode_switch_step = MyactModeSwitchStep::IDLE;
        motor.tx.control_word = CMD_SHUTDOWN;
        motor.tx.op_mode = static_cast<int8_t>(desired.mode);
        motor.tx.target_pos = motor.rx.pos;
        motor.tx.target_vel = 0;
        motor.tx.target_torque = 0;
        motor.tx.pvt_kp = 0;
        motor.tx.pvt_kd = 0;
        return;
    }

    /* 需要停止 */
    if (!desired.enabled) {
        motor.step = MyactMotorStep::STOPPED;
        motor.mode_switch_step = MyactModeSwitchStep::IDLE;
        motor.tx.control_word = CMD_DISABLE_OPERATION;
        motor.tx.op_mode = static_cast<int8_t>(desired.mode);
        motor.tx.target_vel = 0;
        motor.tx.target_pos = motor.rx.pos;
        motor.tx.target_torque = 0;
        motor.tx.pvt_kp = 0;
        motor.tx.pvt_kd = 0;
        return;
    }

    if (desired.mode == MyactControlMode::NONE) {
        motor.step = MyactMotorStep::IDLE;
        motor.mode_switch_step = MyactModeSwitchStep::IDLE;
        motor.tx.control_word = CMD_SHUTDOWN;
        motor.tx.op_mode = static_cast<int8_t>(MyactControlMode::NONE);
        motor.tx.target_pos = motor.rx.pos;
        motor.tx.target_vel = 0;
        motor.tx.target_torque = 0;
        motor.tx.pvt_kp = 0;
        motor.tx.pvt_kd = 0;
        return;
    }

    // 是否进入电机状态切换的状态机
    const bool mode_switch_active =
         (motor.observed.observed_mode != desired.mode) || (motor.mode_switch_step != MyactModeSwitchStep::IDLE);
    if (mode_switch_active) {
        motor.step = MyactMotorStep::MODE_SWITCHING;
        handle_mode_switching(motor);
        // 模式切换采用闭环状态机，本周期不进入常规运行逻辑，避免覆盖切换命令。
        return;
    }

    /* 模式一致则进入使能/运行控制 */
    if (!is_switched_on(sw) && !is_ready_to_switch_on(sw)) {
        motor.tx.control_word = CMD_SHUTDOWN;
    } else if (!is_switched_on(sw) && is_ready_to_switch_on(sw)) {
        motor.tx.control_word = CMD_SWITCH_ON;
    } else {
        motor.tx.control_word = CMD_ENABLE_OPERATION;
    }
    motor.tx.op_mode = static_cast<int8_t>(desired.mode);

    if (motor.observed.operation_enabled) {
        motor.step = MyactMotorStep::RUNNING;
        motor.tx.pvt_kp = 0;
        motor.tx.pvt_kd = 0;
        const double pos_rad_to_raw = pos_rad_to_raw_for_motor(motor.motor_index);
        const double vel_rad_s_to_raw =
            vel_rad_s_to_raw_for_motor(motor.motor_index);
        switch (desired.mode) {
            case MyactControlMode::CSV:
                motor.tx.target_vel = double_to_i32(
                    desired.velocity_rad_s * vel_rad_s_to_raw);
                break;
            case MyactControlMode::CSP:
                motor.tx.target_pos = double_to_i32(
                    desired.position_rad * pos_rad_to_raw);
                break;
            case MyactControlMode::CST:
                motor.tx.target_torque = double_to_i16(desired.torque);
                break;
            case MyactControlMode::PVT: {
                const mb::ImpedanceSetpoint& impedance = desired.impedance_setpoint;
                motor.tx.target_pos = double_to_i32(
                    impedance.position_rad * pos_rad_to_raw);
                motor.tx.target_vel = double_to_i32(
                    impedance.velocity_rad_s * vel_rad_s_to_raw);
                motor.tx.target_torque = double_to_i16(impedance.effort_ff);
                motor.tx.pvt_kp = double_to_i32(impedance.kp * 1000.0);
                motor.tx.pvt_kd = double_to_i32(impedance.kd * 1000.0);
                break;
            }
            case MyactControlMode::NONE:
                break;
            default:
                break;
        }
    } else {
        motor.step = MyactMotorStep::ENABLING;
        motor.tx.target_pos = motor.rx.pos;
        motor.tx.target_vel = 0;
        motor.tx.target_torque = 0;
        motor.tx.pvt_kp = 0;
        motor.tx.pvt_kd = 0;
    }
}


/* 状态机切换电机控制模式 */
void MyActMotorController::handle_mode_switching(MotorState& motor)
{
    const auto& desired  = motor.desired;
    const uint16_t sw = motor.rx.status_word;
    const bool mode_ok = (motor.observed.observed_mode == desired.mode);

    // 模式切换期间固定目标，避免切换过程中产生突变。
    motor.tx.op_mode = static_cast<int8_t>(desired.mode);
    motor.tx.target_pos = motor.rx.pos;
    motor.tx.target_vel = 0;
    motor.tx.target_torque = 0;
    motor.tx.pvt_kp = 0;
    motor.tx.pvt_kd = 0;

    switch (motor.mode_switch_step)
    {
        case MyactModeSwitchStep::IDLE:
            // 1) 写 0x6060(目标模式)
            motor.tx.control_word = CMD_SHUTDOWN;
            motor.mode_switch_step = MyactModeSwitchStep::SET_MODE;
            break;

        case MyactModeSwitchStep::SET_MODE:
            // 保持 0x6040=6，在失能态等待 0x6061 回读到目标模式，闭环推进。
            motor.tx.control_word = CMD_SHUTDOWN;
            if (mode_ok) {
                motor.mode_switch_step = MyactModeSwitchStep::CLEAR;
            }
            break;

        case MyactModeSwitchStep::CLEAR:
            // 2) 读 0x6064，并将 0x607A 对齐到当前位置。
            // 对齐命令至少发送一个周期后再进入下一步。
            motor.tx.control_word = CMD_SHUTDOWN;
            if (!mode_ok) {
                motor.mode_switch_step = MyactModeSwitchStep::SET_MODE;
            } else {
                motor.mode_switch_step = MyactModeSwitchStep::DISABLE;
            }
            break;

        case MyactModeSwitchStep::DISABLE:
            // 3-1) 写 0x6040=6，等待状态字进入 Ready to switch on。
            motor.tx.control_word = CMD_SHUTDOWN;
            if (!mode_ok) {
                motor.mode_switch_step = MyactModeSwitchStep::SET_MODE;
                break;
            }
            if (is_ready_to_switch_on(sw) && !is_switched_on(sw) && !is_operation_enabled(sw)) {
                motor.mode_switch_step = MyactModeSwitchStep::ENABLE;
            }
            break;

        case MyactModeSwitchStep::ENABLE:
            // 3-2) 写 0x6040=7，等待状态字进入 Switched on。
            motor.tx.control_word = CMD_SWITCH_ON;
            if (!mode_ok) {
                motor.mode_switch_step = MyactModeSwitchStep::SET_MODE;
                break;
            }
            if (is_switched_on(sw) && !is_operation_enabled(sw)) {
                motor.mode_switch_step = MyactModeSwitchStep::OPERATING;
            } else if (!is_ready_to_switch_on(sw)) {
                motor.mode_switch_step = MyactModeSwitchStep::DISABLE;
            }
            break;

        case MyactModeSwitchStep::OPERATING:
            // 3-3) 写 0x6040=15，等待状态字进入 Operation enabled。
            motor.tx.control_word = CMD_ENABLE_OPERATION;
            if (!mode_ok) {
                motor.mode_switch_step = MyactModeSwitchStep::SET_MODE;
                break;
            }
            if (is_operation_enabled(sw)) {
                motor.mode_switch_step = MyactModeSwitchStep::DONE;
            } else if (!is_switched_on(sw)) {
                motor.mode_switch_step = MyactModeSwitchStep::ENABLE;
            }
            break;

        case MyactModeSwitchStep::DONE:
            // 4) 闭环完成: 模式正确且已使能，切回常规运行控制。
            motor.tx.control_word = CMD_ENABLE_OPERATION;
            if (mode_ok && is_operation_enabled(sw)) {
                motor.mode_switch_step = MyactModeSwitchStep::IDLE;
                motor.step = MyactMotorStep::RUNNING;
            } else {
                motor.mode_switch_step = mode_ok ? MyactModeSwitchStep::OPERATING : MyactModeSwitchStep::SET_MODE;
            }
            break;
    }
}


mb::CommandSubmitStatus MyActMotorController::validate_command(
    const mb::ControlCommand& cmd) const
{
    if (cmd.kind == mb::ControlCommandKind::SETPOINT) {

        /* 是否为单电机控制 */
        const bool single_motor = (cmd.motor_index != mb::ControlCommand::kAllMotors);
        if (single_motor &&
            (cmd.motor_index < 0 || cmd.motor_index >= static_cast<int>(_motors.size()))) {
            return mb::CommandSubmitStatus::INVALID_COMMAND;
        }
        
        // 单电机命令只校验目标电机，目标值位于 setpoints[0]；
        // 全体命令按数组位置逐电机校验。
        const std::size_t begin = single_motor
                                      ? static_cast<std::size_t>(cmd.motor_index)
                                      : 0;
        const std::size_t end = single_motor ? begin + 1 : _motors.size();

        for (std::size_t i = begin; i < end; ++i) {
            const std::size_t slot = single_motor ? 0 : i;
            const int motor_id = _motors[i].motor_index;
            switch (cmd.setpoint_type) {
                case mb::SetpointCommandType::POSITION_TARGETS:
                    if (!fits_i32(cmd.setpoints[slot] *
                                  pos_rad_to_raw_for_motor(motor_id))) {
                        return mb::CommandSubmitStatus::INVALID_PAYLOAD;
                    }
                    break;

                case mb::SetpointCommandType::VELOCITY_TARGETS:
                    if (!fits_i32(cmd.setpoints[slot] *
                                  vel_rad_s_to_raw_for_motor(motor_id))) {
                        return mb::CommandSubmitStatus::INVALID_PAYLOAD;
                    }
                    break;

                case mb::SetpointCommandType::TORQUE_TARGETS:
                    if (!fits_i16(cmd.setpoints[slot])) {
                        return mb::CommandSubmitStatus::INVALID_PAYLOAD;
                    }
                    break;

                case mb::SetpointCommandType::IMPEDANCE_TARGETS: {
                    const mb::ImpedanceSetpoint& setpoint =
                        cmd.impedance_setpoints[slot];
                    if (!fits_i32(setpoint.position_rad *
                                  pos_rad_to_raw_for_motor(motor_id)) ||
                        !fits_i32(setpoint.velocity_rad_s *
                                  vel_rad_s_to_raw_for_motor(motor_id)) ||
                        !fits_i16(setpoint.effort_ff) ||
                        !fits_i32(setpoint.kp * 1000.0) ||
                        !fits_i32(setpoint.kd * 1000.0)) {
                        return mb::CommandSubmitStatus::INVALID_PAYLOAD;
                    }
                    break;
                }
            }
        }
    }

    return mb::CommandSubmitStatus::ACCEPTED;
}

/// @brief 执行连续目标值命令
void MyActMotorController::apply_setpoint_command_impl(const mb::ControlCommand& cmd)
{
    const bool terminal_fault = terminal_fault_latched();
    if (terminal_fault) {
        return;
    }

    const bool single_motor =
        (cmd.motor_index != mb::ControlCommand::kAllMotors);
    const std::size_t begin = single_motor
                                  ? static_cast<std::size_t>(cmd.motor_index)
                                  : 0;
    const std::size_t end = single_motor ? begin + 1 : _motors.size();

    const MyactControlMode expected_mode = expected_mode_for_setpoint(cmd.setpoint_type);
    for (std::size_t i = begin; i < end; ++i) {
        if (!control_ready_for_mode(_motors[i], expected_mode, terminal_fault)) {
            if (latch_terminal_fault()) {
                mb::RtEvent event;
                event.type = mb::RtEventType::SETPOINT_COMMAND_REJECTED;
                event.tick = discrete_command_tick();
                event.motor_index = _motors[i].motor_index;
                event.reason = static_cast<int>(
                    mb::SetpointRejectReason::MODE_NOT_CONFIRMED);
                event.value = static_cast<uint32_t>(cmd.setpoint_type);
                push_event(event);
            }
            return;
        }
    }

    switch (cmd.setpoint_type) {
        case mb::SetpointCommandType::POSITION_TARGETS:
            for (size_t i = begin; i < end; i++) {
                _motors[i].desired.position_rad =
                    cmd.setpoints[single_motor ? 0 : i];
            }
            break;

        case mb::SetpointCommandType::VELOCITY_TARGETS:
            for (size_t i = begin; i < end; i++) {
                _motors[i].desired.velocity_rad_s =
                    cmd.setpoints[single_motor ? 0 : i];
            }
            break;

        case mb::SetpointCommandType::TORQUE_TARGETS:
            for (size_t i = begin; i < end; i++) {
                _motors[i].desired.torque =
                    cmd.setpoints[single_motor ? 0 : i];
            }
            break;

        case mb::SetpointCommandType::IMPEDANCE_TARGETS:
            for (size_t i = begin; i < end; i++) {
                _motors[i].desired.impedance_setpoint =
                    cmd.impedance_setpoints[single_motor ? 0 : i];
            }
            break;
    }
}


/// @brief 执行电机离散命令队列中的命令
void MyActMotorController::apply_discrete_command_impl(
    int motor_index,
    const mb::DiscreteCommand& cmd)
{
    if (motor_index < 0 || motor_index >= static_cast<int>(_motors.size())) {
        return;
    }

    MotorState& motor = _motors[motor_index];
    
    if (terminal_fault_latched()) {
        if (cmd.type == mb::DiscreteCommandType::STOP) {
            motor.desired.enabled = false;
            motor.mode_switch_step = MyactModeSwitchStep::IDLE;
        }
        return;
    }

    switch (cmd.type) {
        case mb::DiscreteCommandType::STOP:
            // Edge-triggered: avoid resetting mode-switch state on retries.
            if (motor.desired.enabled) {
                motor.desired.enabled = false;
                motor.mode_switch_step = MyactModeSwitchStep::IDLE;
            }
            break;
        case mb::DiscreteCommandType::RESTART:
            // Edge-triggered: first restart arms enable flow; later retries are no-op.
            if (!motor.desired.enabled) {
                reset_motor_setpoints_to_feedback(motor);
                motor.desired.enabled = true;
                motor.mode_switch_step = MyactModeSwitchStep::IDLE;
            }
            break;
        case mb::DiscreteCommandType::SET_MODE:
            if (motor.desired.mode != to_myact_mode(cmd.mode)) {
                motor.desired.mode  = to_myact_mode(cmd.mode);
                reset_motor_setpoints_to_feedback(motor);
            }
            motor.mode_switch_step = MyactModeSwitchStep::SET_MODE;
            break;
    }
}


mb::DiscreteCommandEvaluation MyActMotorController::evaluate_discrete_command_impl(
    int motor_index,
    const mb::DiscreteCommand& cmd) const
{
    if (motor_index < 0 || motor_index >= static_cast<int>(_motors.size())) {
        return mb::DiscreteCommandEvaluation::PENDING;
    }

    const MotorState& motor = _motors[static_cast<std::size_t>(motor_index)];

    if (cmd.type == mb::DiscreteCommandType::STOP) {
        return process_data_ok_ && motor.comm_ok && !motor.observed.operation_enabled
            ? mb::DiscreteCommandEvaluation::SATISFIED
            : mb::DiscreteCommandEvaluation::PENDING;
    }

    const bool terminal_fault = terminal_fault_latched();

    if (terminal_fault) {
        switch (cmd.type) {
            case mb::DiscreteCommandType::RESTART:
                return mb::DiscreteCommandEvaluation::FAILED;

            case mb::DiscreteCommandType::SET_MODE:
                return mb::DiscreteCommandEvaluation::FAILED;
        }
    }

    if (!motor.comm_ok) {
        return mb::DiscreteCommandEvaluation::PENDING;
    }

    if (motor.observed.sw_faulted || motor.rx.error != 0) {
        return mb::DiscreteCommandEvaluation::FAILED;
    }

    bool satisfied = false;
    switch (cmd.type) {
        case mb::DiscreteCommandType::RESTART:
            satisfied = control_ready_for_current_target(motor, terminal_fault);
            break;
        case mb::DiscreteCommandType::SET_MODE:
            satisfied = motor.observed.observed_mode == to_myact_mode(cmd.mode);
            break;
    }
    return satisfied ? mb::DiscreteCommandEvaluation::SATISFIED
                     : mb::DiscreteCommandEvaluation::PENDING;
}


void MyActMotorController::discrete_command_failed_callback(
    int motor_index,
    const mb::DiscreteCommand& cmd,
    mb::DiscreteFailReason reason)
{
    mb::RtEvent event;
    event.type = mb::RtEventType::DISCRETE_COMMAND_FAILED;
    event.tick = discrete_command_tick();
    event.motor_index = motor_index;
    event.command_type = cmd.type;
    event.reason = static_cast<int>(reason);
    event.value = static_cast<uint32_t>(cmd.cur_retry);
    push_event(event);
}


void MyActMotorController::discrete_queue_full_callback(
    int motor_index,
    const mb::ControlCommand& cmd)
{
    mb::RtEvent event;
    event.type = mb::RtEventType::DISCRETE_QUEUE_FULL;
    event.tick = discrete_command_tick();
    event.motor_index = motor_index;
    event.command_type = cmd.discrete_type;
    event.reason = static_cast<int>(mb::DiscreteFailReason::MAX_RETRY);
    push_event(event);
}


void MyActMotorController::update_realtime_feedback()
{
    if (_motors.empty()) {
        return;
    }

    std::array<mb::MotorStatusSnapshot, mb::kMaxMotorCommandSetpoints> feedback{};

    /* 把 MotorState 中的数据填充给对外发布的实时反馈快照 */
    const bool terminal_fault = terminal_fault_latched();

    for (std::size_t i = 0; i < _motors.size(); ++i) {

        const auto& motor = _motors[i];

        feedback[i].motor_index       = motor.motor_index;
        feedback[i].host_timestamp_ns = current_cycle_host_timestamp_ns_;
        feedback[i].position_rad      = motor.observed.position_rad;
        feedback[i].velocity_rad_s    = motor.observed.velocity_rad_s;
        feedback[i].torque_percent    = motor.observed.torque_percent;
        feedback[i].comm_ok           = motor.comm_ok;

        if (motor.comm_ok) {
            feedback[i].enabled = motor.observed.operation_enabled;
            feedback[i].faulted = terminal_fault ||
                                  motor.observed.sw_faulted ||
                                  (motor.rx.error != 0);
            feedback[i].mode = to_motor_control_mode(motor.observed.observed_mode);
        } else {
            feedback[i].enabled = false;
            feedback[i].faulted = terminal_fault;
            feedback[i].mode    = mb::MotorControlMode::NONE;
        }
        feedback[i].control_ready =
            control_ready_for_current_target(motor, terminal_fault);
        feedback[i].target_mode = to_motor_control_mode(motor.desired.mode);
    }

    publish_feedback(feedback);
}


void MyActMotorController::update_status_snapshot()
{
    if (_motors.empty()) {
        return;
    }

    mb::MotorControllerBase::StatusWriteToken write_token;
    if (!write_status(write_token)) {
        push_status_channel_busy_event();
        return;
    }

    mb::MotorStatusSnapshot* status_slot = write_token.data;
    const bool terminal_fault = terminal_fault_latched();
    for (size_t i = 0; i < _motors.size(); i++) {
        const auto& m = _motors[i];
        auto& s = status_slot[i];
        s.motor_index = m.motor_index;
        s.host_timestamp_ns = current_cycle_host_timestamp_ns_;
        s.position_rad = m.observed.position_rad;
        s.velocity_rad_s = m.observed.velocity_rad_s;
        s.torque_percent = m.observed.torque_percent;
        s.comm_ok = m.comm_ok;
        if (m.comm_ok) {
            s.enabled = m.observed.operation_enabled;
            s.faulted = terminal_fault || m.observed.sw_faulted || (m.rx.error != 0);
            s.mode = to_motor_control_mode(m.observed.observed_mode);
        } else {
            s.enabled = false;
            s.faulted = terminal_fault;
            s.mode = mb::MotorControlMode::NONE;
        }
        s.control_ready = control_ready_for_current_target(m, terminal_fault);
        s.target_mode   = to_motor_control_mode(m.desired.mode);
    }

    publish_status(write_token);
}


void MyActMotorController::update_diagnostics_snapshot()
{
    if (_motors.empty()) {
        return;
    }

    mb::LatestStatusChannel<MotorState>::WriteToken write_token;
    if (!diagnostics_channel_.write(write_token)) {
        push_status_channel_busy_event();
        return;
    }

    MotorState* diagnostics_slot = write_token.data;
    for (size_t i = 0; i < _motors.size(); i++) {
        diagnostics_slot[i] = _motors[i];
    }

    diagnostics_channel_.publish(write_token);
}


void MyActMotorController::set_myact_diagnostics_callback(MyactDiagnosticsCallback cb)
{
    diagnostics_channel_.set_callback(std::move(cb));
}

std::vector<MotorState> MyActMotorController::get_myact_diagnostics()
{
    return diagnostics_channel_.get_status();
}

void MyActMotorController::event_sink_trampoline(void* context, const mb::RtEvent& event)
{
    static_cast<MyActMotorController*>(context)->push_event(event);
}


void MyActMotorController::push_status_channel_busy_event()
{
    const uint64_t count =
        status_channel_busy_count_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (count != 1 && (count & (count - 1)) != 0) {
        return;
    }

    mb::RtEvent event;
    event.type = mb::RtEventType::STATUS_CHANNEL_BUSY;
    event.tick = discrete_command_tick();
    event.value = static_cast<uint32_t>(
        std::min<uint64_t>(count, static_cast<uint64_t>(UINT32_MAX)));
    push_event(event);
}


/* 配置电机监控打印: 空列表关闭打印，-1 表示全部电机 */
void MyActMotorController::set_print_info(const std::vector<int>& motor_indices)
{
    const bool enabled = status_monitor_.set_print_info(motor_indices);
    if (is_running()) {
        if (enabled) {
            if (!status_monitor_.start()) {
                std::cerr << "[MYACTUA] motor_mon setup failed: "
                          << status_monitor_.last_start_error() << std::endl;
                latch_terminal_fault();
            }
        } else {
            status_monitor_.stop();
        }
    }
}

}
