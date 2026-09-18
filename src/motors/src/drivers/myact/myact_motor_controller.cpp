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
constexpr uint32_t kStartupWkcStableCycles = 10;

const char* wc_state_name(ec_wc_state_t state)
{
    switch (state) {
        case EC_WC_ZERO:
            return "ZERO";
        case EC_WC_INCOMPLETE:
            return "INCOMPLETE";
        case EC_WC_COMPLETE:
            return "COMPLETE";
    }
    return "UNKNOWN";
}

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

uint32_t encode_mode_fault_value(MyactControlMode actual, MyactControlMode target)
{
    return (static_cast<uint32_t>(static_cast<uint8_t>(actual)) << 8U) |
           static_cast<uint32_t>(static_cast<uint8_t>(target));
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

    status_monitor_.set_status_printer(print_myact_status_table);
    status_monitor_.configure(_motors.size(), options_.background_thread_options);

    _adapter->set_event_sink(this, &MyActMotorController::event_sink_trampoline);
}

/* 析构函数 */
MyActMotorController::~MyActMotorController()
{
    shutdown();
    _adapter->set_event_sink(nullptr, nullptr);
}

/* 初始化 EtherCAT 网络 */
bool MyActMotorController::connect_impl()
{
    return _adapter->init();
}


/// @brief 阻塞等待所有电机进入 OP 且过程数据连续稳定（每 1ms 检查一次）
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
    uint32_t stable_wkc_cycles = 0;
    const uint32_t required_stable_cycles =
        timeout_ms == 0 ? 1U : kStartupWkcStableCycles;

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

        const EthercatBusHealthSnapshot health = _adapter->get_bus_health();
        const bool all_slaves_ready =
            ready_count == static_cast<int>(_motors.size());
        const bool process_data_complete =
            health.master_link_up && health.wc_state == EC_WC_COMPLETE;
        if (all_slaves_ready && process_data_complete) {
            ++stable_wkc_cycles;
        } else {
            stable_wkc_cycles = 0;
        }

        if (stable_wkc_cycles >= required_stable_cycles) {
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                Clock::now() - start_time).count();
            std::cout << "[MYACTUA] All slaves and process data stable for "
                      << required_stable_cycles << " cycle(s) in "
                      << elapsed_ms << " ms" << std::endl;
            return true;
        }

        const auto now = Clock::now();
        if (now >= next_log_time) {
            std::cout << "[MYACTUA] EtherCAT ready: "
                      << ready_count << "/" << _motors.size()
                      << ", link=" << (health.master_link_up ? "up" : "down")
                      << ", wc=" << health.working_counter
                      << ", wc_state=" << wc_state_name(health.wc_state)
                      << ", stable=" << stable_wkc_cycles
                      << "/" << required_stable_cycles << std::endl;
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

    if (status_monitor_.has_print_motor_ids()) {
        if (!status_monitor_.start()) {
            std::cerr << "[MYACTUA] motor_mon setup failed: "
                      << status_monitor_.last_start_error() << std::endl;
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
    if (status_monitor_.has_print_motor_ids()) update_diagnostics_snapshot();
}


void MyActMotorController::realtime_stop_callback() noexcept
{
    status_monitor_.stop();
    std::cout << "[MYACTUA] 实时控制线程已停止" << std::endl;
}


/// @brief 更新 motor.observed，处理通信、故障、模式切换和目标值设置
void MyActMotorController::update()
{
    const bool communication_protection_enabled =
        communication_protection_enabled_.load(std::memory_order_acquire);
    const EthercatBusHealthSnapshot health = _adapter->get_bus_health();
    const bool process_data_ok =
        health.master_link_up && health.wc_state == EC_WC_COMPLETE;
    process_data_ok_ = process_data_ok;

    /* 接受电机回传数据，并记录当前周期通信状态 */
    for (size_t i = 0; i < _motors.size(); i++)
    {
        const bool comm_ok = _adapter->is_configured(_motors[i].motor_index);
        if (status_monitor_.has_print_motor_ids() && !comm_ok) {
            ++_motors[i].comm_offline_total_count;
        }
        _motors[i].comm_ok = comm_ok;
        
        /* 只有完整的本周期 PDO 才更新观测及采样时间。 */
        if (process_data_ok && comm_ok) {
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
            motor.last_valid_host_timestamp_ns = current_cycle_host_timestamp_ns_;
        }
    }

    /* 任一轴退出稳定运行状态时，锁存整机保护。主动 STOP、RESTART 和模式切换
       会先改变 desired/step，因此不会被视为意外失能或模式漂移。 */
    for (auto& motor : _motors) {
        const bool expected_running =
            motor.step == MyactMotorStep::RUNNING &&
            motor.desired.enabled &&
            motor.mode_switch_step == MyactModeSwitchStep::IDLE;

        MyactMotorFaultReason reason = MyactMotorFaultReason::None;
        uint32_t raw_value = 0;
        // 避免正式推理前的启动通信瞬态被提前锁存为 OFFLINE；
        // 不修改实际通信状态，推理开始后恢复原有保护。
        if (!motor.comm_ok) {
            if (communication_protection_enabled) {
                reason = MyactMotorFaultReason::Offline;
            }
        } else if (motor.observed.sw_faulted) {
            reason = MyactMotorFaultReason::StatusWordFault;
            raw_value = motor.rx.status_word;
        } else if (motor.rx.error != 0) {
            reason = MyactMotorFaultReason::ErrorCode;
            raw_value = motor.rx.error;
        } else if (expected_running && !motor.observed.operation_enabled) {
            reason = MyactMotorFaultReason::UnexpectedDisabled;
            raw_value = motor.rx.status_word;
        } else if (expected_running &&
                   motor.observed.observed_mode != motor.desired.mode) {
            reason = MyactMotorFaultReason::UnexpectedMode;
            raw_value = encode_mode_fault_value(
                motor.observed.observed_mode, motor.desired.mode);
        }

        if (reason != MyactMotorFaultReason::None) {
            latch_motor_fault(motor, reason, raw_value);
            break;
        }
    }

    update_communication_watchdog(process_data_ok, health,
                                  communication_protection_enabled);
    if (terminal_fault_latched()) {
        apply_whole_body_stop();
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
    const EthercatBusHealthSnapshot& health,
    bool communication_protection_enabled)
{
    const MyactCommunicationFaultReason current_reason =
        !health.master_link_up
            ? MyactCommunicationFaultReason::LinkDown
            : (health.wc_state != EC_WC_COMPLETE
                   ? MyactCommunicationFaultReason::WkcIncomplete
                   : MyactCommunicationFaultReason::None);
    current_communication_fault_.store(current_reason,
                                       std::memory_order_release);

    // 正式推理前不累计通信故障周期，避免启动期异常带入运行期阈值。
    // 计数仅由 RT 线程修改，重复开启开关不重置计数。
    if (!communication_protection_enabled || process_data_ok) {
        process_data_fail_count_ = 0;
        return;
    }

    if (process_data_fail_count_ < options_.comm_watchdog_fault_cycles) {
        ++process_data_fail_count_;
    }
    if (process_data_fail_count_ >= options_.comm_watchdog_fault_cycles &&
        !communication_fault_latched_.exchange(
            true, std::memory_order_acq_rel)) {
        latch_communication_fault(current_reason, health);
    }
}


MyactCommunicationFaultReason
MyActMotorController::current_communication_fault() const noexcept
{
    return current_communication_fault_.load(std::memory_order_acquire);
}


void MyActMotorController::latch_communication_fault(
    MyactCommunicationFaultReason reason,
    const EthercatBusHealthSnapshot& health)
{
    if (!latch_terminal_fault()) {
        return;
    }

    mb::RtEvent event;
    event.type = mb::RtEventType::COMM_WATCHDOG_FAULT;
    event.tick = discrete_command_tick();
    event.motor_index = -1;
    event.reason = static_cast<int>(reason);
    event.value = health.working_counter;
    push_event(event);
}


bool MyActMotorController::latch_motor_fault(
    MotorState& motor,
    MyactMotorFaultReason reason,
    uint32_t raw_value)
{
    if (!latch_terminal_fault()) {
        return false;
    }

    terminal_fault_motor_index_ = motor.motor_index;

    mb::RtEvent event;
    event.type = mb::RtEventType::MOTOR_FAULT_LATCHED;
    event.tick = discrete_command_tick();
    event.motor_index = motor.motor_index;
    event.reason = static_cast<int>(reason);
    event.value = raw_value;
    push_event(event);
    return true;
}


void MyActMotorController::apply_whole_body_stop()
{
    for (auto& motor : _motors) {
        const bool is_latched_fault_motor =
            motor.motor_index == terminal_fault_motor_index_;
        const bool has_current_drive_fault =
            motor.comm_ok &&
            (motor.observed.sw_faulted || motor.rx.error != 0);

        motor.step = (is_latched_fault_motor || has_current_drive_fault)
            ? MyactMotorStep::FAULT
            : MyactMotorStep::STOPPED;
        motor.mode_switch_step = MyactModeSwitchStep::IDLE;
        motor.desired.enabled = false;

        reset_motor_setpoints_to_feedback(motor);
        motor.tx.control_word = CMD_DISABLE_OPERATION;
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
    for (std::size_t i = 0; i < _motors.size(); ++i) {
        const int motor_id = _motors[i].motor_index;
        switch (cmd.setpoint_type) {
            case mb::SetpointCommandType::POSITION_TARGETS:
                if (!fits_i32(cmd.setpoints[i] *
                              pos_rad_to_raw_for_motor(motor_id))) {
                    return mb::CommandSubmitStatus::INVALID_PAYLOAD;
                }
                break;

            case mb::SetpointCommandType::VELOCITY_TARGETS:
                if (!fits_i32(cmd.setpoints[i] *
                              vel_rad_s_to_raw_for_motor(motor_id))) {
                    return mb::CommandSubmitStatus::INVALID_PAYLOAD;
                }
                break;

            case mb::SetpointCommandType::TORQUE_TARGETS:
                if (!fits_i16(cmd.setpoints[i])) {
                    return mb::CommandSubmitStatus::INVALID_PAYLOAD;
                }
                break;

            case mb::SetpointCommandType::IMPEDANCE_TARGETS: {
                const mb::ImpedanceSetpoint& setpoint =
                    cmd.impedance_setpoints[i];
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

    return mb::CommandSubmitStatus::ACCEPTED;
}

/// @brief 执行连续目标值命令
void MyActMotorController::apply_setpoint_command_impl(const mb::ControlCommand& cmd)
{
    const bool terminal_fault = terminal_fault_latched();
    if (terminal_fault) {
        return;
    }

    const MyactControlMode expected_mode = expected_mode_for_setpoint(cmd.setpoint_type);
    for (std::size_t i = 0; i < _motors.size(); ++i) {
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
            for (size_t i = 0; i < _motors.size(); i++) {
                _motors[i].desired.position_rad =
                    cmd.setpoints[i];
            }
            break;

        case mb::SetpointCommandType::VELOCITY_TARGETS:
            for (size_t i = 0; i < _motors.size(); i++) {
                _motors[i].desired.velocity_rad_s =
                    cmd.setpoints[i];
            }
            break;

        case mb::SetpointCommandType::TORQUE_TARGETS:
            for (size_t i = 0; i < _motors.size(); i++) {
                _motors[i].desired.torque =
                    cmd.setpoints[i];
            }
            break;

        case mb::SetpointCommandType::IMPEDANCE_TARGETS:
            for (size_t i = 0; i < _motors.size(); i++) {
                _motors[i].desired.impedance_setpoint =
                    cmd.impedance_setpoints[i];
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
    const mb::DiscreteCommand& cmd)
{
    mb::RtEvent event;
    event.type = mb::RtEventType::DISCRETE_QUEUE_FULL;
    event.tick = discrete_command_tick();
    event.motor_index = motor_index;
    event.command_type = cmd.type;
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
        feedback[i].host_timestamp_ns = motor.last_valid_host_timestamp_ns;
        feedback[i].position_rad      = motor.observed.position_rad;
        feedback[i].velocity_rad_s    = motor.observed.velocity_rad_s;
        feedback[i].torque_percent    = motor.observed.torque_percent;
        feedback[i].comm_ok           = motor.comm_ok;

        const bool is_latched_fault_motor =
            terminal_fault &&
            motor.motor_index == terminal_fault_motor_index_;
        const bool has_current_drive_fault =
            motor.comm_ok &&
            (motor.observed.sw_faulted || motor.rx.error != 0);

        if (motor.comm_ok) {
            feedback[i].enabled = motor.observed.operation_enabled;
            feedback[i].faulted =
                is_latched_fault_motor || has_current_drive_fault;
            feedback[i].mode = to_motor_control_mode(motor.observed.observed_mode);
        } else {
            feedback[i].enabled = false;
            feedback[i].faulted = is_latched_fault_motor;
            feedback[i].mode    = mb::MotorControlMode::NONE;
        }
        feedback[i].control_ready =
            control_ready_for_current_target(motor, terminal_fault);
        feedback[i].target_mode = to_motor_control_mode(motor.desired.mode);
    }

    publish_feedback(feedback);
}


void MyActMotorController::update_diagnostics_snapshot()
{
    if (_motors.empty()) {
        return;
    }

    MotorState* diagnostics_slot = status_monitor_.acquire_write_slot();
    for (size_t i = 0; i < _motors.size(); i++) {
        diagnostics_slot[i] = _motors[i];
    }

    status_monitor_.publish_written();
}


void MyActMotorController::event_sink_trampoline(void* context, const mb::RtEvent& event)
{
    static_cast<MyActMotorController*>(context)->push_event(event);
}


/* 配置电机监控打印: 空列表关闭打印，-1 表示全部电机 */
void MyActMotorController::set_print_info(const std::vector<int>& motor_indices)
{
    status_monitor_.set_print_info(motor_indices);
}

}
