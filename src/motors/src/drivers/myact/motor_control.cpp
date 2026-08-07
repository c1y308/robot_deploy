#include "driver/myact/motor_control.hpp"
#include "driver/myact/myact_debug_printers.hpp"
#include "driver/myact/motor_units.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>

namespace myactua{

namespace mb = motor_base;

namespace {
constexpr double kRawTorqueToPercent = 0.1;

int32_t double_to_i32(double value)
{
    if (!std::isfinite(value)) {
        return 0;
    }
    const double lo = static_cast<double>(std::numeric_limits<int32_t>::min());
    const double hi = static_cast<double>(std::numeric_limits<int32_t>::max());
    return static_cast<int32_t>(std::llround(std::max(lo, std::min(hi, value))));
}

int16_t double_to_i16(double value)
{
    if (!std::isfinite(value)) {
        return 0;
    }
    const double lo = static_cast<double>(std::numeric_limits<int16_t>::min());
    const double hi = static_cast<double>(std::numeric_limits<int16_t>::max());
    return static_cast<int16_t>(std::llround(std::max(lo, std::min(hi, value))));
}

std::size_t checked_motor_count(int num_motors)
{
    if (num_motors < 0 ||
        static_cast<std::size_t>(num_motors) > mb::kMaxMotorCommandSetpoints) {
        throw std::invalid_argument("MYACTUA num_motors exceeds fixed realtime capacity");
    }
    return static_cast<std::size_t>(num_motors);
}

}

MyactControlMode MYACTUA::to_myact_mode(mb::MotorControlMode mode)
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

mb::MotorControlMode MYACTUA::to_motor_control_mode(MyactControlMode mode)
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
MYACTUA::MYACTUA(std::shared_ptr<EthercatAdapter> adapter, int num_motors)
    : MYACTUA(std::move(adapter), num_motors, Options())
{
}

MYACTUA::MYACTUA(std::shared_ptr<EthercatAdapter> adapter,
                 int num_motors,
                 Options options)
    : mb::MotorControllerBase(checked_motor_count(num_motors), options),
      options_(options),
      _adapter(std::move(adapter))
{
    options_.status_publish_period_ms =
        std::max(1, options_.status_publish_period_ms);

    /* 初始化电机状态列表 */
    for (int i = 0; i < num_motors; i++) {
        _motors.emplace_back(i);
    }
    comm_ok_rt_.resize(num_motors, 0);
    set_rt_event_fallback_printer(print_myact_rt_event);
    diagnostics_channel_.configure(_motors.size(), options_.status_publish_period_ms);
    status_monitor_.set_status_provider([this]() { return get_myact_diagnostics(); });
    status_monitor_.set_status_printer(print_myact_status_table);
    if (_adapter) {
        _adapter->set_rt_event_sink(this, &MYACTUA::rt_event_sink_trampoline);
    }
}

/* 析构函数 */
MYACTUA::~MYACTUA()
{
    shutdown();
    if (_adapter) {
        _adapter->set_rt_event_sink(nullptr, nullptr);
    }
}

/* 连接网卡函数 */
bool MYACTUA::connect_impl(const char* ifname)
{
    if (!_adapter) {
        return false;
    }
    return _adapter->init(ifname);
}


/* 等待所有 EtherCAT 从站进入 OP */
bool MYACTUA::wait_all_motors_ready(
    int timeout_ms,
    int poll_ms,
    const std::function<bool()>& should_stop) const
{
    if (!_adapter) {
        std::cerr << "[MYACTUA] EtherCAT adapter is null." << std::endl;
        return false;
    }

    using Clock = std::chrono::steady_clock;
    const int effective_timeout_ms = std::max(0, timeout_ms);
    const int effective_poll_ms = std::max(1, poll_ms);
    const auto start_time = Clock::now();
    const auto deadline = start_time + std::chrono::milliseconds(effective_timeout_ms);
    auto next_log_time = start_time;
    bool first_check = true;

    while (first_check || (effective_timeout_ms > 0 && Clock::now() < deadline)) {
        first_check = false;

        if (should_stop && should_stop()) {
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                Clock::now() - start_time).count();
            std::cout << "[MYACTUA] wait_all_motors_ready interrupted after "
                      << elapsed_ms << " ms" << std::endl;
            return false;
        }

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
            next_log_time = now + std::chrono::milliseconds(effective_poll_ms);
        }

        if (effective_timeout_ms == 0) {
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


/* 电机实时控制单周期 */
void MYACTUA::realtime_cycle_callback()
{
    _adapter->receive_physical();
    update();
    _adapter->send_physical();
    update_status_snapshot_rt();
    update_diagnostics_snapshot_rt();
}


mb::CommandSubmitResult MYACTUA::validate_command(
    const mb::ControlCommand& cmd) const
{
    if (cmd.kind == mb::ControlCommandKind::DISCRETE) {
        return mb::CommandSubmitResult::ACCEPTED;
    }

    MyactControlMode expected_mode = MyactControlMode::NONE;
    switch (cmd.setpoint_type) {
        case mb::SetpointCommandType::POSITION_TARGETS:
            expected_mode = MyactControlMode::CSP;
            break;

        case mb::SetpointCommandType::VELOCITY_TARGETS:
            expected_mode = MyactControlMode::CSV;
            break;

        case mb::SetpointCommandType::TORQUE_TARGETS:
            expected_mode = MyactControlMode::CST;
            break;

        case mb::SetpointCommandType::IMPEDANCE_TARGETS:
            expected_mode = MyactControlMode::PVT;
            break;
    }

    for (std::size_t i = 0; i < _motors.size(); ++i) {
        if (_motors[i].desired.mode != expected_mode) {
            return mb::CommandSubmitResult::INVALID_COMMAND;
        }
    }

    return mb::CommandSubmitResult::ACCEPTED;
}


void MYACTUA::apply_setpoint_command_callback(const mb::ControlCommand& cmd)
{
    switch (cmd.setpoint_type) {
        case mb::SetpointCommandType::POSITION_TARGETS:
            for (size_t i = 0; i < _motors.size(); i++) {
                _motors[i].desired.position_rad = cmd.setpoints[i];
            }
            break;

        case mb::SetpointCommandType::VELOCITY_TARGETS:
            for (size_t i = 0; i < _motors.size(); i++) {
                _motors[i].desired.velocity_rad_s = cmd.setpoints[i];
            }
            break;

        case mb::SetpointCommandType::TORQUE_TARGETS:
            for (size_t i = 0; i < _motors.size(); i++) {
                _motors[i].desired.torque = cmd.setpoints[i];
            }
            break;

        case mb::SetpointCommandType::IMPEDANCE_TARGETS:
            for (size_t i = 0; i < _motors.size(); i++) {
                _motors[i].desired.impedance_setpoint = cmd.impedance_setpoints[i];
            }
            break;
    }
}


void MYACTUA::update()
{
    /* 接受电机回传数据，并记录当前周期通信状态 */
    for (size_t i = 0; i < _motors.size(); i++)
    {
        const bool comm_ok = _adapter->is_configured(_motors[i].motor_index);
        comm_ok_rt_[i] = comm_ok ? 1U : 0U;
        if (!comm_ok) {
            ++_motors[i].comm_offline_total_count;
        }
        _motors[i].comm_ok = comm_ok;
        
        /* 只接受通信正常的电机数据 */
        if (comm_ok) {
            _motors[i].rx = _adapter->receive(_motors[i].motor_index);
            refresh_observed_state(_motors[i]);
        }
    }

    /* 设置电机目标值。通信异常时保持当前控制状态，不覆写 step。 */
    for (size_t i = 0; i < _motors.size(); i++)
    {
        if (!comm_ok_rt_[i]) {
            continue;
        }
        process_single_motor(_motors[i]);
    }

    /* 最终发送 */
    for (size_t i = 0; i < _motors.size(); i++)
    {
        if (!comm_ok_rt_[i]) continue;
        _adapter->send(_motors[i].motor_index, _motors[i].tx);
    }
}


void MYACTUA::refresh_observed_state(MotorState& motor)
{
    const uint16_t sw = motor.rx.status_word;
    motor.observed.status_word = sw;
    motor.observed.mode = static_cast<MyactControlMode>(motor.rx.op_mode);
    motor.observed.operation_enabled = is_operation_enabled(sw);
    motor.observed.fault = is_fault(sw) || (motor.rx.error != 0);
}


/* 设置 DesiredState 结构体中的数据 */
void MYACTUA::apply_discrete_command_callback(
    int motor_index,
    const mb::DiscreteCommand& cmd)
{
    if (motor_index < 0 || motor_index >= static_cast<int>(_motors.size())) {
        return;
    }

    MotorState& motor = _motors[motor_index];
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
                motor.desired.enabled = true;
                motor.mode_switch_step = MyactModeSwitchStep::IDLE;
            }
            break;
        case mb::DiscreteCommandType::SET_MODE:
            if (motor.desired.mode != to_myact_mode(cmd.mode)) {
                motor.desired.mode = to_myact_mode(cmd.mode);
                motor.mode_switch_step = MyactModeSwitchStep::IDLE;
            }
            break;
    }
}


mb::DiscreteCommandEvaluation MYACTUA::evaluate_discrete_command_callback(
    int motor_index,
    const mb::DiscreteCommand& cmd) const
{
    if (motor_index < 0 || motor_index >= static_cast<int>(_motors.size())) {
        return mb::DiscreteCommandEvaluation::PENDING;
    }

    if (motor_index >= static_cast<int>(comm_ok_rt_.size()) ||
        comm_ok_rt_[static_cast<std::size_t>(motor_index)] == 0) {
        return mb::DiscreteCommandEvaluation::PENDING;
    }

    const MotorState& motor = _motors[static_cast<std::size_t>(motor_index)];
    const auto& observed = motor.observed;
    if (observed.fault) {
        return mb::DiscreteCommandEvaluation::FAILED;
    }

    bool satisfied = false;
    switch (cmd.type) {
        case mb::DiscreteCommandType::STOP:
            satisfied = !observed.operation_enabled;
            break;
        case mb::DiscreteCommandType::RESTART:
            satisfied = observed.operation_enabled;
            break;
        case mb::DiscreteCommandType::SET_MODE:
            satisfied = observed.mode == to_myact_mode(cmd.mode);
            break;
    }
    return satisfied ? mb::DiscreteCommandEvaluation::SATISFIED
                     : mb::DiscreteCommandEvaluation::PENDING;
}



void MYACTUA::process_single_motor(MotorState& motor)
{
    const auto& observed = motor.observed;
    const auto& desired  = motor.desired;
    const uint16_t sw = observed.status_word;

    if (observed.fault) {
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

    const bool mode_switch_active =
         (observed.mode != desired.mode) || (motor.mode_switch_step != MyactModeSwitchStep::IDLE);
    if (mode_switch_active) {
        motor.step = MyactMotorStep::MODE_SWITCHING;
        handle_mode_switching(motor);
        // 模式切换采用闭环状态机，本周期不进入常规运行逻辑，避免覆盖切换命令。
        return;
    }

    /* 模式一致后进入使能/运行控制 */
    motor.tx.control_word = get_next_control_word(sw);
    motor.tx.op_mode = static_cast<int8_t>(desired.mode);

    if (observed.operation_enabled) {
        motor.step = MyactMotorStep::RUNNING;
        motor.tx.pvt_kp = 0;
        motor.tx.pvt_kd = 0;
        switch (desired.mode) {
            case MyactControlMode::CSV:
                motor.tx.target_vel = double_to_i32(
                    desired.velocity_rad_s / kRawVelToRadPerSec);
                break;
            case MyactControlMode::CSP:
                motor.tx.target_pos = double_to_i32(
                    desired.position_rad / kRawPosToRad);
                break;
            case MyactControlMode::CST:
                motor.tx.target_torque = double_to_i16(desired.torque);
                break;
            case MyactControlMode::PVT: {
                const mb::ImpedanceSetpoint& impedance = desired.impedance_setpoint;
                motor.tx.target_pos = double_to_i32(
                    impedance.position_rad / kRawPosToRad);
                motor.tx.target_vel = double_to_i32(
                    impedance.velocity_rad_s / kRawVelToRadPerSec);
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
void MYACTUA::handle_mode_switching(MotorState& motor)
{
    const auto& observed = motor.observed;
    const auto& desired  = motor.desired;
    const uint16_t sw = observed.status_word;
    const bool mode_ok = (observed.mode == desired.mode);

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


/* 依据当前状态获取下一个控制命令 */
ControlWordCommand MYACTUA::get_next_control_word(uint16_t status_word)
{
    if (!is_switched_on(status_word) && !is_ready_to_switch_on(status_word)) {
        return CMD_SHUTDOWN;
    }
    if (!is_switched_on(status_word) &&  is_ready_to_switch_on(status_word)) {
        return CMD_SWITCH_ON;
    }
    if (is_switched_on(status_word) && !is_operation_enabled(status_word)) {
        return CMD_ENABLE_OPERATION;
    }

    return CMD_ENABLE_OPERATION;
}



void MYACTUA::update_status_snapshot_rt()
{
    if (_motors.empty()) {
        return;
    }

    mb::MotorControllerBase::StatusWriteToken write_token;
    if (!try_begin_status_write_rt(write_token)) {
        push_status_overwritten_event_rt();
        return;
    }

    mb::MotorStatusSnapshot* status_slot = write_token.data;
    for (size_t i = 0; i < _motors.size(); i++) {
        const auto& m = _motors[i];
        auto& s = status_slot[i];
        s.motor_index = m.motor_index;
        s.position_rad = raw_pos_to_rad(static_cast<double>(m.rx.pos));
        s.velocity_rad_s = raw_vel_to_rad_s(static_cast<double>(m.rx.vel));
        s.torque_percent = static_cast<double>(m.rx.torque) * kRawTorqueToPercent;
        s.comm_ok = m.comm_ok;
        if (m.comm_ok) {
            const uint16_t sw = m.rx.status_word;
            s.enabled = is_operation_enabled(sw);
            s.faulted = is_fault(sw) || (m.rx.error != 0);
            s.mode = to_motor_control_mode(m.observed.mode);
        } else {
            s.enabled = false;
            s.faulted = false;
            s.mode = mb::MotorControlMode::NONE;
        }
        s.target_mode = to_motor_control_mode(m.desired.mode);
    }

    if (publish_status_rt(write_token)) {
        push_status_overwritten_event_rt();
    }
}


void MYACTUA::update_diagnostics_snapshot_rt()
{
    if (_motors.empty()) {
        return;
    }

    mb::LatestStatusChannel<MyactDiagnosticsSnapshot>::WriteToken write_token;
    if (!diagnostics_channel_.try_begin_write_rt(write_token)) {
        push_status_overwritten_event_rt();
        return;
    }

    MyactDiagnosticsSnapshot* diagnostics_slot = write_token.data;
    for (size_t i = 0; i < _motors.size(); i++) {
        const auto& m = _motors[i];
        auto& s = diagnostics_slot[i];
        s.motor_index = m.motor_index;
        s.position = static_cast<double>(m.rx.pos);
        s.velocity = static_cast<double>(m.rx.vel);
        s.torque = static_cast<double>(m.rx.torque);
        s.comm_ok = m.comm_ok;
        s.status_word = m.rx.status_word;
        s.error_code = m.rx.error;
        s.op_mode = static_cast<MyactControlMode>(m.rx.op_mode);
        s.target_mode = m.desired.mode;
        s.tx_mode = static_cast<MyactControlMode>(m.tx.op_mode);
        s.step = m.step;
        s.mode_switch_step = m.mode_switch_step;
        s.desired_enabled = m.desired.enabled;
        s.offline_count = m.comm_offline_total_count;
        s.command_position = m.tx.target_pos;
        s.command_velocity = m.tx.target_vel;
        s.command_torque = m.tx.target_torque;
        s.command_kp = m.tx.pvt_kp;
        s.command_kd = m.tx.pvt_kd;
    }

    if (diagnostics_channel_.publish_rt(write_token)) {
        push_status_overwritten_event_rt();
    }
}


double MYACTUA::raw_pos_to_rad(double raw_pos)
{
    return myactua::raw_pos_to_rad(raw_pos);
}


double MYACTUA::raw_vel_to_rad_s(double raw_vel)
{
    return myactua::raw_vel_to_rad_s(raw_vel);
}


std::vector<MyactDiagnosticsSnapshot> MYACTUA::get_myact_diagnostics()
{
    return diagnostics_channel_.get_status();
}


void MYACTUA::set_myact_diagnostics_callback(MyactDiagnosticsCallback cb)
{
    diagnostics_channel_.set_callback(std::move(cb));
}


void MYACTUA::rt_event_sink_trampoline(void* context, const mb::RtEvent& event)
{
    if (!context) {
        return;
    }
    static_cast<MYACTUA*>(context)->push_rt_event(event);
}


void MYACTUA::discrete_command_failed_callback(
    int motor_index,
    const mb::DiscreteCommand& cmd,
    mb::DiscreteFailReason reason)
{
    mb::RtEvent event;
    event.type = mb::RtEventType::DISCRETE_COMMAND_FAILED;
    event.tick = discrete_command_tick_rt();
    event.motor_index = motor_index;
    event.command_type = cmd.type;
    event.reason = static_cast<int>(reason);
    event.value = static_cast<uint32_t>(std::max(0, cmd.cur_retry));
    push_rt_event(event);
}


void MYACTUA::discrete_queue_full_callback(
    int motor_index,
    const mb::ControlCommand& cmd)
{
    mb::RtEvent event;
    event.type = mb::RtEventType::DISCRETE_QUEUE_FULL;
    event.tick = discrete_command_tick_rt();
    event.motor_index = motor_index;
    event.command_type = cmd.discrete_type;
    event.reason = static_cast<int>(mb::DiscreteFailReason::MAX_RETRY);
    push_rt_event(event);
}


void MYACTUA::push_status_overwritten_event_rt()
{
    const uint64_t count =
        status_overwrite_count_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (count != 1 && (count & (count - 1)) != 0) {
        return;
    }

    mb::RtEvent event;
    event.type = mb::RtEventType::STATUS_FRAME_OVERWRITTEN;
    event.tick = discrete_command_tick_rt();
    event.value = static_cast<uint32_t>(
        std::min<uint64_t>(count, static_cast<uint64_t>(UINT32_MAX)));
    push_rt_event(event);
}


/* 配置电机监控打印: 空列表关闭打印，-1 表示全部电机 */
void MYACTUA::set_print_info(const std::vector<int>& motor_indices)
{
    const bool enabled = status_monitor_.set_print_info(
        motor_indices,
        static_cast<int>(_motors.size()));
    if (is_running()) {
        enabled ? status_monitor_.start() : status_monitor_.stop();
    }
}


bool MYACTUA::realtime_start_callback()
{
    diagnostics_channel_.start();
    if (status_monitor_.has_print_motor_ids()) {
        status_monitor_.start();
    }
    
    std::cout << "[MYACTUA] 实时控制线程已启动" << std::endl;
    return true;
}


void MYACTUA::realtime_stop_callback() noexcept
{
    try {
        status_monitor_.stop();
        diagnostics_channel_.stop();
        std::cout << "[MYACTUA] 实时控制线程已停止" << std::endl;
    } catch (...) {
    }
}


}
