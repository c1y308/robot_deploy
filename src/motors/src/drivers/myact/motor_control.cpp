#include "motor_control.hpp"
#include "motor_units.hpp"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <time.h>
#include <pthread.h>
#include <sched.h>
#include <utility>

namespace myactua{

#define NSEC_PER_SEC (1000000000L)
#define CLOCK_TO_USE CLOCK_MONOTONIC

namespace {
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

}

/* 电机控制器构造函数 */
MYACTUA::MYACTUA(std::shared_ptr<EthercatAdapter> adapter, int num_motors)
    : MYACTUA(std::move(adapter), num_motors, Options())
{
}

MYACTUA::MYACTUA(std::shared_ptr<EthercatAdapter> adapter,
                 int num_motors,
                 Options options)
    : options_(options),
      _adapter(adapter),
      cmd_queue_(options_.command_queue_capacity),
      rt_event_dispatcher_(options_.rt_event_queue_capacity)
{
    if (num_motors < 0 ||
        static_cast<std::size_t>(num_motors) > kMaxMotorCommandSetpoints) {
        throw std::invalid_argument("MYACTUA num_motors exceeds fixed realtime capacity");
    }
    options_.max_commands_per_cycle =
        std::max<std::size_t>(1, options_.max_commands_per_cycle);
    options_.status_publish_period_ms =
        std::max(1, options_.status_publish_period_ms);

    /* 初始化电机状态列表 */
    for (int i = 0; i < num_motors; i++) {
        _motors.emplace_back(i);
    }
    comm_ok_rt_.resize(num_motors, 0);
    status_channel_.configure(_motors.size(), options_.status_publish_period_ms);
    status_monitor_.set_status_provider([this]() { return get_status(); });
    discrete_cmd_queues_.reserve(num_motors);
    for (int i = 0; i < num_motors; ++i) {
        discrete_cmd_queues_.emplace_back(options_.discrete_queue_capacity_per_motor);
    }
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
bool MYACTUA::connect(const char* ifname)
{
    if(_adapter->init(ifname))
        return true;
    return false;
}


/* 等待所有 EtherCAT 从站进入 OP */
bool MYACTUA::wait_all_slaves_ready(
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
            std::cout << "[MYACTUA] wait_all_slaves_ready interrupted after "
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
    std::cout << "[MYACTUA] wait_all_slaves_ready timeout after "
              << elapsed_ms << " ms" << std::endl;
    return false;
}


/* 电机实时控制线程 */
void MYACTUA::rt_thread_func()
{
    struct timespec next_period;
    clock_gettime(CLOCK_TO_USE, &next_period);
    const long period_ns = 1000000;

    while (running_) {
        // 1. 收取本周期 EtherCAT 输入
        _adapter->receive_physical();
        // 2.处理控制命令
        process_commands();
        // 3.执行控制周期
        update();
        // 4. 发送本周期 EtherCAT 输出
        _adapter->send_physical();
        // 5.发布 latest-only 状态快照
        update_status_snapshot_rt();
        // 6.等待下一个周期
        next_period.tv_nsec += period_ns;
        if (next_period.tv_nsec >= NSEC_PER_SEC) {
            next_period.tv_nsec -= NSEC_PER_SEC;
            next_period.tv_sec++;
        }
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &next_period, nullptr);
    }
}


CommandSubmitResult MYACTUA::validate_command(const ControlCommand& cmd) const
{
    if (!cmd.payload_valid) {
        return CommandSubmitResult::INVALID_PAYLOAD;
    }

    if (cmd.slave_index < ControlCommand::kAllSlaves ||
        cmd.slave_index >= static_cast<int>(_motors.size())) {
        return CommandSubmitResult::INVALID_COMMAND;
    }

    if (cmd.kind == ControlCommandKind::DISCRETE) {
        return CommandSubmitResult::ACCEPTED;
    }

    if (cmd.kind != ControlCommandKind::SETPOINT) {
        return CommandSubmitResult::INVALID_COMMAND;
    }

    const bool all_slaves = cmd.slave_index == ControlCommand::kAllSlaves;
    switch (cmd.setpoint_type) {
        case SetpointCommandType::SCALAR_SETPOINTS:
            if (cmd.scalar_setpoint_count == 0) {
                return CommandSubmitResult::INVALID_PAYLOAD;
            }
            if (all_slaves &&
                cmd.scalar_setpoint_count > _motors.size()) {
                return CommandSubmitResult::INVALID_PAYLOAD;
            }
            if (!all_slaves && cmd.scalar_setpoint_count != 1) {
                return CommandSubmitResult::INVALID_PAYLOAD;
            }
            return CommandSubmitResult::ACCEPTED;

        case SetpointCommandType::MIT_SETPOINTS:
            if (cmd.mit_setpoint_count == 0) {
                return CommandSubmitResult::INVALID_PAYLOAD;
            }
            if (all_slaves &&
                cmd.mit_setpoint_count > _motors.size()) {
                return CommandSubmitResult::INVALID_PAYLOAD;
            }
            if (!all_slaves && cmd.mit_setpoint_count != 1) {
                return CommandSubmitResult::INVALID_PAYLOAD;
            }
            return CommandSubmitResult::ACCEPTED;
    }

    return CommandSubmitResult::INVALID_COMMAND;
}


void MYACTUA::process_commands()
{
    ControlCommand cmd;
    std::size_t processed = 0;
    while (processed < options_.max_commands_per_cycle &&
           cmd_queue_.try_pop_rt(cmd)) {
        ++processed;
        if (cmd.kind == ControlCommandKind::DISCRETE) {
            enqueue_discrete_command(cmd);
            continue;
        }

        switch (cmd.setpoint_type) {
            /* 设置 DesiredState 中的标量目标值 */
            case SetpointCommandType::SCALAR_SETPOINTS:
                if (cmd.slave_index == ControlCommand::kAllSlaves) {
                    for (size_t i = 0; i < cmd.scalar_setpoint_count && i < _motors.size(); i++) {
                        _motors[i].desired.setpoint = cmd.scalar_setpoints[i];
                    }
                } else if (cmd.slave_index >= 0 &&
                           cmd.slave_index < static_cast<int>(_motors.size()) &&
                           cmd.scalar_setpoint_count == 1) {
                    _motors[cmd.slave_index].desired.setpoint = cmd.scalar_setpoints[0];
                }
                break;

            case SetpointCommandType::MIT_SETPOINTS:
                if (cmd.slave_index == ControlCommand::kAllSlaves) {
                    for (size_t i = 0; i < cmd.mit_setpoint_count && i < _motors.size(); i++) {
                        _motors[i].desired.mit_setpoint = cmd.mit_setpoints[i];
                    }
                } else if (cmd.slave_index >= 0 &&
                           cmd.slave_index < static_cast<int>(_motors.size()) &&
                           cmd.mit_setpoint_count == 1) {
                    _motors[cmd.slave_index].desired.mit_setpoint = cmd.mit_setpoints[0];
                }
                break;
        }
    }
}

/* 如果为离散命令需要确保完整执行； */
void MYACTUA::enqueue_discrete_command(const ControlCommand& cmd)
{
    /* 就地定义、使用，拉满封装性 */
    auto enqueue_one = [this, &cmd](int idx) {
        if (idx < 0 || idx >= static_cast<int>(_motors.size())) {
            return;
        }
        DiscreteCommand pending(cmd.discrete_type, cmd.mode);
        pending.phase = DiscretePhase::QUEUED;  // 初始状态为 QUEUED(入队列)

        pending.enqueue_tick      = discrete_cmd_tick_;
        pending.next_retry_tick   = discrete_cmd_tick_;  // 首次 apply 不等待
        pending.next_verify_tick  = discrete_cmd_tick_;  // 占位值 
        pending.deadline_tick     = discrete_cmd_tick_ + kDiscreteTimeoutTicks;

        pending.cur_retry = 0;
        pending.max_retries = kDiscreteMaxRetries;
        pending.stable_success_cycles = 0;
        pending.fail_reason = kDiscreteFailNone;

        if (!discrete_cmd_queues_[idx].push_back_rt(pending)) {
            push_discrete_queue_full_event_rt(idx, cmd);
        }
    };

    /* kAllSlaves 表示对所有电机执行命令 */
    if (cmd.slave_index == ControlCommand::kAllSlaves) {
        for (int i = 0; i < static_cast<int>(_motors.size()); ++i) {
            enqueue_one(i);
        }
    } else {
        enqueue_one(cmd.slave_index);
    }
}


void MYACTUA::update()
{
    ++discrete_cmd_tick_;  // 离散命令时间戳增加

    /* 接受电机回传数据，并记录当前周期通信状态 */
    for (size_t i = 0; i < _motors.size(); i++)
    {
        const bool comm_ok = _adapter->is_configured(_motors[i].slave_index);
        comm_ok_rt_[i] = comm_ok ? 1U : 0U;
        if (!comm_ok) {
            ++_motors[i].comm_offline_total_count;
        }
        _motors[i].comm_ok = comm_ok;
        
        /* 只接受通信正常的电机数据 */
        if (comm_ok) {
            _motors[i].rx = _adapter->receive(_motors[i].slave_index);
            refresh_observed_state(_motors[i]);
        }
    }

    /* 处理离散命令 */
    service_discrete_commands();

    /* 设置电机目标值。通信异常时保持当前控制状态，不覆写 step。 */
    for (size_t i = 0; i < _motors.size(); i++)
    {
        if (!comm_ok_rt_[i]) {
            continue;
        }
        process_single_motor(_motors[i], _motors[i].desired.setpoint);
    }

    /* 最终发送 */
    for (size_t i = 0; i < _motors.size(); i++)
    {
        if (!comm_ok_rt_[i]) continue;
        _adapter->send(_motors[i].slave_index, _motors[i].tx);
    }
}


void MYACTUA::refresh_observed_state(MotorState& motor)
{
    const uint16_t sw = motor.rx.status_word;
    motor.observed.status_word = sw;
    motor.observed.mode = (ControlMode)motor.rx.op_mode;
    motor.observed.operation_enabled = is_operation_enabled(sw);
    motor.observed.fault = is_fault(sw) || (motor.rx.error != 0);
}


void MYACTUA::service_discrete_commands()
{
    for (size_t i = 0; i < _motors.size(); ++i) {
        auto& queue = discrete_cmd_queues_[i];
        /* 无命令则跳过 */
        if (queue.empty()) {
            continue;
        }
        /* 获取命令和对应的电机 */
        auto& cmd   = queue.front_rt();
        auto& motor = _motors[i];

        /* 当前命令已完成 */
        if (cmd.phase == DiscretePhase::DONE) {
            queue.pop_front_rt();
            continue;
        }
        if (cmd.phase == DiscretePhase::FAILED) {
            push_discrete_failure_event_rt(
                static_cast<int>(i), cmd, cmd.fail_reason);
            queue.pop_front_rt();
            continue;
        }

        /* 超时 */
        if (discrete_cmd_tick_ > cmd.deadline_tick) {
            cmd.phase = DiscretePhase::FAILED;
            cmd.fail_reason = kDiscreteFailTimeout;
            continue;
        }

        /* 当前掉线，保持等待，不推进命令阶段 */
        if (!comm_ok_rt_[i]) {
            cmd.stable_success_cycles = 0;
            continue;
        }

        /* 驱动明确报错，命令直接失败 */
        if (motor.observed.fault) {
            cmd.phase = DiscretePhase::FAILED;
            cmd.fail_reason = kDiscreteFailFault;
            continue;
        }

        if (cmd.phase == DiscretePhase::QUEUED) {
            cmd.phase = DiscretePhase::APPLY_PENDING;
        }

        if (cmd.phase == DiscretePhase::APPLY_PENDING) {
            if (cmd.cur_retry >= cmd.max_retries) {
                cmd.phase = DiscretePhase::FAILED;
                cmd.fail_reason = kDiscreteFailMaxRetry;
                continue;
            }
            if (discrete_cmd_tick_ >= cmd.next_retry_tick) {
                apply_discrete_command_to_motor(static_cast<int>(i), cmd);
                cmd.cur_retry += 1;
                cmd.next_retry_tick  = discrete_cmd_tick_ + kDiscreteRetryTicks;
                cmd.next_verify_tick = discrete_cmd_tick_ + kDiscreteVerifyIntervalTicks;
                cmd.stable_success_cycles = 0;
                cmd.phase = DiscretePhase::VERIFYING;
            }
            continue;
        }

        if (cmd.phase == DiscretePhase::VERIFYING) {
            /* 没到验证时间 */
            if (discrete_cmd_tick_ < cmd.next_verify_tick) {
                continue;
            }

            /* 验证成功 */
            if (is_discrete_command_satisfied(motor, cmd)) {
                cmd.stable_success_cycles += 1;
                if (cmd.stable_success_cycles >= kDiscreteSuccessStableTicks) {
                    cmd.phase = DiscretePhase::DONE;
                    queue.pop_front_rt();
                } else {
                    cmd.next_verify_tick = discrete_cmd_tick_ + kDiscreteVerifyIntervalTicks;
                }
            }
            /* 验证失败 */
            else {
                cmd.stable_success_cycles = 0;
                /* 重发周期到了，可以进行重发 */
                if (discrete_cmd_tick_ >= cmd.next_retry_tick) {
                    cmd.phase = DiscretePhase::APPLY_PENDING;
                }
            }
        }
    }
}

/* 设置 DesiredState 结构体中的数据 */
void MYACTUA::apply_discrete_command_to_motor(int motor_index, const DiscreteCommand& cmd)
{
    if (motor_index < 0 || motor_index >= static_cast<int>(_motors.size())) {
        return;
    }

    MotorState& motor = _motors[motor_index];
    switch (cmd.type) {
        case DiscreteCommandType::STOP:
            // Edge-triggered: avoid resetting mode-switch state on retries.
            if (motor.desired.enabled) {
                motor.desired.enabled = false;
                motor.mode_switch_step = ModeSwitchStep::IDLE;
            }
            break;
        case DiscreteCommandType::RESTART:
            // Edge-triggered: first restart arms enable flow; later retries are no-op.
            if (!motor.desired.enabled) {
                motor.desired.enabled = true;
                motor.mode_switch_step = ModeSwitchStep::IDLE;
            }
            break;
        case DiscreteCommandType::SET_MODE:
            if (motor.desired.mode != cmd.mode) {
                motor.desired.mode  = cmd.mode;
                motor.mode_switch_step = ModeSwitchStep::IDLE;
            }
            break;
    }
}


bool MYACTUA::is_discrete_command_satisfied(const MotorState& motor, const DiscreteCommand& cmd) const
{
    const auto& observed = motor.observed;
    if (observed.fault) {
        return false;
    }

    switch (cmd.type) {
        case DiscreteCommandType::STOP:
            return !observed.operation_enabled;
        case DiscreteCommandType::RESTART:
            return  observed.operation_enabled;
        case DiscreteCommandType::SET_MODE:
            return observed.mode == cmd.mode;
    }
    return false;
}



void MYACTUA::process_single_motor(MotorState& motor, double setvalue)
{
    const auto& observed = motor.observed;
    const auto& desired  = motor.desired;
    const uint16_t sw = observed.status_word;

    if (observed.fault) {
        motor.step = MotorStep::FAULT;
        motor.mode_switch_step = ModeSwitchStep::IDLE;
        motor.tx.control_word = CMD_SHUTDOWN;
        motor.tx.op_mode = desired.mode;
        motor.tx.target_pos = motor.rx.pos;
        motor.tx.target_vel = 0;
        motor.tx.target_torque = 0;
        motor.tx.pvt_kp = 0;
        motor.tx.pvt_kd = 0;
        return;
    }

    /* 需要停止 */
    if (!desired.enabled) {
        motor.step = MotorStep::STOPPED;
        motor.mode_switch_step = ModeSwitchStep::IDLE;
        motor.tx.control_word = CMD_DISABLE_OPERATION;
        motor.tx.op_mode = desired.mode;
        motor.tx.target_vel = 0;
        motor.tx.target_pos = motor.rx.pos;
        motor.tx.target_torque = 0;
        motor.tx.pvt_kp = 0;
        motor.tx.pvt_kd = 0;
        return;
    }

    if (desired.mode == ControlMode::NONE) {
        motor.step = MotorStep::IDLE;
        motor.mode_switch_step = ModeSwitchStep::IDLE;
        motor.tx.control_word = CMD_SHUTDOWN;
        motor.tx.op_mode = ControlMode::NONE;
        motor.tx.target_pos = motor.rx.pos;
        motor.tx.target_vel = 0;
        motor.tx.target_torque = 0;
        motor.tx.pvt_kp = 0;
        motor.tx.pvt_kd = 0;
        return;
    }

    const bool mode_switch_active =
         (observed.mode != desired.mode) || (motor.mode_switch_step != ModeSwitchStep::IDLE);
    if (mode_switch_active) {
        motor.step = MotorStep::MODE_SWITCHING;
        handle_mode_switching(motor);
        // 模式切换采用闭环状态机，本周期不进入常规运行逻辑，避免覆盖切换命令。
        return;
    }

    /* 模式一致后进入使能/运行控制 */
    motor.tx.control_word = get_next_control_word(sw);
    motor.tx.op_mode = desired.mode;

    if (observed.operation_enabled) {
        motor.step = MotorStep::RUNNING;
        motor.tx.pvt_kp = 0;
        motor.tx.pvt_kd = 0;
        switch (desired.mode) {
            case ControlMode::CSV:  // rpm
                motor.tx.target_vel = static_cast<int32_t>(setvalue / kRawVelToRpm);
                break;
            case ControlMode::CSP:  // setvalue: deg, 1 rev = 131072 plus
                motor.tx.target_pos = static_cast<int32_t>((setvalue / kRadToDeg) / kRawPosToRad);
                break;
            case ControlMode::CST:  // 电流百分比
                motor.tx.target_torque = static_cast<int16_t>(setvalue);
                break;
            case ControlMode::PVT: {
                const MitSetpoint& mit = desired.mit_setpoint;
                motor.tx.target_pos = double_to_i32((mit.position_deg / kRadToDeg) / kRawPosToRad);
                motor.tx.target_vel = double_to_i32(mit.velocity_rad_s / kRawVelToRadPerSec);
                motor.tx.target_torque = double_to_i16(mit.torque_ff_permille);
                motor.tx.pvt_kp = double_to_i32(mit.kp * 1000.0);
                motor.tx.pvt_kd = double_to_i32(mit.kd * 1000.0);
                break;
            }
            default:
                break;
        }
    } else {
        motor.step = MotorStep::ENABLING;
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
    motor.tx.op_mode = desired.mode;
    motor.tx.target_pos = motor.rx.pos;
    motor.tx.target_vel = 0;
    motor.tx.target_torque = 0;
    motor.tx.pvt_kp = 0;
    motor.tx.pvt_kd = 0;

    switch (motor.mode_switch_step)
    {
        case ModeSwitchStep::IDLE:
            // 1) 写 0x6060(目标模式)
            motor.tx.control_word = CMD_SHUTDOWN;
            motor.mode_switch_step = ModeSwitchStep::SET_MODE;
            break;

        case ModeSwitchStep::SET_MODE:
            // 保持 0x6040=6，在失能态等待 0x6061 回读到目标模式，闭环推进。
            motor.tx.control_word = CMD_SHUTDOWN;
            if (mode_ok) {
                motor.mode_switch_step = ModeSwitchStep::CLEAR;
            }
            break;

        case ModeSwitchStep::CLEAR:
            // 2) 读 0x6064，并将 0x607A 对齐到当前位置。
            // 对齐命令至少发送一个周期后再进入下一步。
            motor.tx.control_word = CMD_SHUTDOWN;
            if (!mode_ok) {
                motor.mode_switch_step = ModeSwitchStep::SET_MODE;
            } else {
                motor.mode_switch_step = ModeSwitchStep::DISABLE;
            }
            break;

        case ModeSwitchStep::DISABLE:
            // 3-1) 写 0x6040=6，等待状态字进入 Ready to switch on。
            motor.tx.control_word = CMD_SHUTDOWN;
            if (!mode_ok) {
                motor.mode_switch_step = ModeSwitchStep::SET_MODE;
                break;
            }
            if (is_ready_to_switch_on(sw) && !is_switched_on(sw) && !is_operation_enabled(sw)) {
                motor.mode_switch_step = ModeSwitchStep::ENABLE;
            }
            break;

        case ModeSwitchStep::ENABLE:
            // 3-2) 写 0x6040=7，等待状态字进入 Switched on。
            motor.tx.control_word = CMD_SWITCH_ON;
            if (!mode_ok) {
                motor.mode_switch_step = ModeSwitchStep::SET_MODE;
                break;
            }
            if (is_switched_on(sw) && !is_operation_enabled(sw)) {
                motor.mode_switch_step = ModeSwitchStep::OPERATING;
            } else if (!is_ready_to_switch_on(sw)) {
                motor.mode_switch_step = ModeSwitchStep::DISABLE;
            }
            break;

        case ModeSwitchStep::OPERATING:
            // 3-3) 写 0x6040=15，等待状态字进入 Operation enabled。
            motor.tx.control_word = CMD_ENABLE_OPERATION;
            if (!mode_ok) {
                motor.mode_switch_step = ModeSwitchStep::SET_MODE;
                break;
            }
            if (is_operation_enabled(sw)) {
                motor.mode_switch_step = ModeSwitchStep::DONE;
            } else if (!is_switched_on(sw)) {
                motor.mode_switch_step = ModeSwitchStep::ENABLE;
            }
            break;

        case ModeSwitchStep::DONE:
            // 4) 闭环完成: 模式正确且已使能，切回常规运行控制。
            motor.tx.control_word = CMD_ENABLE_OPERATION;
            if (mode_ok && is_operation_enabled(sw)) {
                motor.mode_switch_step = ModeSwitchStep::IDLE;
                motor.step = MotorStep::RUNNING;
            } else {
                motor.mode_switch_step = mode_ok ? ModeSwitchStep::OPERATING : ModeSwitchStep::SET_MODE;
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

    MotorStatusChannel::WriteToken write_token;
    if (!status_channel_.try_begin_write_rt(write_token)) {
        push_status_overwritten_event_rt();
        return;
    }

    MotorStatusSnapshot* status_slot = write_token.data;
    for (size_t i = 0; i < _motors.size(); i++) {
        const auto& m = _motors[i];
        auto& s = status_slot[i];
        s.slave_index = m.slave_index;
        s.position = static_cast<double>(m.rx.pos);
        s.velocity = static_cast<double>(m.rx.vel);
        s.torque   = static_cast<double>(m.rx.torque);
        s.comm_ok  = m.comm_ok;
        s.status_word = m.rx.status_word;
        s.error_code  = m.rx.error;
        s.op_mode = (ControlMode)m.rx.op_mode;
        s.target_mode = m.desired.mode;
        s.tx_mode = (ControlMode)m.tx.op_mode;
        s.step = m.step;
        s.mode_switch_step = m.mode_switch_step;
        s.desired_enabled = m.desired.enabled;
        s.offline_count = m.comm_offline_total_count;
        s.tx_target_pos = m.tx.target_pos;
        s.tx_target_vel = m.tx.target_vel;
        s.tx_target_torque = m.tx.target_torque;
        s.tx_pvt_kp = m.tx.pvt_kp;
        s.tx_pvt_kd = m.tx.pvt_kd;
    }

    if (status_channel_.publish_rt(write_token)) {
        push_status_overwritten_event_rt();
    }
}


double MYACTUA::raw_pos_to_rad(double raw_pos)
{
    return myactua::raw_pos_to_rad(raw_pos);
}


double MYACTUA::rad_to_deg(double rad)
{
    return myactua::rad_to_deg(rad);
}


double MYACTUA::raw_vel_to_rad_s(double raw_vel)
{
    return myactua::raw_vel_to_rad_s(raw_vel);
}


CommandSubmitResult MYACTUA::send_command(const ControlCommand& cmd)
{
    const CommandSubmitResult validation = validate_command(cmd);
    if (validation != CommandSubmitResult::ACCEPTED) {
        return validation;
    }

    if (!cmd_queue_.try_push(cmd)) {
        return CommandSubmitResult::QUEUE_FULL;
    }
    return CommandSubmitResult::ACCEPTED;
}


std::vector<MotorStatusSnapshot> MYACTUA::get_status()
{
    return status_channel_.get_status();
}


std::vector<double> MYACTUA::get_joint_q_rad()
{
    const auto status = get_status();
    std::vector<double> q(status.size(), 0.0);
    for (std::size_t i = 0; i < status.size(); ++i) {
        q[i] = raw_pos_to_rad(status[i].position);
    }
    return q;
}


std::vector<double> MYACTUA::get_joint_vel_rad_s()
{
    const auto status = get_status();
    std::vector<double> dq(status.size(), 0.0);
    for (std::size_t i = 0; i < status.size(); ++i) {
        dq[i] = raw_vel_to_rad_s(status[i].velocity);
    }
    return dq;
}


std::vector<double> MYACTUA::get_joint_tau_raw()
{
    const auto status = get_status();
    std::vector<double> tau(status.size(), 0.0);
    for (std::size_t i = 0; i < status.size(); ++i) {
        tau[i] = status[i].torque;
    }
    return tau;
}


void MYACTUA::set_status_callback(StatusCallback cb)
{
    status_channel_.set_callback(std::move(cb));
}


void MYACTUA::set_rt_event_callback(RtEventCallback cb)
{
    rt_event_dispatcher_.set_callback(std::move(cb));
}


void MYACTUA::push_rt_event(const RtEvent& event)
{
    rt_event_dispatcher_.push_rt(event);
}


void MYACTUA::rt_event_sink_trampoline(void* context, const RtEvent& event)
{
    if (!context) {
        return;
    }
    static_cast<MYACTUA*>(context)->push_rt_event(event);
}


void MYACTUA::push_discrete_failure_event_rt(
    int motor_index,
    const DiscreteCommand& cmd,
    int reason)
{
    RtEvent event;
    event.type = RtEventType::DISCRETE_COMMAND_FAILED;
    event.tick = discrete_cmd_tick_;
    event.motor_index = motor_index;
    event.command_type = cmd.type;
    event.reason = reason;
    event.value = static_cast<uint32_t>(std::max(0, cmd.cur_retry));
    push_rt_event(event);
}


void MYACTUA::push_discrete_queue_full_event_rt(int motor_index, const ControlCommand& cmd)
{
    RtEvent event;
    event.type = RtEventType::DISCRETE_QUEUE_FULL;
    event.tick = discrete_cmd_tick_;
    event.motor_index = motor_index;
    event.command_type = cmd.discrete_type;
    event.reason = kDiscreteFailMaxRetry;
    push_rt_event(event);
}


void MYACTUA::push_status_overwritten_event_rt()
{
    const uint64_t count =
        status_overwrite_count_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (count != 1 && (count & (count - 1)) != 0) {
        return;
    }

    RtEvent event;
    event.type = RtEventType::STATUS_FRAME_OVERWRITTEN;
    event.tick = discrete_cmd_tick_;
    event.value = static_cast<uint32_t>(
        std::min<uint64_t>(count, static_cast<uint64_t>(UINT32_MAX)));
    push_rt_event(event);
}


/* 配置电机监控打印: 空列表关闭打印，-1 表示全部电机 */
void MYACTUA::set_print_info(const std::vector<int>& slave_indices)
{
    const bool enabled = status_monitor_.set_print_info(
        slave_indices,
        static_cast<int>(_motors.size()));
    if (running_) {
        enabled ? status_monitor_.start() : status_monitor_.stop();
    }
}


void MYACTUA::start()
{
    if (running_) return;

    status_channel_.start();
    rt_event_dispatcher_.start();

    running_ = true;
    rt_thread_ = std::thread(&MYACTUA::rt_thread_func, this);
    
    struct sched_param param;
    param.sched_priority = 80;
    const int sched_result =
        pthread_setschedparam(rt_thread_.native_handle(), SCHED_FIFO, &param);
    if (sched_result != 0) {
        std::cerr << "[MYACTUA] Warning: failed to set realtime scheduling "
                  << "(SCHED_FIFO priority 80): "
                  << std::strerror(sched_result) << std::endl;
    }

    if (status_monitor_.has_print_motor_ids()) {
        status_monitor_.start();
    }
    
    std::cout << "[MYACTUA] 实时控制线程已启动" << std::endl;
}


void MYACTUA::shutdown()
{
    const bool was_running = running_.exchange(false);
    status_monitor_.stop();

    if (was_running && rt_thread_.joinable()) {
        rt_thread_.join();
    }

    status_channel_.stop();
    rt_event_dispatcher_.stop();
    if (!was_running) return;
    
    std::cout << "[MYACTUA] 实时控制线程已停止" << std::endl;
}


}
