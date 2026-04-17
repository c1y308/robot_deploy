#include "motor_control.hpp"
#include <algorithm>
#include <iostream>
#include <time.h>
#include <pthread.h>
#include <sched.h>

namespace myactua{

#define NSEC_PER_SEC (1000000000L)
#define CLOCK_TO_USE CLOCK_MONOTONIC

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kPosPlusPerRev = 131072.0;
constexpr double kRawPosToRad = (2.0 * kPi) / kPosPlusPerRev;
constexpr double kRawVelToRpm = 60.0 / kPosPlusPerRev;
constexpr double kRpmToRadPerSec = (2.0 * kPi) / 60.0;
constexpr double kRadToDeg = 180.0 / kPi;


constexpr uint64_t kDiscreteRetryCycles = 100;       // 重试间隔：100 ms @ 1kHz
constexpr uint64_t kDiscreteVerifyIntervalCycles = 100; // 验证间隔：100 ms @ 1kHz
constexpr int kDiscreteSuccessStableCycles = 1;     // 连续 1 个周期满足判据视为成功
constexpr int kDiscreteMaxRetries = 100;            // 最多重试 100 次
constexpr uint64_t kDiscreteTimeoutCycles = 5000;   // 命令超时：5 s @ 1kHz

enum DiscreteFailReason {
    kDiscreteFailNone = 0,
    kDiscreteFailFault = 1,
    kDiscreteFailTimeout = 2,
    kDiscreteFailMaxRetry = 3,
};
}

static struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;
    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    } else {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }
    return result;
}


MYACTUA::MYACTUA(std::shared_ptr<EthercatAdapter> adapter, int num_motors) : _adapter(adapter)
{
    for (int i = 0; i < num_motors; i++) {
        _motors.emplace_back(i);
        print_motor_ids_.push_back(i);
    }
    status_snapshot_.resize(num_motors);
    discrete_cmd_queues_.resize(num_motors);
}


MYACTUA::~MYACTUA()
{
    shutdown();
}


bool MYACTUA::connect(const char* ifname)
{
    if(_adapter->init(ifname))
        return true;
    return false;
}


void MYACTUA::rt_thread_func()
{
    struct timespec next_period;
    clock_gettime(CLOCK_TO_USE, &next_period);
    const long period_ns = 1000000;

    while (running_) {
        // 1.处理控制命令
        process_commands();
        // 2.执行控制周期
        update({});
        // 3.更新电机状态快照
        update_status_snapshot();
        // 4.调用回调函数
        if (status_callback_) {
            status_callback_(status_snapshot_);
        }
        // 5.等待下一个周期
        next_period.tv_nsec += period_ns;
        if (next_period.tv_nsec >= NSEC_PER_SEC) {
            next_period.tv_nsec -= NSEC_PER_SEC;
            next_period.tv_sec++;
        }
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &next_period, nullptr);
    }
}



void MYACTUA::update(const std::vector<double> &setvalues)
{
    static int print_count = 0;
    ++control_cycle_;
    std::vector<bool> comm_ok(_motors.size(), false);

    /* 接受电机回传数据，并记录当前周期通信状态 */
    for (size_t i = 0; i < _motors.size(); i++)
    {
        comm_ok[i] = _adapter->isConfigured(_motors[i].slave_index);
        if (!comm_ok[i]) {
            ++_motors[i].comm_offline_total_count;
        }
        _motors[i].comm_ok = comm_ok[i];
        
        /* 只接受通信正常的电机数据 */
        if (comm_ok[i]) {
            _motors[i].rx = _adapter->receive(_motors[i].slave_index);
            refresh_observed_state(_motors[i]);
        }
    }

    service_discrete_commands(comm_ok);

    /* 设置电机目标值。通信异常时保持当前控制状态，不覆写 step。 */
    for (size_t i = 0; i < _motors.size(); i++)
    {
        if (!comm_ok[i]) {
            continue;
        }
        double val = (i < setvalues.size()) ? setvalues[i] : _motors[i].desired.setpoint;
        process_single_motor(_motors[i], val);
    }

    for (size_t i = 0; i < _motors.size(); i++)
    {
        if (!comm_ok[i]) continue;
        _adapter->send(_motors[i].slave_index, _motors[i].tx);
    }

    if (++print_count >= 500) {
        print_count = 0;
        if (!print_motor_ids_.empty()) {
            print_motors_info();
        }
    }
}


void MYACTUA::refresh_observed_state(MotorState& motor)
{
    const uint16_t sw = motor.rx.status_word;
    motor.observed.status_word = sw;
    motor.observed.error_code = motor.rx.error;
    motor.observed.mode = (ControlMode)motor.rx.op_mode;
    motor.observed.ready_to_switch_on = is_ready_to_switch_on(sw);
    motor.observed.switched_on = is_switched_on(sw);
    motor.observed.operation_enabled = is_operation_enabled(sw);
    motor.observed.fault = is_fault(sw) || (motor.rx.error != 0);
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
        return;
    }

    const bool mode_switch_active =
        (motor.mode_switch_step != ModeSwitchStep::IDLE) || (observed.mode != desired.mode);
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
        motor.tx.max_torque = 5000;
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
            default:
                break;
        }
    } else {
        motor.step = MotorStep::ENABLING;
        motor.tx.target_pos = motor.rx.pos;
        motor.tx.target_vel = 0;
        motor.tx.target_torque = 0;
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
    motor.tx.max_torque = 5000;

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


void MYACTUA::set_mode(ControlMode mode, int slave_index)
{
    if(slave_index >= 0 && slave_index < _motors.size())
    {
        _motors[slave_index].desired.mode = mode;
        _motors[slave_index].mode_switch_step = ModeSwitchStep::IDLE;
    }
}


void MYACTUA::stop_motor(int slave_index)
{
    if(slave_index < 0) {
        for(auto& motor : _motors) {
            motor.desired.enabled = false;
            motor.mode_switch_step = ModeSwitchStep::IDLE;
        }
    } else if(slave_index < _motors.size()) {
        _motors[slave_index].desired.enabled = false;
        _motors[slave_index].mode_switch_step = ModeSwitchStep::IDLE;
    }
}


void MYACTUA::restart(int slave_index)
{
    if(slave_index < 0) {
        for(auto& motor : _motors) {
            motor.desired.enabled = true;
            motor.mode_switch_step = ModeSwitchStep::IDLE;
        }
    } else if(slave_index < _motors.size()) {
        _motors[slave_index].desired.enabled = true;
        _motors[slave_index].mode_switch_step = ModeSwitchStep::IDLE;
    }
}


void MYACTUA::start()
{
    if (running_) return;
    
    running_ = true;
    rt_thread_ = std::thread(&MYACTUA::rt_thread_func, this);
    
    struct sched_param param;
    param.sched_priority = 80;
    pthread_setschedparam(rt_thread_.native_handle(), SCHED_FIFO, &param);
    
    std::cout << "[MYACTUA] 实时控制线程已启动" << std::endl;
}


void MYACTUA::shutdown()
{
    if (!running_) return;
    
    running_ = false;
    if (rt_thread_.joinable()) {
        rt_thread_.join();
    }
    
    std::cout << "[MYACTUA] 实时控制线程已停止" << std::endl;
}



void MYACTUA::process_commands()
{
    ControlCommand cmd;
    while (cmd_queue_.pop(cmd, 0)) {
        switch (cmd.type) {
            case CommandType::SET_SETPOINTS:
                for (size_t i = 0; i < cmd.values.size() && i < _motors.size(); i++) {
                    _motors[i].desired.setpoint = cmd.values[i];
                }
                break;
                
            case CommandType::STOP:
                enqueue_discrete_command(cmd);
                break;
                
            case CommandType::RESTART:
                enqueue_discrete_command(cmd);
                break;
                
            case CommandType::SET_MODE:
                enqueue_discrete_command(cmd);
                break;
        }
    }
}


void MYACTUA::enqueue_discrete_command(const ControlCommand& cmd)
{
    /* 就地定义、使用，拉满封装性 */
    auto enqueue_one = [this, &cmd](int idx) {
        if (idx < 0 || idx >= static_cast<int>(_motors.size())) {
            return;
        }
        DiscreteCommand pending(cmd.type, cmd.mode);
        pending.phase = DiscretePhase::QUEUED;

        pending.enqueue_cycle = control_cycle_;
        pending.next_retry_cycle = control_cycle_;
        pending.next_verify_cycle = control_cycle_;
        pending.deadline_cycle = control_cycle_ + kDiscreteTimeoutCycles;
        pending.max_retries = kDiscreteMaxRetries;
        pending.retry_count = 0;
        pending.stable_success_cycles = 0;
        pending.fail_reason = kDiscreteFailNone;
        discrete_cmd_queues_[idx].push_back(pending);
    };


    /* 小于 0 表示对所有电机执行命令 */
    if (cmd.slave_index < 0) {
        for (int i = 0; i < static_cast<int>(_motors.size()); ++i) {
            enqueue_one(i);
        }
    } else {
        enqueue_one(cmd.slave_index);
    }
}


void MYACTUA::service_discrete_commands(const std::vector<bool>& comm_ok)
{
    for (size_t i = 0; i < _motors.size(); ++i) {
        auto& queue = discrete_cmd_queues_[i];
        /* 无命令则跳过 */
        if (queue.empty()) {
            continue;
        }

        auto& cmd = queue.front();
        auto& motor = _motors[i];

        /* 当前命令已完成 */
        if (cmd.phase == DiscretePhase::DONE) {
            queue.pop_front();
            continue;
        }
        if (cmd.phase == DiscretePhase::FAILED) {
            std::cerr << "[MYACTUA] discrete command failed on motor " << i
                      << ", type=" << static_cast<int>(cmd.type)
                      << ", reason=" << cmd.fail_reason
                      << ", retry=" << cmd.retry_count << "\n";
            queue.pop_front();
            continue;
        }

        if (control_cycle_ > cmd.deadline_cycle) {
            cmd.phase = DiscretePhase::FAILED;
            cmd.fail_reason = kDiscreteFailTimeout;
            continue;
        }

        /* 当前掉线，保持等待，不推进命令阶段 */
        if (!comm_ok[i]) {
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
            if (cmd.retry_count >= cmd.max_retries) {
                cmd.phase = DiscretePhase::FAILED;
                cmd.fail_reason = kDiscreteFailMaxRetry;
                continue;
            }
            if (control_cycle_ >= cmd.next_retry_cycle) {
                apply_discrete_command_to_motor(static_cast<int>(i), cmd);
                cmd.retry_count += 1;
                cmd.next_retry_cycle = control_cycle_ + kDiscreteRetryCycles;
                cmd.next_verify_cycle = control_cycle_ + kDiscreteVerifyIntervalCycles;
                cmd.stable_success_cycles = 0;
                cmd.phase = DiscretePhase::VERIFYING;
            }
            continue;
        }

        if (cmd.phase == DiscretePhase::VERIFYING) {
            if (control_cycle_ < cmd.next_verify_cycle) {
                continue;
            }
            /* 验证成功 */
            if (is_discrete_command_satisfied(motor, cmd)) {
                cmd.stable_success_cycles += 1;
                if (cmd.stable_success_cycles >= kDiscreteSuccessStableCycles) {
                    cmd.phase = DiscretePhase::DONE;
                    queue.pop_front();
                } else {
                    cmd.next_verify_cycle = control_cycle_ + kDiscreteVerifyIntervalCycles;
                }
            }
            /* 验证失败 */
            else {
                cmd.stable_success_cycles = 0;
                if (control_cycle_ >= cmd.next_retry_cycle) {
                    cmd.phase = DiscretePhase::APPLY_PENDING;
                }
            }
        }
    }
}


void MYACTUA::apply_discrete_command_to_motor(int motor_index, const DiscreteCommand& cmd)
{
    if (motor_index < 0 || motor_index >= static_cast<int>(_motors.size())) {
        return;
    }

    MotorState& motor = _motors[motor_index];
    switch (cmd.type) {
        case CommandType::STOP:
            // Edge-triggered: avoid resetting mode-switch state on retries.
            if (motor.desired.enabled) {
                motor.desired.enabled = false;
                motor.mode_switch_step = ModeSwitchStep::IDLE;
            }
            break;
        case CommandType::RESTART:
            // Edge-triggered: first restart arms enable flow; later retries are no-op.
            if (!motor.desired.enabled) {
                motor.desired.enabled = true;
                motor.mode_switch_step = ModeSwitchStep::IDLE;
            }
            break;
        case CommandType::SET_MODE:
            if (motor.desired.mode != cmd.mode) {
                motor.desired.mode  = cmd.mode;
                motor.mode_switch_step = ModeSwitchStep::IDLE;
            }
            break;
        case CommandType::SET_SETPOINTS:
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
        case CommandType::STOP:
            return !observed.operation_enabled;
        case CommandType::RESTART:
            return  observed.operation_enabled;
        case CommandType::SET_MODE:
            return observed.mode == cmd.mode;
        case CommandType::SET_SETPOINTS:
            return true;
    }
    return false;
}


void MYACTUA::update_status_snapshot()
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    for (size_t i = 0; i < _motors.size(); i++) {
        const auto& m = _motors[i];
        status_snapshot_[i].slave_index = m.slave_index;
        status_snapshot_[i].position = static_cast<double>(m.rx.pos);
        status_snapshot_[i].velocity = static_cast<double>(m.rx.vel);
        status_snapshot_[i].torque = static_cast<double>(m.rx.torque);
        status_snapshot_[i].comm_ok = m.comm_ok;
        status_snapshot_[i].status_word = m.rx.status_word;
        status_snapshot_[i].error_code = m.rx.error;
        status_snapshot_[i].op_mode = (ControlMode)m.rx.op_mode;
        status_snapshot_[i].target_mode = m.desired.mode;
    }
}


double MYACTUA::raw_pos_to_rad(double raw_pos)
{
    return raw_pos * kRawPosToRad;
}


double MYACTUA::rad_to_deg(double rad)
{
    return rad * kRadToDeg;
}


double MYACTUA::raw_vel_to_rad_s(double raw_vel)
{
    return raw_vel * kRawVelToRpm * kRpmToRadPerSec;
}


void MYACTUA::send_command(const ControlCommand& cmd)
{
    cmd_queue_.push(cmd);
}


std::vector<MotorStatusSnapshot> MYACTUA::get_status()
{
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_snapshot_;
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
    status_callback_ = std::move(cb);
}


/* 默认为不打印电机信息 */
void MYACTUA::set_print_info(const std::vector<int>& slave_indices)
{
    for (auto& motor : _motors) {
        motor.print_info = false;
    }
    print_motor_ids_.clear();

    for (int slave_index : slave_indices) {
        if (slave_index < 0 || slave_index >= static_cast<int>(_motors.size())) {
            continue;
        }
        /* 去重 */
        if (std::find(print_motor_ids_.begin(), print_motor_ids_.end(), slave_index) ==
            print_motor_ids_.end()) {
            print_motor_ids_.push_back(slave_index);
            _motors[slave_index].print_info = true;
        }
    }
}


void MYACTUA::print_motors_info(void){
    printf("\033[2J\033[H");

    printf("\033[1;36m============================ MOTOR REAL-TIME MONITOR ============================\033[0m\n");
    printf("%-6s | %-10s | %-16s | %-22s | %-8s | %-8s | %-7s | %-8s | %-6s | %-6s | %-10s\n",
        "ID", "OFFLINE_CNT", "STEP", "MODE_SWITCH_STEP", "RX_MODE", "TX_MODE", "DES_EN", "TX_CW",
        "OP_EN", "SW_ON", "RDY_SW_ON");
    printf("---------------------------------------------------------------------------------------------------------------------------------\n");

    for (const auto& m : _motors) {
        if (std::find(print_motor_ids_.begin(), print_motor_ids_.end(), m.slave_index) ==
            print_motor_ids_.end()) {
            continue;
        }
        const char* color_code = "\033[32m";
        if (m.step == MotorStep::FAULT) color_code = "\033[31m";
        if (m.step == MotorStep::STOPPED) color_code = "\033[35m";
        if (m.step == MotorStep::MODE_SWITCHING) color_code = "\033[33m";
        char cw_hex[9] = {};
        std::snprintf(cw_hex, sizeof(cw_hex), "0x%04X", static_cast<unsigned>(m.tx.control_word));

        const char* step_name = "UNKNOWN";
        switch (m.step) {
            case MotorStep::IDLE: step_name = "IDLE"; break;
            case MotorStep::ENABLING: step_name = "ENABLING"; break;
            case MotorStep::RUNNING: step_name = "RUNNING"; break;
            case MotorStep::STOPPED: step_name = "STOPPED"; break;
            case MotorStep::FAULT: step_name = "FAULT"; break;
            case MotorStep::MODE_SWITCHING: step_name = "MODE_SWITCHING"; break;
        }

        const char* mode_switch_step_name = "N/A";
        switch (m.mode_switch_step) {
            case ModeSwitchStep::IDLE: mode_switch_step_name = "IDLE"; break;
            case ModeSwitchStep::SET_MODE: mode_switch_step_name = "SET_MODE"; break;
            case ModeSwitchStep::CLEAR: mode_switch_step_name = "CLEAR"; break;
            case ModeSwitchStep::DISABLE: mode_switch_step_name = "DISABLE"; break;
            case ModeSwitchStep::ENABLE: mode_switch_step_name = "ENABLE"; break;
            case ModeSwitchStep::OPERATING: mode_switch_step_name = "OPERATING"; break;
            case ModeSwitchStep::DONE: mode_switch_step_name = "DONE"; break;
        }

        const char* mode_name = "UNKNOWN";
        switch (m.rx.op_mode) {
            case ControlMode::NONE: mode_name = "NONE"; break;
            case ControlMode::CSP: mode_name = "CSP"; break;
            case ControlMode::CSV: mode_name = "CSV"; break;
            case ControlMode::CST: mode_name = "CST"; break;
            default: break;
        }

        const char* tx_mode_name = "UNKNOWN";
        switch (m.tx.op_mode) {
            case ControlMode::NONE: tx_mode_name = "NONE"; break;
            case ControlMode::CSP: tx_mode_name = "CSP"; break;
            case ControlMode::CSV: tx_mode_name = "CSV"; break;
            case ControlMode::CST: tx_mode_name = "CST"; break;
            default: break;
        }

        printf("M %-4d | %-10u | %s%-16s\033[0m | %s%-22s\033[0m | %-8s | %-8s | %-7s | %-8s | %-6s | %-6s | %-10s\n",
            m.slave_index,
            static_cast<unsigned int>(m.comm_offline_total_count),
            color_code,
            step_name,
            color_code,
            mode_switch_step_name,
            mode_name,
            tx_mode_name,
            m.desired.enabled ? "Y" : "N",
            cw_hex,
            m.observed.operation_enabled ? "Y" : "N",
            m.observed.switched_on ? "Y" : "N",
            m.observed.ready_to_switch_on ? "Y" : "N");
    }
    printf("\033[1;36m=================================================================================\033[0m\n");
    
    fflush(stdout); 
}

}
