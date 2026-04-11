#include "motor_control.hpp"
#include <iostream>
#include <time.h>
#include <pthread.h>
#include <sched.h>

namespace myactua{

#define NSEC_PER_SEC (1000000000L)
#define CLOCK_TO_USE CLOCK_MONOTONIC

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
    }
    status_snapshot_.resize(num_motors);
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


void MYACTUA::update(const std::vector<double> &setvalues)
{
    static int print_count = 0;
    /* 接受电机回传数据 */
    for (size_t i = 0; i < _motors.size(); i++)
    {
        if (_adapter->isConfigured(_motors[i].slave_index)) {
            _motors[i].rx = _adapter->receive(_motors[i].slave_index);
        } else {
            _motors[i].rx = {};
        }
    }
    /* 设置电机目标值 */
    for (size_t i = 0; i < _motors.size(); i++)
    {
        if (!_adapter->isConfigured(_motors[i].slave_index)) {
            _motors[i].step = MotorStep::IDLE;
            continue;
        }
        double val = (i < setvalues.size()) ? setvalues[i] : _motors[i].setpoint;
        process_single_motor(_motors[i], val);
    }

    for (size_t i = 0; i < _motors.size(); i++)
    {
        if (!_adapter->isConfigured(_motors[i].slave_index)) continue;
        _adapter->send(_motors[i].slave_index, _motors[i].tx);
    }

    if (++print_count >= 500) {
        print_count = 0;
        print_motors_info();
    }
}


void MYACTUA::process_single_motor(MotorState& motor, double setvalue)
{
    uint16_t sw = motor.rx.status_word;

    if(motor.step == MotorStep::STOPPED) {
        motor.tx.control_word = CMD_DISABLE_OPERATION;
        motor.tx.target_vel = 0;
        motor.tx.target_pos = motor.rx.pos;
        motor.tx.target_torque = 0;
        return;
    }

    if (is_fault(sw)) 
    {
        motor.step = MotorStep::FAULT;
        motor.tx.control_word = CMD_SHUTDOWN;
        return;
    }
    if (motor.rx.op_mode != motor.target_mode && motor.step == MotorStep::RUNNING)
    {
        motor.step = MotorStep::MODE_SWITCHING;
    }
    if(motor.step == MotorStep::MODE_SWITCHING){
        handle_mode_switching(motor);
        return;
    }
    
    motor.tx.control_word = get_next_control_word(sw);
    if (is_operation_enabled(sw))  
    {   
        motor.step = MotorStep::RUNNING;
        motor.tx.max_torque = 5000;
        switch (motor.target_mode) 
        {
            case ControlMode::CSV:
                motor.tx.target_vel = static_cast<int32_t>((setvalue * 131072.0) / 60.0);
                break;
            case ControlMode::CSP:
                motor.tx.target_pos = static_cast<int32_t>(setvalue * (65535.0 / 180.0));
                break;
            case ControlMode::CST:
                motor.tx.target_torque = static_cast<int16_t>(setvalue);
                break;
            default:
                break;
        }
    } 
    else 
    {
        motor.step = MotorStep::ENABLING;
        motor.tx.target_pos = motor.rx.pos;
        motor.tx.target_vel = 0;
        motor.tx.target_torque = 0;
    }
}


/* 状态机切换电机控制模式 */
void MYACTUA::handle_mode_switching(MotorState& motor)
{
    uint16_t sw = motor.rx.status_word;

    switch (motor.mode_switch_step)
    {
        case ModeSwitchStep::IDLE:
            motor.mode_switch_step = ModeSwitchStep::SET_MODE_CLEAR_DISABLE;
            break;

        case ModeSwitchStep::SET_MODE_CLEAR_DISABLE:
            motor.tx.op_mode = motor.target_mode;
            motor.tx.target_pos = motor.rx.pos;
            motor.tx.target_vel = 0;
            motor.tx.target_torque = 0;
            motor.tx.control_word = CMD_SHUTDOWN;
            motor.mode_switch_step = ModeSwitchStep::ENABLE;
            break;

        case ModeSwitchStep::ENABLE:
            if (!is_switched_on(sw) && !is_operation_enabled(sw)) {
                if (is_ready_to_switch_on(sw)) {
                    motor.tx.control_word = CMD_SWITCH_ON;
                    motor.mode_switch_step = ModeSwitchStep::OPERATING;
                } else {
                    motor.tx.control_word = CMD_SHUTDOWN;
                }
            } else
                motor.tx.control_word = CMD_SHUTDOWN;
            break;

        case ModeSwitchStep::OPERATING:
            if(is_switched_on(sw)){
                if (is_operation_enabled(sw)) {
                    motor.mode_switch_step = ModeSwitchStep::DONE;
                } else {
                    motor.tx.control_word = CMD_ENABLE_OPERATION;
                }
            }else
                motor.tx.control_word = CMD_SWITCH_ON;
            break;

        case ModeSwitchStep::DONE:
            if (motor.rx.op_mode == motor.target_mode) {
                motor.mode_switch_step = ModeSwitchStep::IDLE;
                motor.step = MotorStep::RUNNING;
            } else {
                motor.mode_switch_step = ModeSwitchStep::SET_MODE_CLEAR_DISABLE;
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
        _motors[slave_index].target_mode = mode;
    }
}


void MYACTUA::stop_motor(int slave_index)
{
    if(slave_index < 0) {
        for(auto& motor : _motors) {
            motor.step = MotorStep::STOPPED;
            motor.mode_switch_step = ModeSwitchStep::IDLE;
        }
    } else if(slave_index < _motors.size()) {
        _motors[slave_index].step = MotorStep::STOPPED;
        _motors[slave_index].mode_switch_step = ModeSwitchStep::IDLE;
    }
}


void MYACTUA::restart(int slave_index)
{
    if(slave_index < 0) {
        for(auto& motor : _motors) {
            motor.step = MotorStep::ENABLING;
            motor.mode_switch_step = ModeSwitchStep::IDLE;
        }
    } else if(slave_index < _motors.size()) {
        _motors[slave_index].step = MotorStep::ENABLING;
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


void MYACTUA::process_commands()
{
    ControlCommand cmd;
    while (cmd_queue_.pop(cmd, 0)) {
        switch (cmd.type) {
            case CommandType::SET_SETPOINTS:
                for (size_t i = 0; i < cmd.values.size() && i < _motors.size(); i++) {
                    _motors[i].setpoint = cmd.values[i];
                }
                break;
                
            case CommandType::STOP:
                stop_motor(cmd.slave_index);
                break;
                
            case CommandType::RESTART:
                restart(cmd.slave_index);
                break;
                
            case CommandType::SET_MODE:
                set_mode(cmd.mode, cmd.slave_index);
                break;
        }
    }
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
        status_snapshot_[i].status_word = m.rx.status_word;
        status_snapshot_[i].error_code = m.rx.error;
        status_snapshot_[i].op_mode = m.rx.op_mode;
        status_snapshot_[i].target_mode = m.target_mode;
    }
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


void MYACTUA::set_status_callback(StatusCallback cb)
{
    status_callback_ = std::move(cb);
}


void MYACTUA::print_motors_info(void){
    printf("\033[2J\033[H");

    printf("\033[1;36m============================ MOTOR REAL-TIME MONITOR ============================\033[0m\n");
    printf("%-6s | %-16s | %-22s | %-10s | %-10s | %-10s | %-12s | %-6s\n",
        "ID", "STEP", "MODE_SWITCH_STEP", "POS", "VEL", "TORQUE", "TARGET(TX)", "MODE");
    printf("---------------------------------------------------------------------------------------------------------------------------------\n");

    for (const auto& m : _motors) {
        const char* color_code = "\033[32m";
        if (m.step == MotorStep::FAULT) color_code = "\033[31m";
        if (m.step == MotorStep::STOPPED) color_code = "\033[35m";
        if (m.step == MotorStep::MODE_SWITCHING) color_code = "\033[33m";

        int32_t current_target = 0;
        if (m.rx.op_mode == 8)      current_target = m.tx.target_pos;
        else if (m.rx.op_mode == 9) current_target = m.tx.target_vel;
        else if (m.rx.op_mode == 10) current_target = (int32_t)m.tx.target_torque;

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
            case ModeSwitchStep::SET_MODE_CLEAR_DISABLE: mode_switch_step_name = "SET_MODE_CLEAR_DISABLE"; break;
            case ModeSwitchStep::ENABLE: mode_switch_step_name = "ENABLE"; break;
            case ModeSwitchStep::OPERATING: mode_switch_step_name = "OPERATING"; break;
            case ModeSwitchStep::DONE: mode_switch_step_name = "DONE"; break;
        }

        printf("M %-4d | %s%-16s\033[0m | %s%-22s\033[0m | %-10d | %-10d | %-10d | %-12d | %-6d\n",
            m.slave_index,
            color_code,
            step_name,
            color_code,
            mode_switch_step_name,
            m.rx.pos,
            m.rx.vel,
            m.rx.torque,
            current_target,
            static_cast<int>(m.rx.op_mode));
    }
    printf("\033[1;36m=================================================================================\033[0m\n");
    
    fflush(stdout); 
}

}
