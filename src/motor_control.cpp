#include "myactua/motor_control.hpp"

namespace myactua{

/* 定义构造函数 */
MYACTUA::MYACTUA(std::shared_ptr<EthercatAdapter> adapter, int num_motors) : _adapter(adapter)
{
    for (int i = 0; i < num_motors; i++) {
        _motors.emplace_back(i); // 从站电机索引从0开始
    }
}


/* 定义连接与初始化网络函数 */
bool MYACTUA::connect(const char* ifname)
{
    if(_adapter->init(ifname))
        return true;
    return false;
}


/* 一帧的处理与发送函数 */
void MYACTUA::update(const std::vector<double> &setvalues)
{
    static int print_count = 0; // 静态计数器

    for (size_t i = 0; i < _motors.size(); i++)
    {
        _motors[i].rx = _adapter->receive(_motors[i].slave_index);

        double val = (i < setvalues.size()) ? setvalues[i] : 0.0;
        process_single_motor(_motors[i], val);

        _adapter->send(_motors[i].slave_index, _motors[i].tx);
    }

    // 打印
    // 每 100 个周期（约 100ms）执行一次界面刷新
    if (++print_count >= 100) {
        print_count = 0; // 重置计数器
        print_motors_info();
    }
}

/* 设置控制字，目标速度或目标位置或目标力矩 */
void MYACTUA::process_single_motor(MotorState& motor, double setvalue)
{
    uint16_t sw = motor.rx.status_word;

    /* 运行模式不一致或正在切换中 */
    if (motor.rx.op_mode != motor.target_mode || motor.step == MotorStep::MODE_SWITCHING) 
    {   
        motor.step = MotorStep::MODE_SWITCHING;
        handle_mode_switching(motor);
        return;
    }

    if (is_fault(sw)) 
    {
        motor.step = MotorStep::FAULT;
        motor.tx.control_word = CMD_SHUTDOWN;
        return;
    }

    motor.tx.control_word = get_next_control_word(sw);

    /* 伺服正在运行，设置新的目标值 */
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
            motor.tx.control_word = CMD_SHUTDOWN;  // 失能并停止运行
            motor.mode_switch_step = ModeSwitchStep::ENABLE;
            break;

        case ModeSwitchStep::ENABLE:
            if (!is_switched_on(sw) && !is_operation_enabled(sw)) {
                if (is_ready_to_switch_on(sw)) {
                    motor.tx.control_word = CMD_SWITCH_ON;  // 使能，但不运行
                    motor.mode_switch_step = ModeSwitchStep::OPERATING;
                } else {
                    motor.tx.control_word = CMD_SHUTDOWN;  // 还未准备好，继续等待
                }
            } else
                motor.tx.control_word = CMD_SHUTDOWN;  // 控制字=6，继续等待失能
            break;

        case ModeSwitchStep::OPERATING:
            if(is_switched_on(sw)){  // 使能成功
                if (is_operation_enabled(sw)) {
                    motor.mode_switch_step = ModeSwitchStep::DONE;
                } else {
                    motor.tx.control_word = CMD_ENABLE_OPERATION;  // 控制字=15，等待使能完成
                }
            }else
                motor.tx.control_word = CMD_SWITCH_ON;  // 还未使能，继续使能
            break;

            case ModeSwitchStep::DONE:
                if (motor.rx.op_mode == motor.target_mode) {
                    motor.mode_switch_step = ModeSwitchStep::IDLE;
                    motor.step = MotorStep::RUNNING;
                }
                break;
    }
}


ControlWordCommand MYACTUA::get_next_control_word(uint16_t status_word)
{
    /* 当前未使能，电机不可以使能 */
    if (!is_switched_on(status_word) && !is_ready_to_switch_on(status_word)) {
        return CMD_SHUTDOWN;
    }
    /* 当前未使能，电机可以使能 */
    if (!is_switched_on(status_word) &&  is_ready_to_switch_on(status_word)) {
        return CMD_SWITCH_ON;  // 使能但不运行
    }
    /* 电机已使能，当前并未运行 */
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


void MYACTUA::print_motors_info(void){
    // 保存光标并回到左上角
        printf("\033[s\033[H");

        // 绘制表头
        printf("\n\033[1;36m============================ MOTOR REAL-TIME MONITOR ============================\033[0m\n");
        // ID(6) | STEP(12) | POS(10) | VEL(10) | TORQUE(10) | TARGET(12) | MODE(6)
        printf("%-6s | %-12s | %-10s | %-10s | %-10s | %-12s | %-6s\n", 
            "ID", "STEP", "POS", "VEL", "TORQUE", "TARGET(TX)", "MODE");
        printf("---------------------------------------------------------------------------------\n");

        for (const auto& m : _motors) {
            // 1. 确定颜色字符串 (char*)
            const char* color_code = "\033[32m"; // 绿色
            if (m.step == MotorStep::FAULT) color_code = "\033[31m"; // 红色
            if (m.step == MotorStep::MODE_SWITCHING) color_code = "\033[33m"; // 黄色

            // 2. 根据模式确定当前要观察的发送值
            int32_t current_target = 0;
            if (m.rx.op_mode == 8)      current_target = m.tx.target_pos;
            else if (m.rx.op_mode == 9) current_target = m.tx.target_vel;
            else if (m.rx.op_mode == 10) current_target = (int32_t)m.tx.target_torque;

            printf("M %-4d | %s%-12d\033[0m | %-10d | %-10d | %-10d | %-12d | %-6d\n", 
                m.slave_index,     // 对应 M %-4d
                color_code,        // 对应 %s
                m.step,            // 对应 %-12d
                m.rx.pos,          // 对应 %-10d
                m.rx.vel,          // 对应 %-10d
                m.rx.torque,       // 对应 %-10d
                current_target,    // 对应 %-12d
                m.rx.op_mode); // 对应 %-6d
        }
        printf("\033[1;36m=================================================================================\033[0m\n");
        
        // 恢复光标到输入行
        printf("\033[u");
        fflush(stdout); 
}

}