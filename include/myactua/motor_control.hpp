#pragma once
#include <vector>
#include <memory>
#include <cstdio>
#include "myactua/MotorTypes.hpp"
#include "myactua/EthercatAdapter.hpp"

#define LIMIT(VAL, MIN, MAX) ((VAL)<(MIN)?(MIN):((VAL)>(MAX)?(MAX):(VAL)))

namespace myactua{


/* 电机运行状态 */
enum class MotorStep {
    IDLE,           // 待机
    ENABLING,       // 使能中，会等待电机进入 Operation Enabled 状态后自动进入RUNNING状态
    RUNNING,        // 已进入 Operation Enabled，可接收运动指令
    STOPPED,        // 停止状态(保持使能但不运行)
    FAULT,          // 故障
    MODE_SWITCHING  // 模式切换中
};


/* 电机控制器类 */
class MYACTUA {
public:
    struct MotorState{
        int slave_index;  // 电机ID序号
        ControlMode target_mode;
        MotorStep step;
        ModeSwitchStep mode_switch_step;  // 模式切换子状态
        TxPDO tx;
        RxPDO rx;

        MotorState(int index)
        : slave_index(index), target_mode(ControlMode::NONE), step(MotorStep::IDLE), 
          mode_switch_step(ModeSwitchStep::IDLE), tx({}), rx({}) {}
    };

    /** @param adapter 已经初始化好的 EthercatAdapterIGH 指针
      * @param num_motors 从站数量 **/
    MYACTUA(std::shared_ptr<EthercatAdapter> adapter,int num_motors);

    /** 周期更新函数 @param setvalues 目标值数组，长度与从站数量相同 **/
    void update(const std::vector<double>& setvalues);

    void set_mode(ControlMode mode,int slave_index);

    /* 停止电机(保持使能状态但不运行) */
    void stop(int slave_index = -1);

    /* 重启电机(从停止状态恢复运行) */
    void restart(int slave_index = -1);

    /* 连接与初始化网络 */
    bool connect(const char* ifname);

private:
    std::shared_ptr<EthercatAdapter> _adapter;
    std::vector<MotorState> _motors;

    void process_single_motor(MotorState& motor, double setvalue);

    void handle_mode_switching(MotorState& motor);

    ControlWordCommand get_next_control_word(uint16_t status_word);

    void print_motors_info(void);
};

} // namespace myactua