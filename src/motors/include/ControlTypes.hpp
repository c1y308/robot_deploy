#pragma once

#include <vector>
#include <cstdint>
#include "MotorTypes.hpp"

namespace myactua {

enum class CommandType {
    SET_SETPOINTS,  // 设置目标值
    STOP,           // 停止电机
    RESTART,        // 启动电机
    SET_MODE        // 设置电机模式
};


struct ControlCommand {
    CommandType type;            // 控制命令类型
    int slave_index;             // 电机索引
    std::vector<double> values;  // 目标值 (仅在 SET_SETPOINTS 命令中有效)
    ControlMode mode;            // 电机模式（仅在 SET_MODE 命令中有效）

    ControlCommand() : type(CommandType::STOP), slave_index(-1), mode(ControlMode::NONE) {}

    ControlCommand( CommandType t,
                    int idx = -1,
                    const std::vector<double>& vals = {},
                    ControlMode m = ControlMode::NONE)
                    : type(t), slave_index(idx), values(vals), mode(m) {}
};


/* 电机状态快照 */
struct MotorStatusSnapshot {
    int     slave_index;    // ID
    double  position;       // 位置
    double  velocity;       // 速度
    double  torque;         // 扭矩
    bool    comm_ok;        // 通信状态    
    uint16_t status_word;   // 状态字
    uint16_t error_code;    // 错误代码
    ControlMode op_mode;    // 当前模式
    ControlMode target_mode;// 目标模式

    MotorStatusSnapshot() 
        : slave_index(-1), position(0), velocity(0), torque(0), comm_ok(false), status_word(0), error_code(0), 
          op_mode(ControlMode::NONE), target_mode(ControlMode::NONE) {}
};

} // namespace myactua
