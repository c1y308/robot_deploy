#pragma once

#include <vector>
#include <cstdint>
#include "myactua/MotorTypes.hpp"

namespace myactua {

enum class CommandType {
    SET_SETPOINTS,  // 设置目标值
    STOP,           // 停止电机
    RESTART,        // 启动电机
    SET_MODE        // 设置电机模式
};


struct ControlCommand {
    CommandType type;  // 控制命令类型
    int slave_index;   // 电机索引
    std::vector<double> values; // 目标值
    ControlMode mode;  // 电机模式（仅在 SET_MODE 命令中有效）

    ControlCommand() : type(CommandType::STOP), slave_index(-1), mode(ControlMode::NONE) {}

    ControlCommand(CommandType t, int idx = -1, 
                   const std::vector<double>& vals = {}, 
                   ControlMode m = ControlMode::NONE)
        : type(t), slave_index(idx), values(vals), mode(m) {}
};

/* 电机状态快照（发布给用户） */
struct MotorStatusSnapshot {
    int slave_index;
    double position;
    double velocity;
    double torque;
    uint16_t status_word;
    uint16_t error_code;
    ControlMode op_mode;
    ControlMode target_mode;

    MotorStatusSnapshot() 
        : slave_index(-1), position(0), velocity(0), torque(0),
          status_word(0), error_code(0), 
          op_mode(ControlMode::NONE), target_mode(ControlMode::NONE) {}
};

} // namespace myactua
