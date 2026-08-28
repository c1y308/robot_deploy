#pragma once

#include <cstdint>

#include "protocol/ethercat/ethercat_types.hpp"
#include "driver/myact/myact_types.hpp"
#include "motor_base/command_types.hpp"

namespace myactua {


/// @brief 电机期望值与目标模式；mode 表示 target mode，不表示驱动器已确认运行模式。
struct DesiredState {
    bool enabled;

    MyactControlMode mode;

    double position_rad;
    double velocity_rad_s;
    double torque;
    motor_base::ImpedanceSetpoint impedance_setpoint;

    DesiredState()
        : enabled(false), mode(MyactControlMode::CSP), position_rad(0.0),
          velocity_rad_s(0.0), torque(0.0), impedance_setpoint()
    {
    }
};

/// @brief 电机观测的解码后状态（物理单位 + 0x6041 状态字位镜像 + 预转型模式；
/// 不含 0x603F 错误寄存器，fault 判定需调用点补充 rx.error）
struct ObservedState {
    double position_rad;      // rx.pos 转换后的弧度
    double velocity_rad_s;    // rx.vel 转换后的弧度/秒
    double torque_percent;    // rx.torque 转换后的百分比

    bool sw_faulted;
    bool operation_enabled;
    MyactControlMode observed_mode;

    ObservedState()
        : position_rad(0.0),
          velocity_rad_s(0.0),
          torque_percent(0.0),
          sw_faulted(false),
          operation_enabled(false),
          observed_mode(MyactControlMode::CSP)
    {
    }
};

struct MotorState {
    int motor_index;

    DesiredState  desired;
    ObservedState observed;

    MyactMotorStep step;
    MyactModeSwitchStep mode_switch_step;

    TxPDO tx;
    RxPDO rx;

    bool     comm_ok;
    uint32_t comm_offline_total_count;

    MotorState() : MotorState(-1) {}
    explicit MotorState(int index)
        : motor_index(index),
          desired(),
          observed(),
          step(MyactMotorStep::IDLE),
          mode_switch_step(MyactModeSwitchStep::IDLE),
          tx({}),
          rx({}),
          comm_ok(false),
          comm_offline_total_count(0)
    {
    }
};

} // namespace myactua
