#pragma once

#include <cstdint>

namespace motor_base {

/* 跨电机类型的公共控制模式语义。 */
enum class MotorControlMode : int8_t {
    NONE = 0,
    POSITION,
    VELOCITY,
    TORQUE,
    IMPEDANCE
};

} // namespace motor_base
