#pragma once

#include <cstdint>

namespace myactua {

/* MYACTUA/CiA402 PDO operation modes. These are driver-specific wire values. */
enum class MyactControlMode : int8_t {
    NONE = 0,
    PVT = 0x05,
    MIT = PVT,
    CSP = 0x08,
    CSV = 0x09,
    CST = 0x0A,
};

enum class MyactMotorStep {
    IDLE,
    ENABLING,
    RUNNING,
    STOPPED,
    FAULT,
    MODE_SWITCHING
};

enum class MyactModeSwitchStep {
    IDLE,
    SET_MODE,
    CLEAR,
    DISABLE,
    ENABLE,
    OPERATING,
    DONE
};

struct MyactDiagnosticsSnapshot {
    int motor_index;

    double position;
    double velocity;
    double torque;

    bool comm_ok;

    uint16_t status_word;
    uint16_t error_code;

    MyactControlMode op_mode;
    MyactControlMode target_mode;
    MyactControlMode tx_mode;

    MyactMotorStep step;
    MyactModeSwitchStep mode_switch_step;
    bool desired_enabled;

    uint32_t offline_count;

    int32_t command_position;
    int32_t command_velocity;
    int16_t command_torque;
    int32_t command_kp;
    int32_t command_kd;

    MyactDiagnosticsSnapshot()
        : motor_index(-1), position(0.0), velocity(0.0), torque(0.0),
          comm_ok(false), status_word(0), error_code(0),
          op_mode(MyactControlMode::NONE),
          target_mode(MyactControlMode::NONE),
          tx_mode(MyactControlMode::NONE),
          step(MyactMotorStep::IDLE),
          mode_switch_step(MyactModeSwitchStep::IDLE),
          desired_enabled(false), offline_count(0), command_position(0),
          command_velocity(0), command_torque(0), command_kp(0),
          command_kd(0) {}
};

} // namespace myactua
