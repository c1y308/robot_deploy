#pragma once

#include <cstdint>

#include "EthercatTypes.hpp"
#include "driver/myact/myact_types.hpp"
#include "motor_base/ControlTypes.hpp"

namespace myactua {

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

struct ObservedState {
    bool fault;
    bool operation_enabled;
    uint16_t status_word;
    MyactControlMode mode;

    ObservedState()
        : fault(false),
          operation_enabled(false),
          status_word(0),
          mode(MyactControlMode::CSP)
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

    bool comm_ok;
    uint32_t comm_offline_total_count;

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
