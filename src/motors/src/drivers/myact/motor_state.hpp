#pragma once

#include <cstdint>

#include "ControlTypes.hpp"
#include "MotorTypes.hpp"

namespace myactua {

struct DesiredState {
    bool enabled;
    ControlMode mode;
    double setpoint;
    MitSetpoint mit_setpoint;

    DesiredState()
        : enabled(false), mode(ControlMode::CSP), setpoint(0.0),
          mit_setpoint()
    {
    }
};

struct ObservedState {
    bool fault;
    bool operation_enabled;
    uint16_t status_word;
    ControlMode mode;

    ObservedState()
        : fault(false),
          operation_enabled(false),
          status_word(0),
          mode(ControlMode::CSP)
    {
    }
};

struct MotorState {
    int slave_index;

    DesiredState  desired;
    ObservedState observed;

    MotorStep step;
    ModeSwitchStep mode_switch_step;

    TxPDO tx;
    RxPDO rx;

    bool comm_ok;
    uint32_t comm_offline_total_count;

    explicit MotorState(int index)
        : slave_index(index),
          desired(),
          observed(),
          step(MotorStep::IDLE),
          mode_switch_step(ModeSwitchStep::IDLE),
          tx({}),
          rx({}),
          comm_ok(false),
          comm_offline_total_count(0)
    {
    }
};

} // namespace myactua
