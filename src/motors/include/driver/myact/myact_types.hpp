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

enum class MyactCommunicationFaultReason : int {
    None = 0,
    LinkDown = 1,
    WkcIncomplete = 2,
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

} // namespace myactua
