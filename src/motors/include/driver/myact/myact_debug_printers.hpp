#pragma once

#include <vector>

#include "driver/myact/motor_state.hpp"

namespace motor_base {
struct RtEvent;
}

namespace myactua {

void print_myact_status_table(
    const std::vector<MotorState>& status,
    const std::vector<int>& motor_indices);

void print_myact_event(const motor_base::RtEvent& event);

} // namespace myactua
