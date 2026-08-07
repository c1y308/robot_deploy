#pragma once

#include <vector>

#include "driver/myact/myact_types.hpp"

namespace motor_base {
struct RtEvent;
}

namespace myactua {

void print_myact_status_table(
    const std::vector<MyactDiagnosticsSnapshot>& status,
    const std::vector<int>& motor_indices);

void print_myact_rt_event(const motor_base::RtEvent& event);

} // namespace myactua
