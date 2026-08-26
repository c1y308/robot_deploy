#pragma once

#include "motor_base/command_types.hpp"

#include <array>
#include <cstdint>
#include <cstddef>

namespace motor_base {

struct RealtimeMotorFeedback {
    std::uint64_t sequence{0};
    std::int64_t timestamp_ns{0};
    std::size_t motor_count{0};

    std::array<double, kMaxMotorCommandSetpoints> q{};
    std::array<double, kMaxMotorCommandSetpoints> dq{};
    std::array<double, kMaxMotorCommandSetpoints> torque_percent{};
};

}  // namespace motor_base
