#pragma once

#include <cstddef>

#ifndef ROBOT_POLICY_ENABLE_GAIT_PHASE_OBS
#define ROBOT_POLICY_ENABLE_GAIT_PHASE_OBS 0
#endif

namespace inference::policy_observation {

inline constexpr bool kEnableGaitPhase =
    ROBOT_POLICY_ENABLE_GAIT_PHASE_OBS != 0;

inline constexpr std::size_t kDof = 12;
inline constexpr std::size_t kFrameStack = 15;

inline constexpr std::size_t kBaseAngVelSize = 3;
inline constexpr std::size_t kProjectedGravitySize = 3;
inline constexpr std::size_t kVelocityCommandsSize = 3;
inline constexpr std::size_t kGaitPhaseSize = 2;
inline constexpr std::size_t kJointPosRelSize = kDof;
inline constexpr std::size_t kJointVelRelSize = kDof;
inline constexpr std::size_t kLastActionSize = kDof;

inline constexpr std::size_t kSingleObservationSize =
    kBaseAngVelSize +
    kProjectedGravitySize +
    kVelocityCommandsSize +
    (kEnableGaitPhase ? kGaitPhaseSize : 0) +
    kJointPosRelSize +
    kJointVelRelSize +
    kLastActionSize;

inline constexpr std::size_t kObservationSize =
    kSingleObservationSize * kFrameStack;

static_assert(kObservationSize == (kEnableGaitPhase ? 705 : 675),
              "policy observation size must match the selected model version");

}  // namespace inference::policy_observation
