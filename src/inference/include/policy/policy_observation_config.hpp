#pragma once

#include <array>
#include <cstddef>

namespace inference::policy_observation {

inline constexpr std::size_t kDof = 12;
inline constexpr std::size_t kFrameStack = 15;

inline constexpr std::size_t kBaseAngVelSize = 3;
inline constexpr std::size_t kProjectedGravitySize = 3;
inline constexpr std::size_t kVelocityCommandsSize = 3;
inline constexpr std::size_t kGaitPhaseSize = 2;
inline constexpr std::size_t kJointPosRelSize = kDof;
inline constexpr std::size_t kJointVelRelSize = kDof;
inline constexpr std::size_t kLastActionSize = kDof;

inline constexpr std::size_t kSingleObservationSizeWithoutGaitPhase =
    kBaseAngVelSize +
    kProjectedGravitySize +
    kVelocityCommandsSize +
    kJointPosRelSize +
    kJointVelRelSize +
    kLastActionSize;

inline constexpr std::size_t kSingleObservationSizeWithGaitPhase =
    kSingleObservationSizeWithoutGaitPhase + kGaitPhaseSize;

inline constexpr std::size_t kObservationSizeWithoutGaitPhase =
    kSingleObservationSizeWithoutGaitPhase * kFrameStack;
inline constexpr std::size_t kObservationSizeWithGaitPhase =
    kSingleObservationSizeWithGaitPhase * kFrameStack;

inline constexpr std::size_t kMaxObservationSize =
    kObservationSizeWithGaitPhase;

inline constexpr std::size_t observation_size(bool enable_gait_phase) noexcept
{
    return enable_gait_phase ? kObservationSizeWithGaitPhase
                             : kObservationSizeWithoutGaitPhase;
}

static_assert(kObservationSizeWithoutGaitPhase == 675 &&
                  kObservationSizeWithGaitPhase == 705,
              "policy observation size must match the selected model version");

}  // namespace inference::policy_observation

namespace inference {

/* 固定容量缓冲；实际有效长度由 PolicyRuntimeConfig::GaitPhase::enabled 决定。 */
using PolicyObservation =
    std::array<float, policy_observation::kMaxObservationSize>;
using PolicyAction = std::array<float, policy_observation::kDof>;

}  // namespace inference
