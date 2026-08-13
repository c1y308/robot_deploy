#pragma once

#include "policy/policy_observation_config.hpp"
#include "robot/robot_config.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace inference {

class TorchPolicyRunner;

using PolicyAction = std::array<float, policy_observation::kDof>;

struct PolicyObservationTerms {
    std::array<float, policy_observation::kBaseAngVelSize> base_ang_vel{};
    std::array<float, policy_observation::kProjectedGravitySize> projected_gravity{};
    std::array<float, policy_observation::kVelocityCommandsSize> velocity_commands{};
    std::array<float, policy_observation::kJointPosRelSize> joint_pos_rel{};
    std::array<float, policy_observation::kJointVelRelSize> joint_vel_rel{};
    PolicyAction last_action{};
};

struct PolicyRuntimeStepResult {
    PolicyAction raw_action{};
    std::int64_t inference_start_ns{0};
    std::int64_t inference_end_ns{0};
};

class PolicyRuntime {
public:
    static constexpr std::size_t kDof = policy_observation::kDof;
    static constexpr std::size_t kFrameStack = policy_observation::kFrameStack;
    static constexpr std::size_t kObservationSize = policy_observation::kObservationSize;

    PolicyRuntime();
    ~PolicyRuntime();

    PolicyRuntime(const PolicyRuntime&) = delete;
    PolicyRuntime& operator=(const PolicyRuntime&) = delete;

    bool load(const PolicyConfig& config);
    void shutdown();
    void reset();

    bool is_loaded() const;
    // terms.last_action is the previous policy output a_{t-1}; infer() stores
    // the newly produced a_t only after successful inference for the next frame.
    bool infer(const PolicyObservationTerms& terms, PolicyRuntimeStepResult& result);

    void advance_frame() noexcept;
    void advance_episode() noexcept;

    std::uint64_t frame_index() const noexcept { return frame_index_; }
    std::uint64_t episode_length() const noexcept { return episode_length_; }
    const PolicyAction& last_action() const noexcept { return last_action_raw_; }
    const std::string& last_error() const noexcept { return last_error_; }

private:
#ifdef ROBOT_POLICY_RUNTIME_TESTING
public:
#endif
    using ObservationArray = std::array<float, kObservationSize>;

    void build_observation(const PolicyObservationTerms& terms,
                           ObservationArray& observation);
#ifdef ROBOT_POLICY_RUNTIME_TESTING
private:
#endif
    void set_error(std::string message);

    PolicyConfig policy_config_;
    std::unique_ptr<TorchPolicyRunner> runner_;
    PolicyAction last_action_raw_{};
    ObservationArray observation_history_{};
    bool observation_history_ready_{false};
    std::uint64_t episode_length_{0};
    std::uint64_t frame_index_{0};
    std::string last_error_;
};

}  // namespace inference
