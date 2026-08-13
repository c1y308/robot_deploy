#include "policy/policy_runtime.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

constexpr std::size_t kLastActionOffset =
    inference::policy_observation::kFrameStack *
    (inference::policy_observation::kBaseAngVelSize +
     inference::policy_observation::kProjectedGravitySize +
     inference::policy_observation::kVelocityCommandsSize +
     (inference::policy_observation::kEnableGaitPhase
          ? inference::policy_observation::kGaitPhaseSize
          : 0) +
     inference::policy_observation::kJointPosRelSize +
     inference::policy_observation::kJointVelRelSize);

void expect(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

void expect_near(float actual, float expected, const std::string& message)
{
    if (std::abs(actual - expected) > 1e-6F) {
        std::cerr << "FAIL: " << message
                  << " expected=" << expected
                  << " actual=" << actual << "\n";
        std::exit(1);
    }
}

inference::PolicyAction make_action(float base)
{
    inference::PolicyAction action{};
    for (std::size_t i = 0; i < action.size(); ++i) {
        action[i] = base + static_cast<float>(i) * 0.01F;
    }
    return action;
}

void expect_last_action_frame(
    const inference::PolicyRuntime::ObservationArray& observation,
    std::size_t frame,
    const inference::PolicyAction& expected)
{
    const std::size_t frame_offset =
        kLastActionOffset + frame * inference::policy_observation::kLastActionSize;
    for (std::size_t i = 0; i < expected.size(); ++i) {
        expect_near(observation[frame_offset + i],
                    expected[i],
                    "last_action history should use terms.last_action");
    }
}

void test_last_action_history_uses_previous_action_terms()
{
    inference::PolicyRuntime runtime;
    inference::PolicyObservationTerms terms;
    inference::PolicyRuntime::ObservationArray observation{};

    const inference::PolicyAction previous_action_0 = make_action(1.0F);
    terms.last_action = previous_action_0;
    runtime.build_observation(terms, observation);

    for (std::size_t frame = 0;
         frame < inference::policy_observation::kFrameStack;
         ++frame) {
        expect_last_action_frame(observation, frame, previous_action_0);
    }

    const inference::PolicyAction previous_action_1 = make_action(2.0F);
    terms.last_action = previous_action_1;
    runtime.build_observation(terms, observation);

    for (std::size_t frame = 0;
         frame + 1 < inference::policy_observation::kFrameStack;
         ++frame) {
        expect_last_action_frame(observation, frame, previous_action_0);
    }
    expect_last_action_frame(observation,
                             inference::policy_observation::kFrameStack - 1,
                             previous_action_1);
}

}  // namespace

int main()
{
    test_last_action_history_uses_previous_action_terms();

    std::cout << "policy_runtime_observation_test passed\n";
    return 0;
}
