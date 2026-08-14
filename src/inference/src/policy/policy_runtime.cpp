#include "policy/policy_runtime.hpp"
#include "policy/torch_policy_runner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace inference {
namespace {

std::int64_t policy_runtime_now_ns() noexcept
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

std::array<float, 2> gait_phase_observation(std::uint64_t episode_length,
                                            double step_dt,
                                            double period)
{
    constexpr double kTwoPi = 6.28318530717958647692;
    const double global_phase =
        std::fmod(static_cast<double>(episode_length) * step_dt, period) / period;
    return {
        static_cast<float>(std::sin(global_phase * kTwoPi)),
        static_cast<float>(std::cos(global_phase * kTwoPi))
    };
}

template <std::size_t TermSize, std::size_t ObservationSize>
void fill_term_history(std::array<float, ObservationSize>& history,
                       std::size_t offset,
                       std::size_t frame_stack,
                       const std::array<float, TermSize>& current_term)
{
    for (std::size_t frame = 0; frame < frame_stack; ++frame) {
        std::copy(current_term.begin(),
                  current_term.end(),
                  history.begin() + offset + frame * TermSize);
    }
}

template <std::size_t TermSize, std::size_t ObservationSize>
void append_term_history(std::array<float, ObservationSize>& history,
                         std::size_t offset,
                         std::size_t frame_stack,
                         const std::array<float, TermSize>& current_term)
{
    const std::size_t term_history_size = frame_stack * TermSize;
    std::copy(history.begin() + offset + TermSize,
              history.begin() + offset + term_history_size,
              history.begin() + offset);
    std::copy(current_term.begin(),
              current_term.end(),
              history.begin() + offset + term_history_size - TermSize);
}

}  // namespace

PolicyRuntime::PolicyRuntime() = default;
PolicyRuntime::~PolicyRuntime() = default;

bool PolicyRuntime::load(const PolicyConfig& config)
{
    shutdown();
    policy_config_ = config;
    last_error_.clear();

    if (policy_config_.model_path.empty()) {
        set_error("policy.model_path is empty");
        return false;
    }
    if (!std::isfinite(policy_config_.step_dt) ||
        policy_config_.step_dt <= 0.0) {
        set_error("policy.step_dt must be a finite positive value");
        return false;
    }
    if constexpr (policy_observation::kEnableGaitPhase) {
        if (!std::isfinite(policy_config_.gait_phase_period) ||
            policy_config_.gait_phase_period <= 0.0) {
            set_error("gait_phase_period must be a finite positive value");
            return false;
        }
    }

    auto runner = std::make_unique<TorchPolicyRunner>();
    if (!runner->load(policy_config_.model_path)) {
        set_error(runner->last_error());
        return false;
    }

    runner_ = std::move(runner);
    return true;
}

void PolicyRuntime::shutdown()
{
    runner_.reset();
    reset();
}

void PolicyRuntime::reset()
{
    last_action_raw_.fill(0.0F);
    observation_history_.fill(0.0F);
    observation_history_ready_ = false;
    episode_length_ = 0;
    frame_index_ = 0;
}

bool PolicyRuntime::is_loaded() const
{
    return runner_ && runner_->is_loaded();
}

bool PolicyRuntime::infer(const PolicyObservationTerms& terms,
                          PolicyRuntimeStepResult& result)
{
    if (!is_loaded()) {
        set_error("policy is not loaded");
        return false;
    }

    ObservationArray observation = {};
    build_observation(terms, observation);

    result.inference_start_ns = policy_runtime_now_ns();
    if (!runner_->infer(observation, result.raw_action)) {
        result.inference_end_ns = policy_runtime_now_ns();
        set_error(runner_->last_error());
        return false;
    }
    result.inference_end_ns = policy_runtime_now_ns();

    if (!std::all_of(result.raw_action.begin(),
                     result.raw_action.end(),
                     [](float value) { return std::isfinite(value); })) {
        set_error("policy output contains non-finite value");
        return false;
    }

    last_action_raw_ = result.raw_action;
    last_error_.clear();
    return true;
}

void PolicyRuntime::advance_frame() noexcept
{
    ++frame_index_;
}

void PolicyRuntime::advance_episode() noexcept
{
    ++episode_length_;
}

void PolicyRuntime::build_observation(const PolicyObservationTerms& terms,
                                      ObservationArray& observation)
{
    constexpr std::size_t kBaseAngVelOffset = 0;
    constexpr std::size_t kBaseAngVelHistorySize =
        policy_observation::kFrameStack * policy_observation::kBaseAngVelSize;
    constexpr std::size_t kProjectedGravityOffset =
        kBaseAngVelOffset + kBaseAngVelHistorySize;
    constexpr std::size_t kProjectedGravityHistorySize =
        policy_observation::kFrameStack * policy_observation::kProjectedGravitySize;
    constexpr std::size_t kVelocityCommandsOffset =
        kProjectedGravityOffset + kProjectedGravityHistorySize;
    constexpr std::size_t kVelocityCommandsHistorySize =
        policy_observation::kFrameStack * policy_observation::kVelocityCommandsSize;
    constexpr std::size_t kGaitPhaseOffset =
        kVelocityCommandsOffset + kVelocityCommandsHistorySize;
    constexpr std::size_t kGaitPhaseHistorySize =
        policy_observation::kEnableGaitPhase
            ? policy_observation::kFrameStack * policy_observation::kGaitPhaseSize
            : 0;
    constexpr std::size_t kJointPosRelOffset =
        kGaitPhaseOffset + kGaitPhaseHistorySize;
    constexpr std::size_t kJointPosRelHistorySize =
        policy_observation::kFrameStack * policy_observation::kJointPosRelSize;
    constexpr std::size_t kJointVelRelOffset =
        kJointPosRelOffset + kJointPosRelHistorySize;
    constexpr std::size_t kJointVelRelHistorySize =
        policy_observation::kFrameStack * policy_observation::kJointVelRelSize;
    constexpr std::size_t kLastActionOffset =
        kJointVelRelOffset + kJointVelRelHistorySize;
    constexpr std::size_t kObservationEnd =
        kLastActionOffset + policy_observation::kFrameStack * policy_observation::kLastActionSize;
    static_assert(kObservationEnd == policy_observation::kObservationSize,
                  "policy observation offsets must cover the configured input size");

    if (!observation_history_ready_) {
        fill_term_history(observation_history_, kBaseAngVelOffset,
                          kFrameStack, terms.base_ang_vel);
        fill_term_history(observation_history_, kProjectedGravityOffset,
                          kFrameStack, terms.projected_gravity);
        fill_term_history(observation_history_, kVelocityCommandsOffset,
                          kFrameStack, terms.velocity_commands);
        if constexpr (policy_observation::kEnableGaitPhase) {
            const std::array<float, 2> gait_phase =
                gait_phase_observation(episode_length_,
                                       policy_config_.step_dt,
                                       policy_config_.gait_phase_period);
            fill_term_history(observation_history_, kGaitPhaseOffset,
                              kFrameStack, gait_phase);
        }
        fill_term_history(observation_history_, kJointPosRelOffset,
                          kFrameStack, terms.joint_pos_rel);
        fill_term_history(observation_history_, kJointVelRelOffset,
                          kFrameStack, terms.joint_vel_rel);
        fill_term_history(observation_history_, kLastActionOffset,
                          kFrameStack, terms.last_action);
        observation_history_ready_ = true;
    } else {
        append_term_history(observation_history_, kBaseAngVelOffset,
                            kFrameStack, terms.base_ang_vel);
        append_term_history(observation_history_, kProjectedGravityOffset,
                            kFrameStack, terms.projected_gravity);
        append_term_history(observation_history_, kVelocityCommandsOffset,
                            kFrameStack, terms.velocity_commands);
        if constexpr (policy_observation::kEnableGaitPhase) {
            const std::array<float, 2> gait_phase =
                gait_phase_observation(episode_length_,
                                       policy_config_.step_dt,
                                       policy_config_.gait_phase_period);
            append_term_history(observation_history_, kGaitPhaseOffset,
                                kFrameStack, gait_phase);
        }
        append_term_history(observation_history_, kJointPosRelOffset,
                            kFrameStack, terms.joint_pos_rel);
        append_term_history(observation_history_, kJointVelRelOffset,
                            kFrameStack, terms.joint_vel_rel);
        append_term_history(observation_history_, kLastActionOffset,
                            kFrameStack, terms.last_action);
    }

    observation = observation_history_;
}

void PolicyRuntime::set_error(std::string message)
{
    last_error_ = std::move(message);
}

}  // namespace inference
