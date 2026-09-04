#pragma once

#include "policy/policy_observation_config.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace inference {

struct PolicyRuntimeConfig;

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

    bool load(const PolicyRuntimeConfig& config);
    void shutdown();

    bool is_loaded() const;
    bool infer(const PolicyObservation& observation, PolicyRuntimeStepResult& result);

    const std::string& last_error() const noexcept { return last_error_; }

private:
    struct Impl;

    bool dry_run_and_validate_output();
    void unload();
    void set_error(std::string message);

    std::unique_ptr<Impl> impl_;
    bool loaded_{false};
    std::string last_error_;
};

}  // namespace inference
