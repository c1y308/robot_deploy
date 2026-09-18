#pragma once

#include "policy/policy_observation_config.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

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

    PolicyRuntime();
    ~PolicyRuntime();

    PolicyRuntime(const PolicyRuntime&) = delete;
    PolicyRuntime& operator=(const PolicyRuntime&) = delete;

    bool load(const PolicyRuntimeConfig& config,
              int intra_op_threads = 0,
              int inter_op_threads = 0,
              int openblas_threads = 0,
              const std::vector<int>& expected_worker_cpus = {});
    void shutdown();

    bool is_loaded() const;
    bool infer(const PolicyObservation& observation, PolicyRuntimeStepResult& result);
    std::size_t observation_size() const noexcept { return observation_size_; }

    const std::string& last_error() const noexcept { return last_error_; }

private:
    struct Impl;

    bool dry_run_and_validate_output();
    bool configure_parallel_runtime(int intra_op_threads,
                                    int inter_op_threads,
                                    int openblas_threads);
    void unload();
    void set_error(std::string message);

    std::unique_ptr<Impl> impl_;
    std::size_t observation_size_{0};
    std::string last_error_;
};

}  // namespace inference
