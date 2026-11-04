#pragma once

#include "policy/policy_observation_config.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace inference {

struct PolicyRuntimeConfig;
class PolicySessionOptions;

struct PolicyRuntimeStepResult {
    PolicyAction raw_action{};
    std::int64_t inference_start_ns{0};
    std::int64_t inference_end_ns{0};
};

// 策略推理运行时
class PolicyRuntime {
public:
    static constexpr std::size_t kDof = kModelDof;

    PolicyRuntime();
    ~PolicyRuntime();

    PolicyRuntime(const PolicyRuntime&) = delete;
    PolicyRuntime& operator=(const PolicyRuntime&) = delete;

    // 由所有者线程串行调用；load 完成校验和预热，infer 复用固定缓冲区。
    bool load(const PolicyRuntimeConfig& config,
              const PolicySessionOptions& session_options);
    void shutdown();

    bool is_loaded() const;
    bool infer(const PolicyObservation& observation, PolicyRuntimeStepResult& result);

    const std::string& last_error() const noexcept { return last_error_; }

private:
    struct Impl;

    bool warm_up();

    std::unique_ptr<Impl> impl_;
    std::string last_error_;
};

}  // namespace inference
