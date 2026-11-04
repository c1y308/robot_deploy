#pragma once

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <string>

namespace inference {

inline constexpr std::size_t kDefaultInferenceRecorderQueueDepth = 4096;

struct InferenceRecorderConfig {
    bool enabled{true};
    std::filesystem::path directory{"log"};
    std::string file_prefix{};
    std::chrono::milliseconds flush_interval{1000};
    std::size_t max_queue_depth{kDefaultInferenceRecorderQueueDepth};
    // 默认使用 675 维观测；RobotInterface 初始化时按所选观测布局设置。
    std::size_t policy_observation_size{675};
};

}  // namespace inference
