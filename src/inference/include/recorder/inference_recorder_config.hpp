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
};

}  // namespace inference
