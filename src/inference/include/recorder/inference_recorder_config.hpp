#pragma once

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <string>

namespace inference {

inline constexpr std::size_t kDefaultInferenceRecorderQueueDepth = 4096;

struct InferenceRecorderConfig {
    bool enabled;
    std::filesystem::path directory;
    std::string file_prefix;
    std::chrono::milliseconds flush_interval;
    std::size_t max_queue_depth;
};

}  // namespace inference
