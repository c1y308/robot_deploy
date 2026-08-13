#pragma once

#include "recorder/inference_record.hpp"

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

namespace inference {

class InferenceRecorder {
public:
    InferenceRecorder() = default;
    ~InferenceRecorder();

    InferenceRecorder(const InferenceRecorder&) = delete;
    InferenceRecorder& operator=(const InferenceRecorder&) = delete;

    bool start(InferenceRecorderConfig config);
    bool try_record(const InferenceRecord& record) noexcept;
    void stop() noexcept;

    bool running() const noexcept;
    std::uint64_t dropped_records() const noexcept;
    std::size_t max_queue_depth() const noexcept;
    std::string last_error() const;
    std::filesystem::path log_path() const;

private:
    void worker_loop() noexcept;

    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<InferenceRecord> queue_;
    std::thread worker_;
    std::ofstream file_;
    InferenceRecorderConfig config_;
    std::filesystem::path log_path_;
    std::string last_error_;
    std::uint64_t dropped_records_{0};
    std::int64_t session_start_timestamp_ns_{0};
    bool running_{false};
    bool stop_requested_{false};
};

}  // namespace inference
