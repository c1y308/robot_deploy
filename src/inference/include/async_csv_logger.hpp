#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

namespace inference {

class AsyncCsvLogger {
public:
    struct Config {
        std::filesystem::path directory;
        std::string file_prefix;
        std::string header;
        std::chrono::milliseconds flush_interval{1000};
        std::size_t max_queue_depth = 4096;
    };

    AsyncCsvLogger() = default;
    ~AsyncCsvLogger();

    AsyncCsvLogger(const AsyncCsvLogger&) = delete;
    AsyncCsvLogger& operator=(const AsyncCsvLogger&) = delete;

    bool start(Config config);
    void stop();

    bool enqueue(std::string row);
    bool is_running() const;

    std::string last_error() const;
    std::filesystem::path log_path() const;
    std::uint64_t dropped_rows() const;

private:
    void worker_loop();

    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<std::string> queue_;
    std::thread worker_;
    std::ofstream file_;
    Config config_;
    std::filesystem::path log_path_;
    std::string last_error_;
    std::uint64_t dropped_rows_ = 0;
    bool running_ = false;
    bool stop_requested_ = false;
};

}  // namespace inference
