#include "async_csv_logger.hpp"

#include <ctime>
#include <iomanip>
#include <sstream>
#include <utility>

namespace inference {
namespace {

std::string local_timestamp_for_filename()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch()).count() % 1000;

    std::tm local_time = {};
    localtime_r(&now_time, &local_time);

    std::ostringstream stream;
    stream << std::put_time(&local_time, "%Y%m%d_%H%M%S")
           << '_' << std::setw(3) << std::setfill('0') << ms;
    return stream.str();
}

}  // namespace

AsyncCsvLogger::~AsyncCsvLogger()
{
    stop();
}

bool AsyncCsvLogger::start(Config config)
{
    stop();

    if (config.file_prefix.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_error_ = "CSV logger file_prefix is empty";
        return false;
    }
    if (config.flush_interval.count() <= 0) {
        config.flush_interval = std::chrono::milliseconds(1000);
    }

    try {
        std::filesystem::create_directories(config.directory);
    } catch (const std::filesystem::filesystem_error& error) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_error_ = std::string("failed to create CSV log dir: ") + error.what();
        return false;
    }

    const std::filesystem::path path =
        config.directory /
        (config.file_prefix + "_" + local_timestamp_for_filename() + ".csv");

    std::ofstream file(path, std::ios::out);
    if (!file) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_error_ = "failed to open CSV log: " + path.string();
        return false;
    }

    file << config.header;
    if (!config.header.empty() && config.header.back() != '\n') {
        file << '\n';
    }
    if (!file) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_error_ = "failed to write CSV header: " + path.string();
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        config_ = std::move(config);
        log_path_ = path;
        file_ = std::move(file);
        queue_.clear();
        dropped_rows_ = 0;
        last_error_.clear();
        stop_requested_ = false;
        running_ = true;
    }

    worker_ = std::thread(&AsyncCsvLogger::worker_loop, this);
    return true;
}

void AsyncCsvLogger::stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_requested_ = true;
    }
    condition_.notify_all();

    if (worker_.joinable()) {
        worker_.join();
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
        running_ = false;
        stop_requested_ = false;
    }
}

bool AsyncCsvLogger::enqueue(std::string row)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ || stop_requested_) {
            return false;
        }

        if (config_.max_queue_depth > 0 &&
            queue_.size() >= config_.max_queue_depth) {
            queue_.pop_front();
            ++dropped_rows_;
        }

        queue_.push_back(std::move(row));
    }

    condition_.notify_one();
    return true;
}

bool AsyncCsvLogger::is_running() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return running_;
}

std::string AsyncCsvLogger::last_error() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return last_error_;
}

std::filesystem::path AsyncCsvLogger::log_path() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return log_path_;
}

std::uint64_t AsyncCsvLogger::dropped_rows() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_rows_;
}

void AsyncCsvLogger::worker_loop()
{
    const std::chrono::milliseconds flush_interval = [this] {
        std::lock_guard<std::mutex> lock(mutex_);
        return config_.flush_interval;
    }();
    auto next_flush = std::chrono::steady_clock::now() + flush_interval;
    bool dirty = true;

    while (true) {
        std::deque<std::string> rows;
        bool stopping = false;

        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait_until(lock, next_flush, [this] {
                return stop_requested_ || !queue_.empty();
            });
            queue_.swap(rows);
            stopping = stop_requested_;
        }

        for (const std::string& row : rows) {
            file_ << row << '\n';
            dirty = true;
        }

        if (!file_) {
            std::lock_guard<std::mutex> lock(mutex_);
            last_error_ = "failed to write CSV log: " + log_path_.string();
            running_ = false;
            stop_requested_ = false;
            file_.close();
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if ((dirty && now >= next_flush) || stopping) {
            file_.flush();
            dirty = false;
            next_flush = now + flush_interval;
        }

        if (!file_) {
            std::lock_guard<std::mutex> lock(mutex_);
            last_error_ = "failed to flush CSV log: " + log_path_.string();
            running_ = false;
            stop_requested_ = false;
            file_.close();
            return;
        }

        if (stopping) {
            break;
        }
    }

    file_.flush();
    file_.close();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        stop_requested_ = false;
        queue_.clear();
    }
}

}  // namespace inference
