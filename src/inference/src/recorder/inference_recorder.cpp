#include "recorder/inference_recorder.hpp"

#include <ctime>
#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>

namespace inference {
namespace {

/* 返回本地时间戳，用于文件名，例如：0813_1425 */
std::string local_timestamp_for_filename()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    std::tm local_time = {};
    localtime_r(&now_time, &local_time);

    std::ostringstream stream;
    stream << std::put_time(&local_time, "%m%d_%H%M");
    return stream.str();
}


std::string make_log_filename(const InferenceRecorderConfig& config)
{
    const std::string timestamp = local_timestamp_for_filename();
    if (config.file_prefix.empty()) {
        return timestamp + ".csv";
    }
    return config.file_prefix + "_" + timestamp + ".csv";
}

/* 将带索引的列名追加到输出流中 */
void append_indexed_columns(std::ostream& stream,
                            const char*   prefix,
                            std::size_t   count)
{
    for (std::size_t i = 0; i < count; ++i) {
        stream << ',' << prefix << '_' << i;
    }
}

// 将电机相关的列名追加到输出流中
void append_motor_columns(std::ostream& stream,
                          const char*   prefix,
                          std::size_t   count)
{
    for (std::size_t i = 0; i < count; ++i) {
        stream << ',' << prefix << "_M" << i;
    }
}


void write_header(std::ostream& stream)
{
    stream << "frame_index, elapsed_us, state_timestamp_ns, inference_start_ns, "
           << "inference_end_ns, inference_duration_us, command_timestamp_ns, "
           << "command_applied";
    append_indexed_columns(stream, "raw_action", kInferenceDof);
    append_indexed_columns(stream, "target_q_model_rad", kInferenceDof);
    append_motor_columns(stream, "target_pos_rad", kInferenceMotorCount);
    append_motor_columns(stream, "target_effort_permille", kInferenceMotorCount);
    append_motor_columns(stream, "rx_pos_rad", kInferenceMotorCount);
    append_motor_columns(stream, "rx_vel_rad_s", kInferenceMotorCount);
    append_motor_columns(stream, "torque_pct", kInferenceMotorCount);
    append_motor_columns(stream, "comm_ok", kInferenceMotorCount);
    append_motor_columns(stream, "enabled", kInferenceMotorCount);
    append_motor_columns(stream, "faulted", kInferenceMotorCount);
    stream << '\n';
}


std::int64_t elapsed_us_for_record(const InferenceRecord& record,
                                   std::int64_t session_start_timestamp_ns)
{
    return (record.inference_start_ns - session_start_timestamp_ns) / 1000;
}


std::int64_t inference_duration_us_for_record(const InferenceRecord& record)
{
    return (record.inference_end_ns - record.inference_start_ns) / 1000;
}


template <typename Value, std::size_t Count>
void append_values(std::ostream& stream, const std::array<Value, Count>& values)
{
    for (const auto& value : values) {
        stream << ',' << value;
    }
}


template <std::size_t Count>
void append_u8_values(std::ostream& stream,
                      const std::array<std::uint8_t, Count>& values)
{
    for (std::uint8_t value : values) {
        stream << ',' << static_cast<int>(value);
    }
}


void write_record(std::ostream&          stream,
                  const InferenceRecord& record,
                  std::int64_t           session_start_timestamp_ns)
{
    stream << std::setprecision(17)
           << record.frame_index << ','
           << elapsed_us_for_record(record, session_start_timestamp_ns) << ','
           << record.state_timestamp_ns << ','
           << record.inference_start_ns << ','
           << record.inference_end_ns << ','
           << inference_duration_us_for_record(record) << ','
           << record.command_timestamp_ns << ','
           << (record.command_applied ? 1 : 0);

    append_values(stream, record.raw_action);
    append_values(stream, record.target_q_model_rad);
    append_values(stream, record.target_pos_rad);
    append_values(stream, record.target_effort_permille);
    append_values(stream, record.rx_pos_rad);
    append_values(stream, record.rx_vel_rad_s);
    append_values(stream, record.torque_percent);
    append_u8_values(stream, record.comm_ok);
    append_u8_values(stream, record.enabled);
    append_u8_values(stream, record.faulted);
    stream << '\n';
}

}  // namespace

InferenceRecorder::~InferenceRecorder()
{
    stop();
}

bool InferenceRecorder::start(InferenceRecorderConfig config)
{
    stop();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        config_ = config;
        last_error_.clear();
        dropped_records_ = 0;
    }

    if (!config.enabled) {
        return true;
    }

    if (config.flush_interval.count() <= 0) {
        config.flush_interval = std::chrono::milliseconds(1000);
    }
    if (config.max_queue_depth == 0) {
        config.max_queue_depth = kDefaultInferenceRecorderQueueDepth;
    }

    try {
        std::filesystem::create_directories(config.directory);
    } catch (const std::filesystem::filesystem_error& error) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_error_ = std::string("failed to create inference log dir: ") + error.what();
        return false;
    }

    const std::filesystem::path path =
        config.directory / make_log_filename(config);

    std::ofstream file(path, std::ios::out);
    if (!file) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_error_ = "failed to open inference log: " + path.string();
        return false;
    }

    write_header(file);
    if (!file) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_error_ = "failed to write inference log header: " + path.string();
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        config_ = std::move(config);
        log_path_ = path;
        file_ = std::move(file);
        queue_.clear();
        dropped_records_ = 0;
        last_error_.clear();
        session_start_timestamp_ns_ = steady_now_ns();
        stop_requested_ = false;
        running_ = true;
    }

    try {
        worker_ = std::thread(&InferenceRecorder::worker_loop, this);
    } catch (const std::exception& error) {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        stop_requested_ = false;
        queue_.clear();
        if (file_.is_open()) {
            file_.close();
        }
        last_error_ =
            std::string("failed to start inference recorder worker: ") +
            error.what();
        return false;
    } catch (...) {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        stop_requested_ = false;
        queue_.clear();
        if (file_.is_open()) {
            file_.close();
        }
        last_error_ = "failed to start inference recorder worker";
        return false;
    }

    return true;
}


bool InferenceRecorder::try_record(const InferenceRecord& record) noexcept
{
    try {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!running_ || stop_requested_) {
                return false;
            }

            if (queue_.size() >= config_.max_queue_depth) {
                queue_.pop_front();
                ++dropped_records_;
            }

            queue_.push_back(record);
        }

        condition_.notify_one();
        return true;
    } catch (const std::exception& error) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_error_ = std::string("failed to queue inference record: ") + error.what();
        return false;
    } catch (...) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_error_ = "failed to queue inference record";
        return false;
    }
}


void InferenceRecorder::stop() noexcept
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


bool InferenceRecorder::running() const noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    return running_;
}


std::uint64_t InferenceRecorder::dropped_records() const noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_records_;
}


std::size_t InferenceRecorder::max_queue_depth() const noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    return config_.max_queue_depth;
}


std::string InferenceRecorder::last_error() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return last_error_;
}


std::filesystem::path InferenceRecorder::log_path() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return log_path_;
}


void InferenceRecorder::worker_loop() noexcept
{
    const auto flush_interval = [this] {
        std::lock_guard<std::mutex> lock(mutex_);
        return config_.flush_interval;
    }();
    
    const std::int64_t session_start_timestamp_ns = [this] {
        std::lock_guard<std::mutex> lock(mutex_);
        return session_start_timestamp_ns_;
    }();

    auto next_flush = std::chrono::steady_clock::now() + flush_interval;
    bool dirty = true;

    while (true) {
        std::deque<InferenceRecord> records;
        bool stopping = false;

        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait_until(lock, next_flush, [this] {
                return stop_requested_ || !queue_.empty();
            });
            queue_.swap(records);
            stopping = stop_requested_;
        }

        for (const InferenceRecord& record : records) {
            write_record(file_, record, session_start_timestamp_ns);
            dirty = true;
        }

        if (!file_) {
            std::lock_guard<std::mutex> lock(mutex_);
            last_error_ = "failed to write inference log: " + log_path_.string();
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
            last_error_ = "failed to flush inference log: " + log_path_.string();
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
