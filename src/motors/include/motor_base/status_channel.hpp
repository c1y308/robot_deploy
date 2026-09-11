#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "spsc_latest_channel/spsc_latest_channel.hpp"
#include "motor_base/command_types.hpp"
#include "tool/thread_runtime.hpp"

namespace motor_base {

/* 电机状态快照 */
struct MotorStatusSnapshot {
    int motor_index;

    double position_rad;
    double velocity_rad_s;
    double torque_percent;

    bool comm_ok;
    bool enabled;
    bool faulted;
    bool control_ready;

    MotorControlMode mode;
    MotorControlMode target_mode;
    std::int64_t host_timestamp_ns;

    MotorStatusSnapshot()
        : motor_index(-1), position_rad(0.0), velocity_rad_s(0.0),
          torque_percent(0.0), comm_ok(false), enabled(false), faulted(false),
          control_ready(false), mode(MotorControlMode::NONE),
          target_mode(MotorControlMode::NONE), host_timestamp_ns(0) {}
};


/// @brief 仅保留最新状态的发布-订阅通道（triple-buffer 实现）。
///
/// 三个槽对应三个固定 ownership role：Writing / Middle / Reading。
/// 实时写入线程通过 write / publish 发布快照；
/// 非实时发布线程按固定周期通过 copy_latest_status 消费最新快照。
///
/// @tparam Snapshot channel 传输的数据类型
template <typename Snapshot>
class LatestStatusChannel {
public:
    using StatusCallback = std::function<void(const std::vector<Snapshot>&)>;

    /// @brief 写入槽预留凭证。由 write 填充，publish 消费。
    struct WriteToken {
        std::size_t slot;
        Snapshot*   data;

        WriteToken() : slot(0), data(nullptr) {}
    };

    LatestStatusChannel();
    ~LatestStatusChannel();

    // ──────────────────── 配置 ────────────────────

    void configure(
        std::size_t motor_count,
        int publish_period_ms,
        std::string thread_name = "motor_status",
        robot_base::ThreadRuntimeOptions thread_options = {});

    // ──────────────────── 实时写入 API ────────────────────

    /// @brief 获取写入槽。Producer 独占，绝不阻塞。
    /// @return 总是成功（除非未配置 motor_count）。
    bool write(WriteToken& token);

    /// @brief 发布写入槽，对 Consumer 可见。绝不阻塞、不用 mutex、无 CAS retry。
    void publish(const WriteToken& token);

    // ──────────────────── 非实时读取 API ────────────────────

    std::vector<Snapshot> get_status() const;
    void set_callback(StatusCallback cb);

    // ──────────────────── 生命周期 ────────────────────

    bool start();
    void stop();
    const std::string& last_start_error() const noexcept { return last_start_error_; }

private:
    struct StatusFrame {
        std::size_t count{0};
        std::array<Snapshot, kMaxMotorCommandSetpoints> values{};
    };

    bool copy_latest_status(std::vector<Snapshot>& out);
    void thread_func();

    std::size_t motor_count_{0};
    int publish_period_ms_{1};   // 读取频率
    std::string thread_name_{"motor_status"};
    robot_base::ThreadRuntimeOptions thread_options_;
    std::string last_start_error_;

    robot_base::SpscLatestChannel<StatusFrame> latest_frame_;

    std::vector<Snapshot> status_cache_;
    mutable std::mutex    status_cache_mutex_;

    StatusCallback status_callback_;
    mutable std::mutex callback_mutex_;

    std::thread publisher_thread_;
    std::atomic<bool> publisher_running_{false};
};


// ============================================================================
// 构造 / 析构
// ============================================================================

template <typename Snapshot>
LatestStatusChannel<Snapshot>::LatestStatusChannel()
{
    // 初始 ownership: slot0=Writing, slot1=Reading, slot2=Middle(clean)
}

template <typename Snapshot>
LatestStatusChannel<Snapshot>::~LatestStatusChannel()
{
    stop();
}


// ============================================================================
// 公有方法
// ============================================================================

template <typename Snapshot>
void LatestStatusChannel<Snapshot>::configure(
    std::size_t motor_count,
    int publish_period_ms,
    std::string thread_name,
    robot_base::ThreadRuntimeOptions thread_options)
{
    if (motor_count > kMaxMotorCommandSetpoints) {
        throw std::invalid_argument(
            "LatestStatusChannel motor_count exceeds fixed capacity");
    }
    if (publish_period_ms <= 0) {
        throw std::invalid_argument(
            "LatestStatusChannel publish_period_ms must be positive");
    }

    stop();

    motor_count_ = motor_count;
    publish_period_ms_ = publish_period_ms;
    thread_name_ = std::move(thread_name);
    thread_options_ = std::move(thread_options);

    status_cache_.assign(motor_count_, Snapshot());

    latest_frame_.reset_empty();
}


// ---------------------------------------------------------------------------
// write — Producer 获取写入槽（RT 路径，永不阻塞）
// ---------------------------------------------------------------------------

template <typename Snapshot>
bool LatestStatusChannel<Snapshot>::write(WriteToken& token)
{
    token = WriteToken();
    if (motor_count_ == 0)
        return false;

    StatusFrame* frame = latest_frame_.acquire_write_slot();
    frame->count = motor_count_;
    token.slot = 0;
    token.data = frame->values.data();
    return true;
}

// ---------------------------------------------------------------------------
// publish — Producer 发布写入槽（RT 路径，O(1) 无 CAS 循环）
// ---------------------------------------------------------------------------

template <typename Snapshot>
void LatestStatusChannel<Snapshot>::publish(const WriteToken& token)
{
    if (!token.data ||
        token.slot != 0 ||
        token.data != latest_frame_.acquire_write_slot()->values.data()) {
        return;
    }

    latest_frame_.publish_written();
}


// ---------------------------------------------------------------------------
// get_status — 获取缓存的最新状态（线程安全）
// ---------------------------------------------------------------------------

template <typename Snapshot>
std::vector<Snapshot> LatestStatusChannel<Snapshot>::get_status() const
{
    std::lock_guard<std::mutex> lock(status_cache_mutex_);
    return status_cache_;
}


// ---------------------------------------------------------------------------
// set_callback — 设置状态回调
// ---------------------------------------------------------------------------

template <typename Snapshot>
void LatestStatusChannel<Snapshot>::set_callback(StatusCallback cb)
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    status_callback_ = std::move(cb);
}




// ---------------------------------------------------------------------------
// start / stop — 发布线程生命周期
// ---------------------------------------------------------------------------

template <typename Snapshot>
bool LatestStatusChannel<Snapshot>::start()
{
    bool expected = false;
    if (!publisher_running_.compare_exchange_strong(expected, true)) {
        return true;
    }
    if (!robot_base::start_configured_thread(
            publisher_thread_, thread_name_.c_str(), thread_options_,
            [this] { thread_func(); }, last_start_error_)) {
        publisher_running_.store(false);
        return false;
    }
    return true;
}


template <typename Snapshot>
void LatestStatusChannel<Snapshot>::stop()
{
    if (!publisher_running_.exchange(false)) {
        return;
    }
    if (publisher_thread_.joinable()) {
        publisher_thread_.join();
    }
}


// ============================================================================
// 私有方法
// ============================================================================


// ---------------------------------------------------------------------------
// copy_latest_status — Consumer 获取并复制最新 Middle 帧
// ---------------------------------------------------------------------------

template <typename Snapshot>
bool LatestStatusChannel<Snapshot>::copy_latest_status(std::vector<Snapshot>& out)
{
    out.resize(motor_count_);

    StatusFrame latest;
    if (!latest_frame_.try_consume_latest(latest)) {
        return false;
    }

    const std::size_t count =
        std::min<std::size_t>(latest.count, motor_count_);
    out.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        out[i] = latest.values[i];
    }

    return true;
}


// ---------------------------------------------------------------------------
// thread_func — 发布线程主循环
// ---------------------------------------------------------------------------

template <typename Snapshot>
void LatestStatusChannel<Snapshot>::thread_func()
{
    std::vector<Snapshot> latest(motor_count_);

    while (publisher_running_) {
        if (copy_latest_status(latest)) {
            {
                std::lock_guard<std::mutex> lock(status_cache_mutex_);
                status_cache_ = latest;
            }

            StatusCallback cb;
            {
                std::lock_guard<std::mutex> lock(callback_mutex_);
                cb = status_callback_;
            }
            if (cb) {
                cb(latest);
            }
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(publish_period_ms_));
    }

    // 停止前最后一次拷贝
    if (copy_latest_status(latest)) {
        std::lock_guard<std::mutex> lock(status_cache_mutex_);
        status_cache_ = latest;
    }
}

} // namespace motor_base
