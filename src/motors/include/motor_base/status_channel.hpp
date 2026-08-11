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
#include <thread>
#include <utility>
#include <vector>

#include "motor_base/command_types.hpp"

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

    MotorControlMode mode;
    MotorControlMode target_mode;

    MotorStatusSnapshot()
        : motor_index(-1), position_rad(0.0), velocity_rad_s(0.0),
          torque_percent(0.0), comm_ok(false), enabled(false), faulted(false),
          mode(MotorControlMode::NONE),
          target_mode(MotorControlMode::NONE) {}
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

    void configure(std::size_t motor_count, int publish_period_ms);

    // ──────────────────── 实时写入 API ────────────────────

    /// @brief 获取写入槽。Producer 独占，绝不阻塞。
    /// @return 总是成功（除非未配置 motor_count）。
    bool write(WriteToken& token);

    /// @brief 发布写入槽，对 Consumer 可见。绝不阻塞、不用 mutex、无 CAS retry。
    void publish(const WriteToken& token);

    // ──────────────────── 非实时读取 API ────────────────────

    std::vector<Snapshot> get_status() const;
    void set_callback(StatusCallback cb);

    // ──────────────────── 统计 ────────────────────

    /// @brief Producer 累计 publish 次数（包括被覆盖的帧）
    uint64_t get_publish_count() const;
    /// @brief 被覆盖的帧数（Producer 发布新帧时，旧 dirty Middle 未被 Consumer 读取）
    uint64_t get_overwritten_count() const;

    // ──────────────────── 生命周期 ────────────────────

    void start();
    void stop();

#ifdef MOTOR_BASE_STATUS_CHANNEL_TESTING
    bool copy_latest_status_for_test(std::vector<Snapshot>& out)
    {
        return copy_latest_status(out);
    }
#endif

private:
    // ================================================================
    // Triple-buffer: Writing / Middle / Reading
    // ================================================================
    //
    // Producer private Writing
    //         <-> atomic shared Middle
    //         <-> Consumer private Reading
    //
    // Producer 只交换 Writing <-> Middle；Consumer 只交换 Reading <-> Middle。
    // 没有 Producer <-> Reading 的直接 ownership 转移，因此 Consumer 正在复制
    // Reading 槽时，Producer 的下一轮 Writing 永远不会指向该槽。
    //
    // 关键不变量：
    //   - Producer 绝不等待 Consumer，不用 mutex，不做 CAS retry，不搜索槽位
    //   - Middle 至多保存一个 latest snapshot；dirty=true 表示未消费
    //   - 中间帧可被覆盖；这是 latest-value semantics，不是队列

    static constexpr std::size_t kStatusSlotCount = 3;

    using MiddleState = std::uint32_t;
    static constexpr MiddleState kSlotMask = 0x3u;
    static constexpr MiddleState kDirtyBit = 0x4u;

    static constexpr MiddleState make_middle_state(int slot,
                                                   bool dirty) noexcept
    {
        return (static_cast<MiddleState>(slot) & kSlotMask) |
               (dirty ? kDirtyBit : 0u);
    }

    static constexpr int middle_slot(MiddleState state) noexcept
    {
        return static_cast<int>(state & kSlotMask);
    }

    static constexpr bool middle_dirty(MiddleState state) noexcept
    {
        return (state & kDirtyBit) != 0u;
    }

    bool copy_latest_status(std::vector<Snapshot>& out);
    void thread_func();

    std::size_t motor_count_{0};
    int publish_period_ms_{1};

    // 三槽数据存储
    std::array<std::array<Snapshot, kMaxMotorCommandSetpoints>, kStatusSlotCount> slots_{};

    // writing_idx_: 仅 Producer 读写（非原子）
    int writing_idx_{0};
    // reading_idx_: 仅 Consumer 读写（非原子）
    int reading_idx_{1};
    // middle_state_: 唯一共享 ownership 点，低两位为槽号，bit2 为 dirty。
    std::atomic<MiddleState> middle_state_{make_middle_state(2, false)};

    // ── 统计 ──
    std::atomic<std::uint64_t> publish_count_{0};
    std::atomic<std::uint64_t> overwritten_count_{0};

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
void LatestStatusChannel<Snapshot>::configure(std::size_t motor_count,
                                              int publish_period_ms)
{
    stop();

    if (motor_count > kMaxMotorCommandSetpoints) {
        throw std::invalid_argument(
            "LatestStatusChannel motor_count exceeds fixed capacity");
    }

    motor_count_ = motor_count;
    publish_period_ms_ = std::max(1, publish_period_ms);

    status_cache_.assign(motor_count_, Snapshot());

    // configure 前已 stop()，此时没有 Producer/Consumer 并发访问。
    // 初始 ownership: slot0=Writing, slot1=Reading, slot2=Middle(clean)。
    writing_idx_ = 0;
    reading_idx_ = 1;
    middle_state_.store(make_middle_state(2, false),
                        std::memory_order_relaxed);

    publish_count_.store(0, std::memory_order_relaxed);
    overwritten_count_.store(0, std::memory_order_relaxed);
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

    // Producer 始终独占 writing_idx_，无需 CAS、无需查找；直接给令牌填充数据
    token.slot = static_cast<std::size_t>(writing_idx_);
    token.data = slots_[writing_idx_].data();
    return true;
}


// ---------------------------------------------------------------------------
// publish — Producer 发布写入槽（RT 路径，O(1) 无 CAS 循环）
// ---------------------------------------------------------------------------

template <typename Snapshot>
void LatestStatusChannel<Snapshot>::publish(const WriteToken& token)
{
    if (!token.data ||
        token.slot != static_cast<std::size_t>(writing_idx_) ||
        token.data != slots_[static_cast<std::size_t>(writing_idx_)].data()) {
        return;
    }

    // 当前轮的 write 槽作为下一轮 middle 槽
    const MiddleState new_middle = make_middle_state(writing_idx_, true);

    // 单次 RMW 完成 Writing <-> Middle：
    // - release: 发布前对 slots_[writing_idx_] 的普通写入对 Consumer 可见。
    // - acquire: 若旧 Middle 是 Consumer 刚释放的 Reading，Producer 在重用该槽
    //   为下一轮 Writing 前，能看到 Consumer 对该槽读取完成的 release。
    const MiddleState old_middle = middle_state_.exchange(new_middle, std::memory_order_acq_rel);

    if (middle_dirty(old_middle)) {
        overwritten_count_.fetch_add(1, std::memory_order_relaxed);
    }
    // 当前轮的 middle 槽作为下一轮 Writing 槽
    writing_idx_ = middle_slot(old_middle);
    publish_count_.fetch_add(1, std::memory_order_relaxed);
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
// 统计 getter
// ---------------------------------------------------------------------------

template <typename Snapshot>
uint64_t LatestStatusChannel<Snapshot>::get_publish_count() const
{
    return publish_count_.load(std::memory_order_relaxed);
}

template <typename Snapshot>
uint64_t LatestStatusChannel<Snapshot>::get_overwritten_count() const
{
    return overwritten_count_.load(std::memory_order_relaxed);
}


// ---------------------------------------------------------------------------
// start / stop — 发布线程生命周期
// ---------------------------------------------------------------------------

template <typename Snapshot>
void LatestStatusChannel<Snapshot>::start()
{
    bool expected = false;
    if (!publisher_running_.compare_exchange_strong(expected, true)) {
        return;
    }
    publisher_thread_ = std::thread(&LatestStatusChannel::thread_func, this);
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
bool LatestStatusChannel<Snapshot>::copy_latest_status(
    std::vector<Snapshot>& out)
{
    out.resize(motor_count_);

    const MiddleState released_reading = make_middle_state(reading_idx_, false);

    // 单次 RMW 完成 Reading <-> Middle：
    // - acquire: 若 old_middle.dirty=true 且来自 Producer release，Consumer 复制
    //   slots_[reading_idx_] 时能看到完整 Snapshot。
    // - release: 把上一轮 Reading 槽归还为 clean Middle；Producer 之后 acquire 到
    //   该槽时，Consumer 对该槽的读取已经完成。

    // 当前轮的 reading 槽作为下一轮 middle 槽
    const MiddleState old_middle = middle_state_.exchange(released_reading, std::memory_order_acq_rel);
    // 读取当前轮的 middle 槽
    reading_idx_ = middle_slot(old_middle);

    if (!middle_dirty(old_middle)) {
        return false;
    }

    const auto& src = slots_[static_cast<std::size_t>(reading_idx_)];
    for (std::size_t i = 0; i < motor_count_; ++i) {
        out[i] = src[i];
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


using MotorStatusChannel = LatestStatusChannel<MotorStatusSnapshot>;

} // namespace motor_base
