#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <cstddef>
#include <type_traits>

namespace robot_base {

template <typename T>
class SpscLatestValue {
    static_assert(std::is_trivially_copyable<T>::value,
                  "SpscLatestValue<T> requires trivially copyable T");

public:
    SpscLatestValue() = default;

    SpscLatestValue(const SpscLatestValue&) = delete;
    SpscLatestValue& operator=(const SpscLatestValue&) = delete;

    void reset_empty() noexcept
    {
        writing_slot_ = 0;
        reading_slot_ = 1;
        middle_state_.store(make_middle_state(2, false),
                            std::memory_order_relaxed);
    }

    void reset_with_value(const T& value) noexcept
    {
        reset_empty();
        slots_[2] = value;
        middle_state_.store(make_middle_state(2, true),
                            std::memory_order_relaxed);
    }


    /// @brief 拷贝发布：将值拷入写入槽后发布。
    /// @return 是否覆盖了消费者尚未读取的旧帧
    bool publish(const T& value) noexcept
    {
        // 获取当前写入槽指针，并完成数据写入
        *acquire_write_slot() = value;
        // 发布当前写入槽（把当前写入槽作为新的 middle 槽），并把当前 middle 槽作为新的写入槽
        return publish_written();
    }

    /// @brief 获取当前写入槽指针。仅生产者线程可调用；
    ///        在调用 publish_written() 之前必须完成对该槽的全部写入。
    T* acquire_write_slot() noexcept
    {
        return &slots_[static_cast<std::size_t>(writing_slot_)];
    }

    /// @brief  发布当前写入槽（零拷贝）。仅生产者线程可调用。
    /// @return 是否覆盖了消费者尚未读取的旧帧
    bool publish_written() noexcept
    {
        const MiddleState old_middle =
            middle_state_.exchange(make_middle_state(writing_slot_, true),
                                   std::memory_order_acq_rel);
        writing_slot_ = middle_slot(old_middle);
        return middle_dirty(old_middle);
    }


    bool try_consume_latest(T& out) noexcept
    {
        const MiddleState released_reading = make_middle_state(reading_slot_, false);

        const MiddleState old_middle = middle_state_.exchange(released_reading, std::memory_order_acq_rel);

        reading_slot_ = middle_slot(old_middle);

        if (!middle_dirty(old_middle)) {
            return false;
        }

        out = slots_[static_cast<std::size_t>(reading_slot_)];
        return true;
    }

private:
    static constexpr std::size_t kSlotCount = 3;
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

    std::array<T, kSlotCount> slots_{};
    int writing_slot_{0};

    // reading_slot_ 表示 消费者当前占有/保护的槽位，避免 producer 写这个槽。
    int reading_slot_{1};

    // 读取的时候读取这个槽
    std::atomic<MiddleState> middle_state_{make_middle_state(2, false)};
};

}  // namespace robot_base
