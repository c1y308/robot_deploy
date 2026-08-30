#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>

#include "ringbuffer/ring_buffer.hpp"
#include "motor_base/command_types.hpp"

namespace motor_base {

inline std::uint32_t discrete_command_target_mask(
    const ControlCommand& cmd,
    std::size_t motor_count)
{
    if (cmd.motor_index == ControlCommand::kAllMotors) {
        return (std::uint32_t{1} << motor_count) - 1U;
    }
    return std::uint32_t{1} << static_cast<std::uint32_t>(cmd.motor_index);
}

class CommandQueue {
public:
    struct Entry {
        ControlCommand command;
        std::optional<CommandId> command_id;
    };

    explicit CommandQueue(std::size_t capacity)
        : buffer_(capacity)
    {
    }

    bool try_push(const Entry& value)
    {
        std::lock_guard<std::mutex> lock(push_mutex_);
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        if (head - tail >= buffer_.capacity()) {
            return false;
        }

        buffer_.slot(head) = value;
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    const Entry* front() const
    {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        const std::size_t head = head_.load(std::memory_order_acquire);
        if (tail == head) {
            return nullptr;
        }
        return &buffer_.slot(tail);
    }

    void pop_front()
    {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        tail_.store(tail + 1, std::memory_order_release);
    }

private:
    robot_base::RingBuffer<Entry> buffer_;
    std::atomic<std::size_t> head_{0};
    std::atomic<std::size_t> tail_{0};
    std::mutex push_mutex_;
};

class DiscreteCommandQueue {
public:
    explicit DiscreteCommandQueue(std::size_t capacity)
        : buffer_(capacity)
    {
    }

    bool push_back(const DiscreteCommand& value)
    {
        return buffer_.push_back(value);
    }

    bool empty() const { return buffer_.empty(); }

    DiscreteCommand& front()
    {
        return buffer_.front();
    }

    void pop_front()
    {
        buffer_.pop_front();
    }

private:
    robot_base::RingBuffer<DiscreteCommand> buffer_;
};

class DiscreteCommandResultTracker {
public:
    void initialize(CommandId id, std::uint32_t target_mask)
    {
        Slot& slot = slots_[slot_index(id)];
        slot.target_mask.store(target_mask, std::memory_order_relaxed);
        slot.done_mask.store(0, std::memory_order_relaxed);
        slot.failed_mask.store(0, std::memory_order_relaxed);
        slot.command_id.store(id, std::memory_order_release);
    }

    void clear(CommandId id)
    {
        Slot& slot = slots_[slot_index(id)];
        if (slot.command_id.load(std::memory_order_acquire) == id) {
            slot.command_id.store(0, std::memory_order_release);
        }
    }

    void mark_done(CommandId id, int motor_index)
    {
        Slot& slot = slots_[slot_index(id)];
        if (slot.command_id.load(std::memory_order_acquire) == id) {
            slot.done_mask.fetch_or(
                std::uint32_t{1} << static_cast<std::uint32_t>(motor_index),
                std::memory_order_release);
        }
    }

    void mark_failed(CommandId id, int motor_index)
    {
        Slot& slot = slots_[slot_index(id)];
        if (slot.command_id.load(std::memory_order_acquire) == id) {
            slot.failed_mask.fetch_or(
                std::uint32_t{1} << static_cast<std::uint32_t>(motor_index),
                std::memory_order_release);
        }
    }

    DiscreteCommandResult get(CommandId id) const
    {
        if (id == 0) {
            return DiscreteCommandResult::UNKNOWN;
        }

        const Slot& slot = slots_[slot_index(id)];
        if (slot.command_id.load(std::memory_order_acquire) != id) {
            return DiscreteCommandResult::UNKNOWN;
        }

        if (slot.failed_mask.load(std::memory_order_acquire) != 0) {
            return DiscreteCommandResult::FAILED;
        }

        const std::uint32_t target_mask =
            slot.target_mask.load(std::memory_order_acquire);
        const std::uint32_t done_mask =
            slot.done_mask.load(std::memory_order_acquire);
        return ((done_mask & target_mask) == target_mask)
                   ? DiscreteCommandResult::SUCCEEDED
                   : DiscreteCommandResult::PENDING;
    }

private:
    static constexpr std::size_t kCapacity = 64;

    struct Slot {
        std::atomic<CommandId> command_id{0};
        std::atomic<std::uint32_t> target_mask{0};
        std::atomic<std::uint32_t> done_mask{0};
        std::atomic<std::uint32_t> failed_mask{0};
    };

    static std::size_t slot_index(CommandId id) noexcept
    {
        return static_cast<std::size_t>((id - 1U) % kCapacity);
    }

    std::array<Slot, kCapacity> slots_{};
};

} // namespace motor_base
