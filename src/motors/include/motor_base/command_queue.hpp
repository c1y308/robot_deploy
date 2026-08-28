#pragma once

#include <atomic>
#include <cstddef>
#include <mutex>

#include "ringbuffer/ring_buffer.hpp"
#include "motor_base/command_types.hpp"

namespace motor_base {

class CommandQueue {
public:
    explicit CommandQueue(std::size_t capacity)
        : buffer_(capacity)
    {
    }

    bool try_push(const ControlCommand& value)
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

    const ControlCommand* front() const
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
    robot_base::RingBuffer<ControlCommand> buffer_;
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

} // namespace motor_base
