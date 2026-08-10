#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <mutex>
#include <vector>

#include "motor_base/command_types.hpp"

namespace motor_base {

class CommandQueue {
public:
    explicit CommandQueue(std::size_t capacity)
        : buffer_(std::max<std::size_t>(1, capacity), ControlCommand::stop()),
          capacity_(buffer_.size())
    {
    }

    bool try_push(const ControlCommand& value)
    {
        std::lock_guard<std::mutex> lock(push_mutex_);
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        if (head - tail >= capacity_) {
            return false;
        }

        buffer_[head % capacity_] = value;
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    const ControlCommand* front_rt() const
    {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        const std::size_t head = head_.load(std::memory_order_acquire);
        if (tail == head) {
            return nullptr;
        }
        return &buffer_[tail % capacity_];
    }

    void pop_front_rt()
    {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        tail_.store(tail + 1, std::memory_order_release);
    }

private:
    std::vector<ControlCommand> buffer_;
    std::size_t capacity_;
    std::atomic<std::size_t> head_{0};
    std::atomic<std::size_t> tail_{0};
    std::mutex push_mutex_;
};

class DiscreteCommandQueue {
public:
    explicit DiscreteCommandQueue(std::size_t capacity = 0)
        : buffer_(std::max<std::size_t>(1, capacity)),
          capacity_(buffer_.size()),
          head_(0),
          count_(0)
    {
    }

    bool push_back_rt(const DiscreteCommand& value)
    {
        if (count_ >= capacity_) {
            return false;
        }

        const std::size_t tail = (head_ + count_) % capacity_;
        buffer_[tail] = value;
        ++count_;
        return true;
    }

    bool empty() const { return count_ == 0; }

    DiscreteCommand& front_rt()
    {
        return buffer_[head_];
    }

    void pop_front_rt()
    {
        if (count_ == 0) {
            return;
        }
        head_ = (head_ + 1) % capacity_;
        --count_;
    }

private:
    std::vector<DiscreteCommand> buffer_;
    std::size_t capacity_;
    std::size_t head_;
    std::size_t count_;
};

} // namespace motor_base
