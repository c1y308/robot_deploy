#pragma once

#include <cstddef>
#include <stdexcept>
#include <vector>

namespace robot_base {

template <typename T>
class RingBuffer {
public:
    explicit RingBuffer(std::size_t capacity)
        : buffer_(capacity)
    {
        if (capacity == 0) {
            throw std::invalid_argument("RingBuffer capacity must be positive");
        }
    }

    std::size_t capacity() const noexcept
    {
        return buffer_.size();
    }

    std::size_t size() const noexcept
    {
        return size_;
    }

    bool empty() const noexcept
    {
        return size_ == 0;
    }

    bool full() const noexcept
    {
        return size_ == capacity();
    }

    bool push_back(const T& value)
    {
        if (full()) {
            return false;
        }

        slot(head_ + size_) = value;
        ++size_;
        return true;
    }

    T& front() noexcept
    {
        return slot(head_);
    }

    const T& front() const noexcept
    {
        return slot(head_);
    }

    void pop_front() noexcept
    {
        head_ = wrap(head_ + 1);
        --size_;
    }

    T& slot(std::size_t index) noexcept
    {
        return buffer_[wrap(index)];
    }

    const T& slot(std::size_t index) const noexcept
    {
        return buffer_[wrap(index)];
    }

private:
    std::size_t wrap(std::size_t index) const noexcept
    {
        return index % capacity();
    }

    std::vector<T> buffer_;
    std::size_t head_{0};
    std::size_t size_{0};
};

} // namespace robot_base
