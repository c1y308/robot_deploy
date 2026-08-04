#include "motor_status_channel.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <utility>

namespace myactua {

MotorStatusChannel::MotorStatusChannel()
{
    reset_slots();
}

MotorStatusChannel::~MotorStatusChannel()
{
    stop();
}

void MotorStatusChannel::configure(std::size_t motor_count, int publish_period_ms)
{
    stop();

    if (motor_count > kMaxMotorCommandSetpoints) {
        throw std::invalid_argument("MotorStatusChannel motor_count exceeds fixed capacity");
    }

    motor_count_ = motor_count;
    publish_period_ms_ = std::max(1, publish_period_ms);
    status_cache_.assign(motor_count_, MotorStatusSnapshot());
    status_latest_slot_.store(-1, std::memory_order_relaxed);
    status_next_write_slot_ = 0;
    reset_slots();
}

bool MotorStatusChannel::try_begin_write_rt(WriteToken& token)
{
    token = WriteToken();
    if (motor_count_ == 0) {
        return false;
    }

    std::size_t slot = 0;
    bool reserved = false;
    bool overwrote_ready_slot = false;

    for (std::size_t n = 0; n < kStatusSlotCount; ++n) {
        const std::size_t candidate =
            (status_next_write_slot_ + n) % kStatusSlotCount;
        std::uint8_t expected = kStatusSlotEmpty;
        if (status_slot_state_[candidate].compare_exchange_strong(
                expected,
                kStatusSlotWriting,
                std::memory_order_acq_rel,
                std::memory_order_acquire)) {
            slot = candidate;
            reserved = true;
            break;
        }
    }

    if (!reserved) {
        for (std::size_t n = 0; n < kStatusSlotCount; ++n) {
            const std::size_t candidate =
                (status_next_write_slot_ + n) % kStatusSlotCount;
            std::uint8_t expected = kStatusSlotReady;
            if (status_slot_state_[candidate].compare_exchange_strong(
                    expected,
                    kStatusSlotWriting,
                    std::memory_order_acq_rel,
                    std::memory_order_acquire)) {
                slot = candidate;
                reserved = true;
                overwrote_ready_slot = true;
                break;
            }
        }
    }

    if (!reserved) {
        return false;
    }

    token.slot = slot;
    token.data = status_slots_[slot].data();
    token.overwrote_ready_slot = overwrote_ready_slot;
    return true;
}

bool MotorStatusChannel::publish_rt(const WriteToken& token)
{
    if (!token.data || token.slot >= kStatusSlotCount) {
        return true;
    }

    status_slot_state_[token.slot].store(
        kStatusSlotReady, std::memory_order_release);
    const int old_latest = status_latest_slot_.exchange(
        static_cast<int>(token.slot),
        std::memory_order_acq_rel);

    bool dropped_unread_frame = token.overwrote_ready_slot;
    if (old_latest >= 0 &&
        static_cast<std::size_t>(old_latest) < kStatusSlotCount &&
        old_latest != static_cast<int>(token.slot)) {
        std::uint8_t expected = kStatusSlotReady;
        if (status_slot_state_[static_cast<std::size_t>(old_latest)]
                .compare_exchange_strong(
                    expected,
                    kStatusSlotEmpty,
                    std::memory_order_acq_rel,
                    std::memory_order_acquire)) {
            dropped_unread_frame = true;
        }
    }

    status_next_write_slot_ = (token.slot + 1) % kStatusSlotCount;
    return dropped_unread_frame;
}

std::vector<MotorStatusSnapshot> MotorStatusChannel::get_status() const
{
    std::lock_guard<std::mutex> lock(status_cache_mutex_);
    return status_cache_;
}

void MotorStatusChannel::set_callback(StatusCallback cb)
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    status_callback_ = std::move(cb);
}

void MotorStatusChannel::start()
{
    bool expected = false;
    if (!publisher_running_.compare_exchange_strong(expected, true)) {
        return;
    }
    publisher_thread_ = std::thread(&MotorStatusChannel::thread_func, this);
}

void MotorStatusChannel::stop()
{
    if (!publisher_running_.exchange(false)) {
        return;
    }
    if (publisher_thread_.joinable()) {
        publisher_thread_.join();
    }
}

void MotorStatusChannel::reset_slots()
{
    for (auto& state : status_slot_state_) {
        state.store(kStatusSlotEmpty, std::memory_order_relaxed);
    }
}

bool MotorStatusChannel::copy_latest_status(
    std::vector<MotorStatusSnapshot>& out)
{
    out.resize(motor_count_);
    const int slot = status_latest_slot_.load(std::memory_order_acquire);
    if (slot < 0 || static_cast<std::size_t>(slot) >= kStatusSlotCount) {
        return false;
    }

    const std::size_t slot_index = static_cast<std::size_t>(slot);
    std::uint8_t expected = kStatusSlotReady;
    if (!status_slot_state_[slot_index].compare_exchange_strong(
            expected,
            kStatusSlotReading,
            std::memory_order_acq_rel,
            std::memory_order_acquire)) {
        return false;
    }

    if (status_latest_slot_.load(std::memory_order_acquire) != slot) {
        status_slot_state_[slot_index].store(
            kStatusSlotEmpty, std::memory_order_release);
        return false;
    }

    const auto& status_slot = status_slots_[slot_index];
    for (std::size_t i = 0; i < motor_count_; ++i) {
        out[i] = status_slot[i];
    }

    status_slot_state_[slot_index].store(
        kStatusSlotEmpty, std::memory_order_release);
    int expected_latest = slot;
    status_latest_slot_.compare_exchange_strong(
        expected_latest,
        -1,
        std::memory_order_acq_rel,
        std::memory_order_acquire);
    return true;
}

void MotorStatusChannel::thread_func()
{
    std::vector<MotorStatusSnapshot> latest(motor_count_);

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

    if (copy_latest_status(latest)) {
        std::lock_guard<std::mutex> lock(status_cache_mutex_);
        status_cache_ = latest;
    }
}

} // namespace myactua
