#include "motor_base/rt_event_dispatcher.hpp"

#include <algorithm>
#include <chrono>
#include <utility>

namespace motor_base {

RtEventDispatcher::RtEventDispatcher(
    std::size_t capacity,
    EventPrinter fallback_printer)
    : buffer_(std::max<std::size_t>(1, capacity)),
      capacity_(buffer_.size()),
      fallback_printer_(std::move(fallback_printer))
{
}


void RtEventDispatcher::set_callback(Callback cb)
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback_ = std::move(cb);
}


void RtEventDispatcher::set_fallback_printer(EventPrinter printer)
{
    std::lock_guard<std::mutex> lock(fallback_printer_mutex_);
    fallback_printer_ = std::move(printer);
}


void RtEventDispatcher::start()
{
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return;
    }
    thread_ = std::thread(&RtEventDispatcher::thread_func, this);
}


void RtEventDispatcher::thread_func()
{
    RtEvent event;
    while (running_) {
        bool handled_any = false;
        while (pop(event)) {
            handled_any = true;
            handle_event(event);
        }

        if (!handled_any) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    while (pop(event)) {
        handle_event(event);
    }
}


bool RtEventDispatcher::push(const RtEvent& event)
{
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t tail = tail_.load(std::memory_order_acquire);
    if (head - tail >= capacity_) {
        return false;
    }

    buffer_[head % capacity_] = event;
    head_.store(head + 1, std::memory_order_release);
    return true;
}

bool RtEventDispatcher::pop(RtEvent& event)
{
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    const std::size_t head = head_.load(std::memory_order_acquire);
    if (tail == head) {
        return false;
    }

    event = buffer_[tail % capacity_];
    tail_.store(tail + 1, std::memory_order_release);
    return true;
}


void RtEventDispatcher::handle_event(const RtEvent& event)
{
    const Callback cb = get_callback();
    if (cb) {
        cb(event);
        return;
    }

    const EventPrinter printer = get_fallback_printer();
    if (printer) {
        printer(event);
    }
}

RtEventDispatcher::Callback RtEventDispatcher::get_callback() const
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    return callback_;
}

RtEventDispatcher::EventPrinter RtEventDispatcher::get_fallback_printer() const
{
    std::lock_guard<std::mutex> lock(fallback_printer_mutex_);
    return fallback_printer_;
}


RtEventDispatcher::~RtEventDispatcher()
{
    stop();
}

void RtEventDispatcher::stop()
{
    if (!running_.exchange(false)) {
        return;
    }
    if (thread_.joinable()) {
        thread_.join();
    }
}

} // namespace motor_base
