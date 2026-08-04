#include "rt_event_dispatcher.hpp"

#include <chrono>
#include <iostream>
#include <utility>

namespace myactua {

namespace {

const char* discrete_command_name(DiscreteCommandType type)
{
    switch (type) {
        case DiscreteCommandType::STOP: return "STOP";
        case DiscreteCommandType::RESTART: return "RESTART";
        case DiscreteCommandType::SET_MODE: return "SET_MODE";
    }
    return "UNKNOWN";
}

} // namespace

RtEventDispatcher::RtEventDispatcher(std::size_t capacity)
    : queue_(capacity)
{
}

RtEventDispatcher::~RtEventDispatcher()
{
    stop();
}

void RtEventDispatcher::set_callback(Callback cb)
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback_ = std::move(cb);
}

void RtEventDispatcher::push_rt(const RtEvent& event)
{
    queue_.try_push_rt(event);
}

void RtEventDispatcher::start()
{
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return;
    }
    thread_ = std::thread(&RtEventDispatcher::thread_func, this);
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

void RtEventDispatcher::thread_func()
{
    RtEvent event;
    while (running_) {
        bool handled_any = false;
        while (queue_.try_pop(event)) {
            handled_any = true;
            Callback cb;
            {
                std::lock_guard<std::mutex> lock(callback_mutex_);
                cb = callback_;
            }
            if (cb) {
                cb(event);
            } else {
                print_default_event(event);
            }
        }

        if (!handled_any) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    while (queue_.try_pop(event)) {
        Callback cb;
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            cb = callback_;
        }
        if (cb) {
            cb(event);
        } else {
            print_default_event(event);
        }
    }
}

void RtEventDispatcher::print_default_event(const RtEvent& event) const
{
    switch (event.type) {
        case RtEventType::DISCRETE_COMMAND_FAILED:
            std::cerr << "[MYACTUA] discrete command failed on motor "
                      << event.motor_index
                      << ", type=" << discrete_command_name(event.command_type)
                      << ", reason=" << event.reason
                      << ", retry=" << event.value << "\n";
            break;

        case RtEventType::DISCRETE_QUEUE_FULL:
            std::cerr << "[MYACTUA] discrete command queue full on motor "
                      << event.motor_index
                      << ", type=" << discrete_command_name(event.command_type)
                      << "\n";
            break;

        case RtEventType::STATUS_FRAME_OVERWRITTEN:
            std::cerr << "[MYACTUA] status publisher lagged; overwritten frames="
                      << event.value << "\n";
            break;

        case RtEventType::ECAT_DIAG_SAMPLE:
            std::cout << "[ECAT_DIAG] cycle=" << event.tick
                      << " wc=" << event.value
                      << " wc_state=" << event.reason << "\n";
            break;

        case RtEventType::ECAT_DOMAIN_NOT_COMPLETE:
            std::cerr << "[ECAT_DIAG] domain not complete, cycle=" << event.tick
                      << " wc=" << event.value
                      << " wc_state=" << event.reason << "\n";
            break;
    }
}

} // namespace myactua
