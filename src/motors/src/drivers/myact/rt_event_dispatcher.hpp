#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <thread>

#include "ControlTypes.hpp"
#include "realtime_queue.hpp"

namespace myactua {

class RtEventDispatcher {
public:
    using Callback = std::function<void(const RtEvent&)>;

    explicit RtEventDispatcher(std::size_t capacity = 1);
    ~RtEventDispatcher();

    void set_callback(Callback cb);
    void push_rt(const RtEvent& event);

    void start();
    void stop();

private:
    void thread_func();
    void print_default_event(const RtEvent& event) const;

    RtEventQueue queue_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    Callback callback_;
    mutable std::mutex callback_mutex_;
};

} // namespace myactua
