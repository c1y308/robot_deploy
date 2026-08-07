#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "motor_base/CommandTypes.hpp"

namespace motor_base {

enum class RtEventType {
    DISCRETE_COMMAND_FAILED,
    DISCRETE_QUEUE_FULL,
    STATUS_FRAME_OVERWRITTEN,
    BUS_DIAG_SAMPLE,
    BUS_CYCLE_NOT_COMPLETE
};

struct RtEvent {
    RtEventType type;
    uint64_t tick;
    int motor_index;
    DiscreteCommandType command_type;
    int reason;
    uint32_t value;

    RtEvent()
        : type(RtEventType::DISCRETE_COMMAND_FAILED),
          tick(0),
          motor_index(-1),
          command_type(DiscreteCommandType::STOP),
          reason(0),
          value(0) {}
};

class RtEventDispatcher {
public:
    using Callback = std::function<void(const RtEvent&)>;
    using EventPrinter = std::function<void(const RtEvent&)>;

    explicit RtEventDispatcher(
        std::size_t capacity = 1,
        EventPrinter fallback_printer = {});
    ~RtEventDispatcher();

    void set_callback(Callback cb);
    void set_fallback_printer(EventPrinter printer);
    void push_rt(const RtEvent& event);

    void start();
    void stop();

private:
    bool try_push_rt(const RtEvent& event);
    bool try_pop(RtEvent& event);
    Callback get_callback() const;
    EventPrinter get_fallback_printer() const;
    void handle_event(const RtEvent& event);
    void thread_func();

    std::vector<RtEvent> buffer_;
    std::size_t capacity_;
    std::atomic<std::size_t> head_{0};
    std::atomic<std::size_t> tail_{0};

    std::thread thread_;
    std::atomic<bool> running_{false};

    Callback callback_;
    mutable std::mutex callback_mutex_;

    EventPrinter fallback_printer_;
    mutable std::mutex fallback_printer_mutex_;
};

} // namespace motor_base
