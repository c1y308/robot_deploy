#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "ControlTypes.hpp"

namespace myactua {

class MotorStatusChannel {
public:
    using StatusCallback =
        std::function<void(const std::vector<MotorStatusSnapshot>&)>;

    struct WriteToken {
        std::size_t slot;
        MotorStatusSnapshot* data;
        bool overwrote_ready_slot;

        WriteToken()
            : slot(0), data(nullptr), overwrote_ready_slot(false)
        {
        }
    };

    MotorStatusChannel();
    ~MotorStatusChannel();

    void configure(std::size_t motor_count, int publish_period_ms);

    bool try_begin_write_rt(WriteToken& token);
    bool publish_rt(const WriteToken& token);

    std::vector<MotorStatusSnapshot> get_status() const;
    void set_callback(StatusCallback cb);

    void start();
    void stop();

private:
    static constexpr std::size_t kStatusSlotCount = 2;
    static constexpr std::uint8_t kStatusSlotEmpty = 0;
    static constexpr std::uint8_t kStatusSlotWriting = 1;
    static constexpr std::uint8_t kStatusSlotReady = 2;
    static constexpr std::uint8_t kStatusSlotReading = 3;

    void reset_slots();
    bool copy_latest_status(std::vector<MotorStatusSnapshot>& out);
    void thread_func();

    std::size_t motor_count_{0};
    int publish_period_ms_{1};

    std::array<std::array<MotorStatusSnapshot, kMaxMotorCommandSetpoints>,
               kStatusSlotCount>
        status_slots_{};
    std::array<std::atomic<std::uint8_t>, kStatusSlotCount> status_slot_state_{};
    std::atomic<int> status_latest_slot_{-1};
    std::size_t status_next_write_slot_{0};

    std::vector<MotorStatusSnapshot> status_cache_;
    mutable std::mutex status_cache_mutex_;

    StatusCallback status_callback_;
    mutable std::mutex callback_mutex_;

    std::thread publisher_thread_;
    std::atomic<bool> publisher_running_{false};
};

} // namespace myactua
