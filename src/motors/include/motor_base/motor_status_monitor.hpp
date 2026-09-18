#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <chrono>
#include <functional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "motor_base/command_types.hpp"
#include "spsc_latest_channel/spsc_latest_channel.hpp"
#include "tool/thread_runtime.hpp"

namespace motor_base {

template <typename Snapshot>
class MotorStatusMonitor {
public:
    using StatusPrinter = std::function<void(
        const std::vector<Snapshot>&,
        const std::vector<int>&)>;

    MotorStatusMonitor() = default;

    ~MotorStatusMonitor()
    {
        stop();
    }

    void set_status_printer(StatusPrinter printer)
    {
        status_printer_ = std::move(printer);
    }

    // printer 和打印轴列表均在 start() 前配置。
    void set_print_info(const std::vector<int>& motor_indices)
    {
        print_motor_ids_ = motor_indices;
    }

    bool has_print_motor_ids() const
    {
        return !print_motor_ids_.empty();
    }

    void configure(std::size_t motor_count, robot_base::ThreadRuntimeOptions options)
    {
        thread_options_ = std::move(options);
        latest_status_.resize(motor_count);
        latest_frame_.reset_empty();
    }

    // RT 为唯一生产者；motor_mon 为唯一消费者，打印和缓存均留在 monitor 线程。
    Snapshot* acquire_write_slot() noexcept
    {
        return latest_frame_.acquire_write_slot()->data();
    }

    void publish_written() noexcept
    {
        latest_frame_.publish_written();
    }

    bool start()
    {
        bool expected = false;
        if (!running_.compare_exchange_strong(expected, true)) {
            return true;
        }
        if (!robot_base::start_configured_thread(
                thread_, "motor_mon", thread_options_,
                [this] { thread_func(); }, last_start_error_)) {
            running_.store(false);
            return false;
        }
        return true;
    }

    void stop()
    {
        if (!running_.exchange(false)) {
            return;
        }
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    void print_once()
    {
        if (!status_printer_ || print_motor_ids_.empty()) {
            return;
        }
        std::array<Snapshot, kMaxMotors> latest;
        if (latest_frame_.try_consume_latest(latest)) {
            std::copy_n(latest.begin(), latest_status_.size(), latest_status_.begin());
        }
        status_printer_(latest_status_, print_motor_ids_);
    }

    const std::string& last_start_error() const noexcept
    {
        return last_start_error_;
    }

private:
    void thread_func()
    {
        using Clock = std::chrono::steady_clock;
        constexpr auto period = std::chrono::milliseconds(20);
        auto next_tick = Clock::now();

        while (running_) {
            if (has_print_motor_ids()) {
                print_once();
            }

            next_tick += period;
            std::this_thread::sleep_until(next_tick);
            if (Clock::now() > next_tick + period) {
                next_tick = Clock::now();
            }
        }
    }

    robot_base::SpscLatestChannel<std::array<Snapshot, kMaxMotors>> latest_frame_;
    std::vector<Snapshot> latest_status_; // monitor 线程独占，无锁缓存用于重复打印 latest 值。

    StatusPrinter status_printer_;

    std::vector<int> print_motor_ids_;

    std::thread thread_;
    std::atomic<bool> running_{false};
    robot_base::ThreadRuntimeOptions thread_options_;
    std::string last_start_error_;
};

} // namespace motor_base
