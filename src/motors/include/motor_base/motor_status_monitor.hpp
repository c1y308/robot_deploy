#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace motor_base {

template <typename Snapshot>
class MotorStatusMonitor {
public:
    using StatusProvider = std::function<std::vector<Snapshot>()>;
    using StatusPrinter = std::function<void(
        const std::vector<Snapshot>&,
        const std::vector<int>&)>;

    MotorStatusMonitor() = default;

    explicit MotorStatusMonitor(StatusProvider provider, StatusPrinter printer = {})
        : status_provider_(std::move(provider)),
          status_printer_(std::move(printer))
    {
    }

    ~MotorStatusMonitor()
    {
        stop();
    }

    void set_status_provider(StatusProvider provider)
    {
        std::lock_guard<std::mutex> lock(provider_mutex_);
        status_provider_ = std::move(provider);
    }

    void set_status_printer(StatusPrinter printer)
    {
        std::lock_guard<std::mutex> lock(printer_mutex_);
        status_printer_ = std::move(printer);
    }

    bool set_print_info(const std::vector<int>& motor_indices)
    {
        std::lock_guard<std::mutex> lock(print_mutex_);
        print_motor_ids_ = motor_indices;
        return !print_motor_ids_.empty();
    }

    bool has_print_motor_ids() const
    {
        std::lock_guard<std::mutex> lock(print_mutex_);
        return !print_motor_ids_.empty();
    }

    void start()
    {
        bool expected = false;
        if (!running_.compare_exchange_strong(expected, true)) {
            return;
        }
        thread_ = std::thread(&MotorStatusMonitor::thread_func, this);
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
        const StatusProvider provider = get_status_provider();
        const StatusPrinter printer = get_status_printer();
        if (!provider || !printer) {
            return;
        }

        const std::vector<int> print_motor_ids = get_print_motor_ids();
        if (print_motor_ids.empty()) {
            return;
        }

        printer(provider(), print_motor_ids);
    }

private:
    std::vector<int> get_print_motor_ids() const
    {
        std::lock_guard<std::mutex> lock(print_mutex_);
        return print_motor_ids_;
    }

    StatusProvider get_status_provider() const
    {
        std::lock_guard<std::mutex> lock(provider_mutex_);
        return status_provider_;
    }

    StatusPrinter get_status_printer() const
    {
        std::lock_guard<std::mutex> lock(printer_mutex_);
        return status_printer_;
    }

    void thread_func()
    {
        using Clock = std::chrono::steady_clock;
        constexpr auto period = std::chrono::milliseconds(200);
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

    StatusProvider status_provider_;
    mutable std::mutex provider_mutex_;

    StatusPrinter status_printer_;
    mutable std::mutex printer_mutex_;

    std::vector<int> print_motor_ids_;
    mutable std::mutex print_mutex_;

    std::thread thread_;
    std::atomic<bool> running_{false};
};

} // namespace motor_base
