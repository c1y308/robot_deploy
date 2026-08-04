#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "ControlTypes.hpp"

namespace myactua {

class MotorStatusMonitor {
public:
    using StatusProvider = std::function<std::vector<MotorStatusSnapshot>()>;

    MotorStatusMonitor() = default;
    explicit MotorStatusMonitor(StatusProvider provider);
    ~MotorStatusMonitor();

    void set_status_provider(StatusProvider provider);
    bool set_print_info(const std::vector<int>& slave_indices, int motor_count);
    bool has_print_motor_ids() const;

    void start();
    void stop();
    void print_once();

private:
    std::vector<int> get_print_motor_ids() const;
    StatusProvider get_status_provider() const;
    void thread_func();

    StatusProvider status_provider_;
    mutable std::mutex provider_mutex_;

    std::vector<int> print_motor_ids_;
    mutable std::mutex print_mutex_;

    std::thread thread_;
    std::atomic<bool> running_{false};
};

} // namespace myactua
