#include "motor_status_monitor.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <utility>

#include "motor_units.hpp"

namespace myactua {

namespace {

const char* motor_step_name(MotorStep step)
{
    switch (step) {
        case MotorStep::IDLE: return "IDLE";
        case MotorStep::ENABLING: return "ENABLING";
        case MotorStep::RUNNING: return "RUNNING";
        case MotorStep::STOPPED: return "STOPPED";
        case MotorStep::FAULT: return "FAULT";
        case MotorStep::MODE_SWITCHING: return "MODE_SWITCHING";
    }
    return "UNKNOWN";
}

const char* mode_switch_step_name(ModeSwitchStep step)
{
    switch (step) {
        case ModeSwitchStep::IDLE: return "IDLE";
        case ModeSwitchStep::SET_MODE: return "SET_MODE";
        case ModeSwitchStep::CLEAR: return "CLEAR";
        case ModeSwitchStep::DISABLE: return "DISABLE";
        case ModeSwitchStep::ENABLE: return "ENABLE";
        case ModeSwitchStep::OPERATING: return "OPERATING";
        case ModeSwitchStep::DONE: return "DONE";
    }
    return "N/A";
}

const char* control_mode_name(ControlMode mode)
{
    switch (mode) {
        case ControlMode::NONE: return "NONE";
        case ControlMode::PVT: return "PVT";
        case ControlMode::CSP: return "CSP";
        case ControlMode::CSV: return "CSV";
        case ControlMode::CST: return "CST";
    }
    return "UNKNOWN";
}

} // namespace

MotorStatusMonitor::MotorStatusMonitor(StatusProvider provider)
    : status_provider_(std::move(provider))
{
}

MotorStatusMonitor::~MotorStatusMonitor()
{
    stop();
}

void MotorStatusMonitor::set_status_provider(StatusProvider provider)
{
    std::lock_guard<std::mutex> lock(provider_mutex_);
    status_provider_ = std::move(provider);
}

bool MotorStatusMonitor::set_print_info(
    const std::vector<int>& slave_indices,
    int motor_count)
{
    std::vector<int> normalized_ids;
    const bool print_all =
        std::find(slave_indices.begin(), slave_indices.end(), -1) !=
        slave_indices.end();

    if (print_all) {
        normalized_ids.reserve(static_cast<std::size_t>(std::max(0, motor_count)));
        for (int i = 0; i < motor_count; ++i) {
            normalized_ids.push_back(i);
        }
    } else {
        for (int slave_index : slave_indices) {
            if (slave_index < 0 || slave_index >= motor_count) {
                continue;
            }
            if (std::find(
                    normalized_ids.begin(),
                    normalized_ids.end(),
                    slave_index) == normalized_ids.end()) {
                normalized_ids.push_back(slave_index);
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(print_mutex_);
        print_motor_ids_ = normalized_ids;
    }
    return !normalized_ids.empty();
}

bool MotorStatusMonitor::has_print_motor_ids() const
{
    std::lock_guard<std::mutex> lock(print_mutex_);
    return !print_motor_ids_.empty();
}

void MotorStatusMonitor::start()
{
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return;
    }
    thread_ = std::thread(&MotorStatusMonitor::thread_func, this);
}

void MotorStatusMonitor::stop()
{
    if (!running_.exchange(false)) {
        return;
    }
    if (thread_.joinable()) {
        thread_.join();
    }
}

void MotorStatusMonitor::print_once()
{
    const StatusProvider provider = get_status_provider();
    if (!provider) {
        return;
    }

    const std::vector<MotorStatusSnapshot> status = provider();
    const std::vector<int> print_motor_ids = get_print_motor_ids();
    if (print_motor_ids.empty()) {
        return;
    }

    printf("\033[2J\033[H");

    printf("\033[1;36m============================ MOTOR REAL-TIME MONITOR ============================\033[0m\n");
    printf("%-6s | %-10s | %-16s | %-22s | %-8s | %-8s | %-7s | %-16s | %-14s | %-14s | %-14s\n",
        "ID", "OFFLINE_CNT", "STEP", "MODE_SWITCH_STEP", "RX_MODE", "TX_MODE", "DES_EN",
        "TX_TARGET", "RX_TQ_PCT", "RX_POS_DEG", "TAR_ERR_DEG");
    printf("------------------------------------------------------------------------------------------------------------------------------------------------------\n");

    for (const auto& m : status) {
        if (std::find(print_motor_ids.begin(), print_motor_ids.end(), m.slave_index) == print_motor_ids.end()) {
            continue;
        }
        const char* color_code = "\033[32m";
        if (m.step == MotorStep::FAULT) color_code = "\033[31m";
        if (m.step == MotorStep::STOPPED) color_code = "\033[35m";
        if (m.step == MotorStep::MODE_SWITCHING) color_code = "\033[33m";

        const double rx_pos_rad = raw_pos_to_rad(m.position);
        const double rx_pos_deg = rad_to_deg(rx_pos_rad);
        const double target_pos_deg =
            static_cast<double>(m.tx_target_pos) *
            kRawPosToRad *
            kRadToDeg;
        const double target_error_deg = target_pos_deg - rx_pos_deg;
        char tx_target_info[64] = {};
        switch (m.tx_mode) {
            case ControlMode::PVT:
                std::snprintf(tx_target_info, sizeof(tx_target_info), "%.3f",
                    target_pos_deg);
                break;
            case ControlMode::CSP:
                std::snprintf(tx_target_info, sizeof(tx_target_info), "%.3f",
                    target_pos_deg);
                break;
            case ControlMode::CSV:
                std::snprintf(tx_target_info, sizeof(tx_target_info), "%.3f rpm",
                    static_cast<double>(m.tx_target_vel) * kRawVelToRpm);
                break;
            case ControlMode::CST:
                std::snprintf(tx_target_info, sizeof(tx_target_info), "%d raw",
                    static_cast<int>(m.tx_target_torque));
                break;
            default:
                std::snprintf(tx_target_info, sizeof(tx_target_info), "N/A");
                break;
        }

        printf("M %-4d | %-10u | %s%-16s\033[0m | %s%-22s\033[0m | %-8s | %-8s | %-7s | %-16s | %-13.1f%% | %-14.3f | %-14.3f\n",
            m.slave_index,
            static_cast<unsigned int>(m.offline_count),
            color_code,
            motor_step_name(m.step),
            color_code,
            mode_switch_step_name(m.mode_switch_step),
            control_mode_name(m.op_mode),
            control_mode_name(m.tx_mode),
            m.desired_enabled ? "Y" : "N",
            tx_target_info,
            m.torque / 10.0,
            rx_pos_deg,
            target_error_deg);
    }
    printf("\033[1;36m=================================================================================\033[0m\n");

    fflush(stdout);
}

std::vector<int> MotorStatusMonitor::get_print_motor_ids() const
{
    std::lock_guard<std::mutex> lock(print_mutex_);
    return print_motor_ids_;
}

MotorStatusMonitor::StatusProvider MotorStatusMonitor::get_status_provider() const
{
    std::lock_guard<std::mutex> lock(provider_mutex_);
    return status_provider_;
}

void MotorStatusMonitor::thread_func()
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

} // namespace myactua
