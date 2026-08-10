#include "driver/myact/myact_debug_printers.hpp"

#include <algorithm>
#include <cstdio>
#include <iostream>

#include "driver/myact/motor_units.hpp"
#include "motor_base/rt_event_dispatcher.hpp"

namespace myactua {

namespace {

namespace mb = motor_base;

const char* motor_step_name(MyactMotorStep step)
{
    switch (step) {
        case MyactMotorStep::IDLE: return "IDLE";
        case MyactMotorStep::ENABLING: return "ENABLING";
        case MyactMotorStep::RUNNING: return "RUNNING";
        case MyactMotorStep::STOPPED: return "STOPPED";
        case MyactMotorStep::FAULT: return "FAULT";
        case MyactMotorStep::MODE_SWITCHING: return "MODE_SWITCHING";
    }
    return "UNKNOWN";
}

const char* mode_switch_step_name(MyactModeSwitchStep step)
{
    switch (step) {
        case MyactModeSwitchStep::IDLE: return "IDLE";
        case MyactModeSwitchStep::SET_MODE: return "SET_MODE";
        case MyactModeSwitchStep::CLEAR: return "CLEAR";
        case MyactModeSwitchStep::DISABLE: return "DISABLE";
        case MyactModeSwitchStep::ENABLE: return "ENABLE";
        case MyactModeSwitchStep::OPERATING: return "OPERATING";
        case MyactModeSwitchStep::DONE: return "DONE";
    }
    return "N/A";
}

const char* control_mode_name(MyactControlMode mode)
{
    switch (mode) {
        case MyactControlMode::NONE: return "NONE";
        case MyactControlMode::PVT: return "PVT";
        case MyactControlMode::CSP: return "CSP";
        case MyactControlMode::CSV: return "CSV";
        case MyactControlMode::CST: return "CST";
    }
    return "UNKNOWN";
}

const char* discrete_command_name(mb::DiscreteCommandType type)
{
    switch (type) {
        case mb::DiscreteCommandType::STOP: return "STOP";
        case mb::DiscreteCommandType::RESTART: return "RESTART";
        case mb::DiscreteCommandType::SET_MODE: return "SET_MODE";
    }
    return "UNKNOWN";
}

} // namespace

void print_myact_status_table(
    const std::vector<MyactDiagnosticsSnapshot>& status,
    const std::vector<int>& motor_indices)
{
    printf("\033[2J\033[H");
    const bool print_all =
        std::find(motor_indices.begin(), motor_indices.end(), -1) !=
        motor_indices.end();

    printf("\033[1;36m============================ MOTOR REAL-TIME MONITOR ============================\033[0m\n");
    printf("%-6s | %-10s | %-16s | %-22s | %-8s | %-8s | %-7s | %-16s | %-14s | %-14s | %-14s\n",
        "ID", "OFFLINE_CNT", "STEP", "MODE_SWITCH_STEP", "RX_MODE", "TX_MODE", "DES_EN",
        "TX_TARGET", "RX_TQ_PCT", "RX_POS_DEG", "TAR_ERR_DEG");
    printf("------------------------------------------------------------------------------------------------------------------------------------------------------\n");

    for (const auto& m : status) {
        if (!print_all &&
            std::find(motor_indices.begin(), motor_indices.end(), m.motor_index) ==
            motor_indices.end()) {
            continue;
        }
        const char* color_code = "\033[32m";
        if (m.step == MyactMotorStep::FAULT) color_code = "\033[31m";
        if (m.step == MyactMotorStep::STOPPED) color_code = "\033[35m";
        if (m.step == MyactMotorStep::MODE_SWITCHING) color_code = "\033[33m";

        const double rx_pos_rad = raw_pos_to_rad(m.position);
        const double rx_pos_deg = rad_to_deg(rx_pos_rad);
        const double target_pos_deg =
            static_cast<double>(m.command_position) *
            kRawPosToRad *
            kRadToDeg;
        const double target_error_deg = target_pos_deg - rx_pos_deg;
        char tx_target_info[64] = {};
        switch (m.tx_mode) {
            case MyactControlMode::PVT:
                std::snprintf(tx_target_info, sizeof(tx_target_info), "%.3f",
                    target_pos_deg);
                break;
            case MyactControlMode::CSP:
                std::snprintf(tx_target_info, sizeof(tx_target_info), "%.3f",
                    target_pos_deg);
                break;
            case MyactControlMode::CSV:
                std::snprintf(tx_target_info, sizeof(tx_target_info), "%.3f rpm",
                    static_cast<double>(m.command_velocity) * kRawVelToRpm);
                break;
            case MyactControlMode::CST:
                std::snprintf(tx_target_info, sizeof(tx_target_info), "%d raw",
                    static_cast<int>(m.command_torque));
                break;
            default:
                std::snprintf(tx_target_info, sizeof(tx_target_info), "N/A");
                break;
        }

        printf("M %-4d | %-10u | %s%-16s\033[0m | %s%-22s\033[0m | %-8s | %-8s | %-7s | %-16s | %-13.1f%% | %-14.3f | %-14.3f\n",
            m.motor_index,
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

void print_myact_rt_event(const mb::RtEvent& event)
{
    switch (event.type) {
        case mb::RtEventType::DISCRETE_COMMAND_FAILED:
            std::cerr << "[MYACTUA] discrete command failed on motor "
                      << event.motor_index
                      << ", type=" << discrete_command_name(event.command_type)
                      << ", reason=" << event.reason
                      << ", retry=" << event.value << "\n";
            break;

        case mb::RtEventType::DISCRETE_QUEUE_FULL:
            std::cerr << "[MYACTUA] discrete command queue full on motor "
                      << event.motor_index
                      << ", type=" << discrete_command_name(event.command_type)
                      << "\n";
            break;

        case mb::RtEventType::STATUS_FRAME_OVERWRITTEN:
            std::cerr << "[MYACTUA] status publisher lagged; overwritten frames="
                      << event.value << "\n";
            break;

        case mb::RtEventType::BUS_DIAG_SAMPLE:
            std::cout << "[BUS_DIAG] cycle=" << event.tick
                      << " wc=" << event.value
                      << " wc_state=" << event.reason << "\n";
            break;

        case mb::RtEventType::BUS_CYCLE_NOT_COMPLETE:
            std::cerr << "[BUS_DIAG] cycle not complete, cycle=" << event.tick
                      << " wc=" << event.value
                      << " wc_state=" << event.reason << "\n";
            break;
    }
}

} // namespace myactua
