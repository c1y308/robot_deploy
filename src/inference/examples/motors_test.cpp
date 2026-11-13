#include "config/deploy_config.hpp"
#include "robot/action_processor.hpp"
#include "robot/robot_motor_session.hpp"
#include "tool/tool.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <array>

#ifndef ROBOT_DEPLOY_CONFIG_PATH
#define ROBOT_DEPLOY_CONFIG_PATH ""
#endif

namespace {
std::atomic<bool> g_running{true};


void signal_handler(int) {
    g_running.store(false);
}
}  // namespace


int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::string config_path = ROBOT_DEPLOY_CONFIG_PATH;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            config_path = argv[++i];
        } else {
            std::cerr << "[MOTORS_TEST] usage: motors_test [--config <deploy.yaml>]"
                      << std::endl;
            return -1;
        }
    }

    inference::RobotInterfaceConfig robot_cfg;
    std::string config_error;
    if (!inference::load_deploy_config(config_path, robot_cfg, config_error)) {
        std::cerr << "[MOTORS_TEST] Failed to load deploy config: "
                  << config_error << std::endl;
        return -1;
    }
    inference::RobotMotorSession motors(robot_cfg);

    std::cout << "[MOTORS_TEST] 1/4 initialize()" << std::endl;
    if (!motors.initialize()) {
        std::cerr << "[MOTORS_TEST] initialize failed." << std::endl;
        const auto shutdown_result = motors.shutdown(motors.request_stop());
        if (shutdown_result != inference::ShutdownResult::Confirmed) {
            std::cerr << (shutdown_result == inference::ShutdownResult::ReleasedWithCommLoss
                ? "EtherCAT communication lost; STOP not confirmed; controller released.\n"
                : "Stop not confirmed; RT retained. Destruction will attempt shutdown once and terminate if it fails.\n");
            return 1;
        }
        return -1;
    }

    std::cout << "[MOTORS_TEST] 2/4 restart motors" << std::endl;
    if (!motors.restart()) {
        const auto shutdown_result = motors.shutdown(motors.request_stop());
        if (shutdown_result != inference::ShutdownResult::Confirmed) {
            std::cerr << (shutdown_result == inference::ShutdownResult::ReleasedWithCommLoss
                ? "EtherCAT communication lost; STOP not confirmed; controller released.\n"
                : "Stop not confirmed; RT retained. Destruction will attempt shutdown once and terminate if it fails.\n");
            return 1;
        }
        std::cerr << "[MOTORS_TEST] restart failed." << std::endl;
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "[MOTORS_TEST] 3/4 send zero-position command in rad" << std::endl;
    const std::array<double, motor_base::kMotorNum> zero_target{};
    motor_base::CommandTiming timing;
    timing.produced_at_ns = robot_base::monotonic_now_ns();
    timing.valid_until_ns = timing.produced_at_ns + robot_base::seconds_to_ns(
        robot_cfg.safety.control_command_timeout_ms / 1000.0);
    bool submitted = false;
    if (robot_cfg.motor.control_mode == motor_base::MotorControlMode::POSITION) {
        submitted = motors.submit_position_targets_rad(zero_target, timing);
    } else if (robot_cfg.motor.control_mode == motor_base::MotorControlMode::IMPEDANCE) {
        const auto setpoints = inference::ActionProcessor::make_position_impedance_setpoints(
            zero_target, robot_cfg.motor);
        submitted = motors.submit_impedance_setpoints(setpoints, timing);
    }
    if (!submitted) {
        const auto shutdown_result = motors.shutdown(motors.request_stop());
        if (shutdown_result != inference::ShutdownResult::Confirmed) {
            std::cerr << (shutdown_result == inference::ShutdownResult::ReleasedWithCommLoss
                ? "EtherCAT communication lost; STOP not confirmed; controller released.\n"
                : "Stop not confirmed; RT retained. Destruction will attempt shutdown once and terminate if it fails.\n");
            return 1;
        }
        std::cerr << "[MOTORS_TEST] zero-position command submission failed." << std::endl;
        return -1;
    }
    std::string feedback_error;
    if (!motors.refresh_feedback_for_policy(feedback_error)) {
        std::cerr << "[MOTORS_TEST] " << feedback_error << '\n';
        const auto shutdown_result = motors.shutdown(motors.request_stop());
        if (shutdown_result != inference::ShutdownResult::Confirmed) {
            std::cerr << (shutdown_result == inference::ShutdownResult::ReleasedWithCommLoss
                ? "EtherCAT communication lost; STOP not confirmed; controller released.\n"
                : "Stop not confirmed; RT retained. Destruction will attempt shutdown once and terminate if it fails.\n");
            return 1;
        }
        return -1;
    }
    const auto& state = motors.get_feedback_for_policy();
    std::cout << "[MOTORS_TEST] observed motors=" << state.position_rad.size() << std::endl;

    std::cout << "[MOTORS_TEST] 4/4 monitoring, press Ctrl+C to stop" << std::endl;
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    const auto shutdown_result = motors.shutdown(motors.request_stop());
    if (shutdown_result != inference::ShutdownResult::Confirmed) {
        std::cerr << (shutdown_result == inference::ShutdownResult::ReleasedWithCommLoss
            ? "EtherCAT communication lost; STOP not confirmed; controller released.\n"
            : "Stop not confirmed; RT retained. Destruction will attempt shutdown once and terminate if it fails.\n");
        return 1;
    }
    std::cout << "[MOTORS_TEST] shutdown complete." << std::endl;
    return 0;
}
