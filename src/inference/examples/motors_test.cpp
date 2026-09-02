#include "config/deploy_config.hpp"
#include "robot/robot_motor_session.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

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
    std::string ifname_override;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--device") == 0 && i + 1 < argc) {
            ifname_override = argv[++i];
        } else if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            config_path = argv[++i];
        } else {
            std::cerr << "[MOTORS_TEST] usage: motors_test [--config <deploy.yaml>] "
                         "[--device <ethercat ifname>]" << std::endl;
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
    inference::MotorConfig cfg = robot_cfg.motor;
    if (!ifname_override.empty()) {
        cfg.ethercat_ifname = ifname_override;
    }

    inference::RobotMotorSession motors(cfg);

    std::cout << "[MOTORS_TEST] 1/4 initialize()" << std::endl;
    if (!motors.initialize()) {
        std::cerr << "[MOTORS_TEST] initialize failed." << std::endl;
        return -1;
    }

    std::cout << "[MOTORS_TEST] 2/4 restart motors" << std::endl;
    if (!motors.restart(-1)) {
        std::cerr << "[MOTORS_TEST] restart failed." << std::endl;
        motors.deinitialize();
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "[MOTORS_TEST] 3/4 send zero-position command in rad" << std::endl;
    if (!motors.apply_targets_rad(std::vector<double>(cfg.num_motors, 0.0))) {
        std::cerr << "[MOTORS_TEST] apply_targets_rad failed." << std::endl;
        motors.deinitialize();
        return -1;
    }
    const auto state = motors.get_motor_snapshot();
    std::cout << "[MOTORS_TEST] observed motors=" << state.position_rad.size() << std::endl;

    std::cout << "[MOTORS_TEST] 4/4 monitoring, press Ctrl+C to stop" << std::endl;
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    motors.deinitialize();
    std::cout << "[MOTORS_TEST] shutdown complete." << std::endl;
    return 0;
}
