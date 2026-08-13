#include "robot/robot_motor_session.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <vector>

namespace {
std::atomic<bool> g_running{true};


void signal_handler(int) {
    g_running.store(false);
}
}  // namespace


int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    inference::MotorConfig cfg;

    inference::RobotMotorSession motors(cfg);

    std::cout << "[MOTORS_TEST] 1/4 initialize_and_start()" << std::endl;
    if (!motors.initialize_and_start()) {
        std::cerr << "[MOTORS_TEST] initialize_and_start failed." << std::endl;
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
    const auto state = motors.get_motor_state();
    std::cout << "[MOTORS_TEST] observed motors=" << state.position_rad.size() << std::endl;

    std::cout << "[MOTORS_TEST] 4/4 monitoring, press Ctrl+C to stop" << std::endl;
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    motors.deinitialize();
    std::cout << "[MOTORS_TEST] shutdown complete." << std::endl;
    return 0;
}
