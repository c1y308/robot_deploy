#include "robot_interface.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

namespace {
std::atomic<bool> g_running{true};

void signal_handler(int) {
    g_running.store(false);
}
}  // namespace

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    inference::RobotInterfaceConfig cfg;
    inference::RobotInterface robot(cfg);

    std::cout << "[IMU_TEST] Starting IMU only..." << std::endl;
    if (!robot.initial_and_start_imu()) {
        std::cerr << "[IMU_TEST] Failed to start IMU." << std::endl;
        return -1;
    }

    std::cout << "[IMU_TEST] Running. Press Ctrl+C to stop." << std::endl;
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    robot.deinit_imu();
    std::cout << "[IMU_TEST] IMU stopped." << std::endl;
    return 0;
}
