#include "robot_interface.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
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
        const auto euler = robot.get_euler();
        const auto body_ang_vel = robot.get_body_ang_vel();

        std::cout << std::fixed << std::setprecision(6)
                  << "[IMU_TEST] euler[roll,pitch,heading]=["
                  << euler[0] << ", "
                  << euler[1] << ", "
                  << euler[2] << "] rad, body_ang_vel[roll,pitch,heading]=["
                  << body_ang_vel[0] << ", "
                  << body_ang_vel[1] << ", "
                  << body_ang_vel[2] << "] rad/s\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    robot.deinit_imu();
    std::cout << "[IMU_TEST] IMU stopped." << std::endl;
    return 0;
}
