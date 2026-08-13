#include "robot/robot_imu_session.hpp"

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

    inference::ImuConfig cfg;
    inference::RobotImuSession imu(cfg);

    std::cout << "[IMU_TEST] Starting IMU only..." << std::endl;
    if (!imu.initialize_and_start()) {
        std::cerr << "[IMU_TEST] Failed to start IMU." << std::endl;
        return -1;
    }

    std::cout << "[IMU_TEST] Running. Press Ctrl+C to stop." << std::endl;
    while (g_running.load()) {
        const auto state = imu.get_state();
        std::cout << std::fixed << std::setprecision(6)
                  << "[IMU_TEST] euler[roll,pitch,heading]=["
                  << state.euler[0] << ", "
                  << state.euler[1] << ", "
                  << state.euler[2] << "] rad, body_ang_vel[roll,pitch,heading]=["
                  << state.body_ang_vel[0] << ", "
                  << state.body_ang_vel[1] << ", "
                  << state.body_ang_vel[2] << "] rad/s, projected_gravity=["
                  << state.projected_gravity[0] << ", "
                  << state.projected_gravity[1] << ", "
                  << state.projected_gravity[2] << "] m/s^2\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    imu.deinitialize();
    std::cout << "[IMU_TEST] IMU stopped." << std::endl;
    return 0;
}
