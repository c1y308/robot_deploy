#include "robot_interface.hpp"

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

    inference::RobotInterfaceConfig cfg;
    cfg.num_motors = 1;  // 当前电机个数为 1，其余配置使用默认值

    inference::RobotInterface robot(cfg);

    std::cout << "[MOTORS_TEST] 1/3 initial_and_start_motors()" << std::endl;
    if (!robot.initial_and_start_motors()) {
        std::cerr << "[MOTORS_TEST] initial_and_start_motors failed." << std::endl;
        return -1;
    }

    // std::cout << "[MOTORS_TEST] 2/3 send zero command to motor 0" << std::endl;
    // if (!robot.apply_action(std::vector<double>{0.0})) {
    //     std::cerr << "[MOTORS_TEST] apply_action failed." << std::endl;
    //     robot.deinit_motors();
    //     return -1;
    // }

    std::cout << "[MOTORS_TEST] 3/3 monitoring, press Ctrl+C to stop" << std::endl;
    while (g_running.load()) {
        const auto q    = robot.get_joint_q();
        const auto dq   = robot.get_joint_vel();
        const auto tau  = robot.get_joint_tau();

        if (!q.empty() && !dq.empty() && !tau.empty()) {
            std::cout << "motor0: q(rad)="  << q[0]
                      << ", dq(rad/s)="     << dq[0]
                      << ", tau(raw)="      << tau[0] << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    robot.deinit_motors();
    std::cout << "[MOTORS_TEST] shutdown complete." << std::endl;
    return 0;
}
