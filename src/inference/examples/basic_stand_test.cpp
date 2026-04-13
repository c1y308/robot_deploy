#include "robot_interface.hpp"

#include <atomic>
#include <csignal>
#include <chrono>
#include <iostream>
#include <thread>

namespace {
std::atomic<bool> g_running{true};

void signal_handler(int) {
    g_running.store(false);
}

void print_vec(const std::string& name, const std::vector<double>& v, int max_n = 6) {
    std::cout << name << ": [";
    for (int i = 0; i < static_cast<int>(v.size()) && i < max_n; ++i) {
        std::cout << v[i];
        if (i + 1 < static_cast<int>(v.size()) && i + 1 < max_n) {
            std::cout << ", ";
        }
    }
    if (static_cast<int>(v.size()) > max_n) {
        std::cout << ", ...";
    }
    std::cout << "]\n";
}
}  // namespace

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    inference::RobotInterfaceConfig cfg;
    cfg.num_motors = 12;
    cfg.ethercat_ifname = "enp8s0";
    cfg.imu_device = "/dev/ttyUSB0";
    cfg.imu_baudrate = 921600;
    cfg.stand_pose_rad = std::vector<double>(cfg.num_motors, 0.0);

    inference::RobotInterface robot(cfg);

    std::cout << "[TEST] 1/4 initial_and_start_motors()\n";
    if (!robot.initial_and_start_motors()) {
        std::cerr << "[TEST] initial_and_start_motors failed.\n";
        return -1;
    }

    std::cout << "[TEST] 2/4 read joints + imu\n";
    print_vec("q(rad)", robot.get_joint_q());
    print_vec("dq(rad/s)", robot.get_joint_vel());
    print_vec("tau(raw)", robot.get_joint_tau());
    const auto quat = robot.get_quat();
    const auto w = robot.get_ang_vel();
    std::cout << "quat[wxyz]: [" << quat[0] << ", " << quat[1] << ", "
              << quat[2] << ", " << quat[3] << "]\n";
    std::cout << "ang_vel(rad/s): [" << w[0] << ", " << w[1] << ", "
              << w[2] << "]\n";

    std::cout << "[TEST] 3/4 reset_joints() -> stand/safe pose\n";
    if (!robot.reset_joints()) {
        std::cerr << "[TEST] reset_joints failed.\n";
        robot.deinit_motors();
        return -1;
    }

    std::cout << "[TEST] 4/4 hold pose, Ctrl+C to exit\n";
    while (g_running.load()) {
        const auto q = robot.get_joint_q();
        const auto ang = robot.get_ang_vel();
        if (!q.empty()) {
            std::cout << "q0(rad)=" << q[0]
                      << ", imu_wz(rad/s)=" << ang[2] << "\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    robot.deinit_motors();
    std::cout << "[TEST] shutdown complete.\n";
    return 0;
}
