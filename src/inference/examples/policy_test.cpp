#include "robot_interface.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr const char* kPolicyModelPath = "../model/policy.pt";
constexpr const char* kEthercatIfname = "enp8s0";
constexpr const char* kImuDevice = "/dev/ttyUSB0";
constexpr int kImuBaudrate = 921600;

constexpr double kPolicyHz = 50.0;
constexpr double kPolicyPeriodSec = 1.0 / kPolicyHz;
constexpr double kCommandVx = 0.0;
constexpr double kCommandVy = 0.0;
constexpr double kCommandYawRate = 0.0;

std::atomic<bool> g_stop_requested{false};

void signal_handler(int)
{
    g_stop_requested.store(true);
}

bool file_readable(const std::string& path)
{
    std::ifstream file(path, std::ios::binary);
    return file.good();
}

inference::RobotInterfaceConfig make_robot_config()
{
    inference::RobotInterfaceConfig cfg;
    cfg.num_motors = 12;
    cfg.ethercat_ifname = kEthercatIfname;
    cfg.imu_device      = kImuDevice;
    cfg.imu_baudrate    = kImuBaudrate;

    cfg.policy_model_path = kPolicyModelPath;
    cfg.model_to_motor_index = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

    cfg.action_clip = 18.0;
    cfg.action_scale = 0.25;
    cfg.policy_cycle_time_s = 0.64;

    cfg.stand_pose_rad.assign(12, 0.0);
    cfg.joint_min_rad.assign(12, -3.14);
    cfg.joint_max_rad.assign(12, 3.14);
    cfg.dof_pos_scale.assign(12, 1.0);
    cfg.dof_vel_scale.assign(12, 1.0);
    cfg.command_scale = {1.0, 1.0, 1.0};
    cfg.body_ang_vel_scale = {1.0, 1.0, 1.0};
    cfg.euler_scale = {1.0, 1.0, 1.0};

    return cfg;
}

bool safety_countdown()
{
    for (int remaining = 3; remaining > 0; --remaining) {
        if (g_stop_requested.load()) {
            return false;
        }
        std::cout << "[INFO] Starting hardware in " << remaining
                  << " seconds. Press Ctrl+C to cancel.\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return !g_stop_requested.load();
}

void safe_shutdown(inference::RobotInterface& robot)
{
    std::cout << "[INFO] Stopping motors and releasing hardware...\n";
    robot.stop_motors(-1);
    robot.deinit_motors();
    robot.deinit_imu();
    std::cout << "[INFO] Shutdown complete.\n";
}

}  // namespace

int main()
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    const inference::RobotInterfaceConfig cfg = make_robot_config();

    if (!file_readable(cfg.policy_model_path)) {
        std::cerr << "[ERROR] Policy model is not readable: "
                  << cfg.policy_model_path << "\n"
                  << "[ERROR] Put the TorchScript model at "
                  << "src/inference/model/policy.pt and run from "
                  << "src/inference/build.\n";
        return 1;
    }

    if (!safety_countdown()) {
        std::cout << "[INFO] Startup canceled before hardware initialization.\n";
        return 0;
    }

    inference::RobotInterface robot(cfg);

    std::cout << "[INFO] Starting IMU...\n";
    if (!robot.initial_and_start_imu()) {
        std::cerr << "[ERROR] initial_and_start_imu() failed.\n";
        return 1;
    }

    std::cout << "[INFO] Waiting 1 second for IMU/AHRS warmup...\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (g_stop_requested.load()) {
        safe_shutdown(robot);
        return 0;
    }

    std::cout << "[INFO] Starting EtherCAT motors...\n";
    if (!robot.initial_and_start_motors()) {
        std::cerr << "[ERROR] initial_and_start_motors() failed.\n";
        safe_shutdown(robot);
        return 1;
    }

    std::cout << "[INFO] Loading policy...\n";
    if (!robot.load_policy()) {
        std::cerr << "[ERROR] load_policy() failed.\n";
        safe_shutdown(robot);
        return 1;
    }

    std::cout << "[INFO] Restarting motors...\n";
    if (!robot.restart_motors(-1)) {
        std::cerr << "[ERROR] restart_motors(-1) failed.\n";
        safe_shutdown(robot);
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (g_stop_requested.load()) {
        safe_shutdown(robot);
        return 0;
    }

    std::cout << "[INFO] Resetting joints to stand_pose_rad...\n";
    if (!robot.reset_joints()) {
        std::cerr << "[ERROR] reset_joints() failed.\n";
        safe_shutdown(robot);
        return 1;
    }

    std::cout << "[INFO] Entering policy loop. Press Ctrl+C to stop.\n";
    using Clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(kPolicyPeriodSec));
    auto next_tick = Clock::now();
    auto last_report = Clock::now();

    std::uint64_t steps = 0;
    double total_step_ms = 0.0;
    double max_step_ms = 0.0;

    while (!g_stop_requested.load()) {
        next_tick += period;

        const auto step_start = Clock::now();
        if (!robot.policy_step(kCommandVx, kCommandVy, kCommandYawRate)) {
            std::cerr << "[ERROR] policy_step() failed at step " << steps << ".\n";
            safe_shutdown(robot);
            return 1;
        }
        const auto step_end = Clock::now();

        const double step_ms =
            std::chrono::duration<double, std::milli>(step_end - step_start).count();
        total_step_ms += step_ms;
        max_step_ms = std::max(max_step_ms, step_ms);
        ++steps;

        const auto now = Clock::now();
        if (now - last_report >= std::chrono::seconds(1)) {
            const double avg_step_ms = total_step_ms / static_cast<double>(steps);
            std::cout << std::fixed << std::setprecision(3)
                      << "[INFO] steps=" << steps
                      << " avg_policy_step_ms=" << avg_step_ms
                      << " max_policy_step_ms=" << max_step_ms << "\n";
            last_report = now;
        }

        std::this_thread::sleep_until(next_tick);
    }

    const double avg_step_ms = steps > 0
        ? total_step_ms / static_cast<double>(steps)
        : 0.0;
    std::cout << std::fixed << std::setprecision(3)
              << "[INFO] Exiting policy loop. total_steps=" << steps
              << " avg_policy_step_ms=" << avg_step_ms
              << " max_policy_step_ms=" << max_step_ms << "\n";

    safe_shutdown(robot);
    return 0;
}
