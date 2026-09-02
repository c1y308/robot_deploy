#include "config/deploy_config.hpp"
#include "robot/robot_interface.hpp"
#include "xbox_controller.hpp"

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

#ifndef ROBOT_DEPLOY_CONFIG_PATH
#define ROBOT_DEPLOY_CONFIG_PATH ""
#endif

namespace {

constexpr bool kPrintPolicyTiming = false;

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

/* 打印配置摘要，便于确认加载结果 */
void print_config_summary(const inference::RobotInterfaceConfig& cfg)
{
    std::cout << "[INFO] Deploy config summary:\n"
              << "  ethercat_ifname: " << cfg.motor.ethercat_ifname << "\n"
              << "  imu device: " << cfg.imu.device
              << " baudrate: " << cfg.imu.baudrate
              << " configure_can: "
              << (cfg.imu.configure_can ? "true" : "false")
              << " can_bitrate: " << cfg.imu.can_bitrate << "\n"
              << "  model_path: " << cfg.policy.model_path << "\n"
              << "  step_dt: " << cfg.policy.step_dt << "\n"
              << "  raw_action_clip: " << cfg.action.raw_action_clip << "\n";
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

void send_zero_velocity(inference::RobotInterface& robot)
{
    robot.set_target_velocity(0.0, 0.0, 0.0);
    if (!robot.policy_step()) {
        std::cerr << "[WARN] Failed to send one final zero-velocity policy step.\n";
    }
}

void safe_shutdown(inference::RobotInterface& robot, bool send_zero)
{
    if (send_zero) {
        send_zero_velocity(robot);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "[INFO] Stopping motors and releasing hardware...\n";
    robot.shutdown();
    std::cout << "[INFO] Shutdown complete.\n";
}

}  // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    const std::string config_path =
        argc > 1 ? argv[1] : std::string(ROBOT_DEPLOY_CONFIG_PATH);

    inference::RobotInterfaceConfig cfg;
    std::string config_error;
    if (!inference::load_deploy_config(config_path, cfg, config_error)) {
        std::cerr << "[ERROR] Failed to load deploy config: "
                  << config_error << "\n";
        return 1;
    }
    print_config_summary(cfg);

    xbox_control::XboxController controller;
    if (!controller.open_device()) {
        std::cerr << "[ERROR] " << controller.last_error() << "\n";
        return 1;
    }

    if (!file_readable(cfg.policy.model_path)) {
        std::cerr << "[ERROR] Policy model is not readable: "
                  << cfg.policy.model_path << "\n";
        return 1;
    }

    if (!safety_countdown()) {
        std::cout << "[INFO] Startup canceled before hardware initialization.\n";
        return 0;
    }

    inference::RobotInterface robot(cfg);

    std::cout << "[INFO] Initializing robot runtime...\n";
    if (!robot.initialize()) {
        std::cerr << "[ERROR] robot.initialize() failed.\n";
        safe_shutdown(robot, false);
        return 1;
    }

    std::cout << "[INFO] Starting Xbox polling thread...\n";
    if (!controller.start_polling(std::chrono::milliseconds(20))) {
        std::cerr << "[ERROR] " << controller.last_error() << "\n";
        safe_shutdown(robot, true);
        return 1;
    }

    std::cout << "[INFO] Entering Xbox policy loop. Press Ctrl+C to stop.\n";

    /* 使用 steady_clock 控制推理频率，周期来自 deploy.yaml 的 step_dt */
    using Clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(cfg.policy.step_dt));
    auto next_tick   = Clock::now();
    auto last_report = Clock::now();

    std::uint64_t steps  = 0;       // 推理次数
    double total_step_ms = 0.0;     // 推理总耗时
    double max_step_ms   = 0.0;     // 推理最大耗时
    xbox_control::VelocityCommand command;

    while (!g_stop_requested.load()) {
        next_tick += period;

        if (!controller.latest_command(command)) {
            std::cerr << "[ERROR] " << controller.last_error() << "\n";
            controller.stop_polling();
            safe_shutdown(robot, true);
            return 1;
        }
        // robot.set_target_velocity(0.0, 0.0, 0.0);
        robot.set_target_velocity( command.vx, 0.0, 0.0);
        // robot.set_target_velocity( 0.0, command.vx, 0.0);

        const auto step_start = Clock::now();
        if (!robot.policy_step()) {
            std::cerr << "[ERROR] policy_step() failed at step " << steps << ".\n";
            controller.stop_polling();
            safe_shutdown(robot, false);
            return 1;
        }
        const auto step_end = Clock::now();
        const double step_ms = std::chrono::duration<double, std::milli>(step_end - step_start).count();
        // 更新推理耗时统计
        total_step_ms += step_ms;
        max_step_ms    = std::max(max_step_ms, step_ms);
        ++steps;

        const auto now = Clock::now();
        if (now - last_report >= std::chrono::seconds(1)) {
            // std::cout << std::fixed << std::setprecision(3)
            //           << " vx=" << command.vx
            //           << " vy=" << command.vy
            //           << " yaw_rate=" << command.yaw_rate;
            // if (kPrintPolicyTiming && steps > 0) {
            //     std::cout << " avg_policy_step_ms="
            //               << total_step_ms / static_cast<double>(steps)
            //               << " max_policy_step_ms=" << max_step_ms;
            // }
            // std::cout << "\n";
            // last_report = now;
        }

        std::this_thread::sleep_until(next_tick);
        if (Clock::now() > next_tick + period) {
            next_tick = Clock::now();
        }
    }

    std::cout << "[INFO] Exiting policy loop. total_steps=" << steps << "\n";

    controller.stop_polling();
    safe_shutdown(robot, true);
    return 0;
}
