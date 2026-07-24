#include "robot_interface.hpp"
#include "xbox_controller.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

#if POLICY_V3
constexpr const char* kPolicyModelFile = "policy_v3.pt";
#else
constexpr const char* kPolicyModelFile = "policy.pt";
#endif

constexpr const char* kEthercatIfname = "enp8s0";
constexpr const char* kImuDevice = "/dev/ttyUSB0";
constexpr int kImuBaudrate = 921600;

constexpr double kPolicyHz = 50.0;
constexpr double kPolicyPeriodSec = 1.0 / kPolicyHz;
constexpr bool kPrintPolicyTiming = false;

std::atomic<bool> g_stop_requested{false};

void signal_handler(int)
{
    g_stop_requested.store(true);
}

std::filesystem::path policy_model_path()
{
    return std::filesystem::path(__FILE__).parent_path()
        .parent_path()
        .parent_path() / "inference" / "model" / kPolicyModelFile;
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
    cfg.imu_print_imu   = false;
    cfg.imu_print_ahrs  = false;
    cfg.imu_print_stats = false;

    cfg.policy_model_path = policy_model_path().string();

    cfg.print_motors_info = false;

#if POLICY_V3
    cfg.model_to_motor_index = {0, 1, 2, 3, 4, 5,
                                6, 7, 8, 9, 11, 10};
    cfg.left_ankle_parallel = {8, 10, 8, 11};
    cfg.right_ankle_parallel = {9, 11, 9, 10};
#else
    //  把模型 DOF 顺序映射到电机逻辑索引，长度必须为 12 且不可重复；按模型 DOF 序号使用
    cfg.model_to_motor_index = {0, 6, 1, 7,  2, 8,
                                3, 9, 4, 10, 5, 11};
    cfg.left_ankle_parallel = {8, 10, 4, 5};
    cfg.right_ankle_parallel = {9, 11, 10, 11};

    // cfg.model_to_motor_index = {0, 1, 2, 3,  4,  5,
    //                             6, 7, 8, 9, 11, 10};
#endif
    cfg.print_motor_ids = {0, 1, 2, 3, 4, 5,
                           6, 7, 8, 9, 10, 11};


    // 按模型 DOF 顺序限制 raw_action * action_scale 后的动作偏移: [lower, upper]
    cfg.action_clip = {
        {-0.12, 0.12},
        {-0.12, 0.12},

        {-0.40, 0.40},
        {-0.40, 0.40},

        {-0.12, 0.12},
        {-0.12, 0.12},

        {0.00, 0.85},
        {0.00, 0.85},

        {-0.18, 0.18},
        {-0.18, 0.18},

        {-0.08, 0.08},
        {-0.08, 0.08},
    };
    cfg.action_scale = {
        0.08, 0.08,
        0.45, 0.45,
        0.08, 0.08,
        0.45, 0.45,
        0.20, 0.20,
        0.08, 0.08
    };
    // joint_min/max 是相对 stand_pose_rad 的偏移限位。
    cfg.joint_min_rad.assign(12, -0.45);
    cfg.joint_max_rad.assign(12,  0.45);


    cfg.policy_cycle_time_s = 0.02;

    //  按照 DOF 顺序配置电机方向，1 表示方向一致，-1 表示方向相反；为空时全部按 1
    cfg.motor_to_model_direction = {
         1, -1, 1,  1, -1, -1,
        -1,  1, 1, -1, -1, -1
    };

    // 以下 12 维策略配置均按模型 DOF 序号填写
    cfg.stand_pose_rad = {
     0.0,  0.0, -0.1, -0.1,
     0.0,  0.0,  0.3,  0.3,
    -0.2, -0.2,  0.0,  0.0
    };


    // cfg.stand_pose_rad = {
    //     0.0,  0.0, -0.0, -0.0,
    //     0.0,  0.0,  0.0,  0.0,
    //     -0.0, -0.0,  0.0,  0.0
    // };

    // 电机观测缩放系数
    cfg.dof_pos_scale.assign(12, 1.0);
    cfg.dof_vel_scale.assign(12, 0.05);


    /******************************************************************* */
    cfg.command_scale = {1.0, 1.0, 1.0};
    cfg.body_ang_vel_scale = {0.2, 0.2, 0.2};
    cfg.euler_scale = {1.0, 1.0, 1.0};

    cfg.mit_kp.assign(12, 200);
    cfg.mit_kd.assign(12, 10);

    /* 启用脚踝并联机构 IK/FK 特殊处理 */
    cfg.ankle_parallel_kinematics_enabled = true;
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

    xbox_control::XboxController controller;
    if (!controller.open_device()) {
        std::cerr << "[ERROR] " << controller.last_error() << "\n";
        return 1;
    }

    const inference::RobotInterfaceConfig cfg = make_robot_config();

    if (!file_readable(cfg.policy_model_path)) {
        std::cerr << "[ERROR] Policy model is not readable: "
                  << cfg.policy_model_path << "\n";
        return 1;
    }

    if (!safety_countdown()) {
        std::cout << "[INFO] Startup canceled before hardware initialization.\n";
        return 0;
    }

    inference::RobotInterface robot(cfg);

    std::cout << "[INFO] Starting EtherCAT motors...\n";
    if (!robot.initial_and_start_motors()) {
        std::cerr << "[ERROR] initial_and_start_motors() failed.\n";
        safe_shutdown(robot, false);
        return 1;
    }

    std::cout << "[INFO] Loading policy...\n";
    if (!robot.load_policy()) {
        std::cerr << "[ERROR] load_policy() failed.\n";
        safe_shutdown(robot, false);
        return 1;
    }

    std::cout << "[INFO] Restarting motors...\n";
    if (!robot.restart_motors(-1)) {
        std::cerr << "[ERROR] restart_motors(-1) failed.\n";
        safe_shutdown(robot, false);
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (g_stop_requested.load()) {
        safe_shutdown(robot, false);
        return 0;
    }

    std::cout << "[INFO] Starting IMU...\n";
    if (!robot.initial_and_start_imu()) {
        std::cerr << "[ERROR] initial_and_start_imu() failed.\n";
        safe_shutdown(robot, false);
        return 1;
    }

    std::cout << "[INFO] Waiting 1 second for IMU/AHRS warmup...\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (g_stop_requested.load()) {
        safe_shutdown(robot, false);
        return 0;
    }

    std::cout << "[INFO] Resetting joints to stand_pose_rad...\n";
    if (!robot.reset_joints()) {
        std::cerr << "[ERROR] reset_joints() failed.\n";
        safe_shutdown(robot, false);
        return 1;
    }

    std::cout << "[INFO] Waiting 50 seconds before entering policy loop...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    if (g_stop_requested.load()) {
        safe_shutdown(robot, false);
        return 0;
    }

    std::cout << "[INFO] Entering Xbox policy loop. Press Ctrl+C to stop.\n";
    using Clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(kPolicyPeriodSec));
    auto next_tick = Clock::now();
    auto last_report = Clock::now();

    std::uint64_t steps = 0;
    double total_step_ms = 0.0;
    double max_step_ms = 0.0;
    xbox_control::VelocityCommand command;

    while (!g_stop_requested.load()) {
        next_tick += period;

        if (!controller.poll(command)) {
            std::cerr << "[ERROR] " << controller.last_error() << "\n";
            safe_shutdown(robot, true);
            return 1;
        }
        robot.set_target_velocity(0, 0, 0);
        // robot.set_target_velocity(command.vx, command.vy, command.yaw_rate);

        const auto step_start = Clock::now();
        if (!robot.policy_step()) {
            std::cerr << "[ERROR] policy_step() failed at step " << steps << ".\n";
            safe_shutdown(robot, false);
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
            std::cout << std::fixed << std::setprecision(3)
                      << "[INFO] raw_abs_x=" << command.raw_abs_x
                      << " raw_abs_y=" << command.raw_abs_y
                      << " vx=" << command.vx
                      << " vy=" << command.vy
                      << " yaw_rate=" << command.yaw_rate;
            if (kPrintPolicyTiming && steps > 0) {
                std::cout << " avg_policy_step_ms="
                          << total_step_ms / static_cast<double>(steps)
                          << " max_policy_step_ms=" << max_step_ms;
            }
            std::cout << "\n";
            last_report = now;
        }

        std::this_thread::sleep_until(next_tick);
        if (Clock::now() > next_tick + period) {
            next_tick = Clock::now();
        }
    }

    std::cout << "[INFO] Exiting policy loop. total_steps=" << steps << "\n";

    safe_shutdown(robot, true);
    return 0;
}
