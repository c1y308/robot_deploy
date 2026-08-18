#include "robot/robot_interface.hpp"
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

constexpr const char* kPolicyModelFile = "policy.pt";

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
    cfg.motor.num_motors = 12;
    cfg.motor.ethercat_ifname = kEthercatIfname;
    cfg.imu.device      = kImuDevice;
    cfg.imu.baudrate    = kImuBaudrate;
    cfg.imu.print_imu   = false;
    cfg.imu.print_ahrs  = false;
    cfg.policy.model_path = policy_model_path().string();

    cfg.motor.print_motors_info = true;

    cfg.recorder.enabled = true;
    cfg.ankle_torque.target_torque_limit_permille = 2000.0;

    // 对齐训练 joint_ids_map:
    // [0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11]。
    // 普通单关节 DOF 使用前 8 项；脚踝并联 DOF 由 left/right_ankle_parallel 指定。
    cfg.joint_mapping.model_to_motor_index = {0, 6, 1, 7, 2, 8, 3, 9};

    cfg.joint_mapping.left_ankle_parallel = {8, 10, 4, 5};
    cfg.joint_mapping.right_ankle_parallel = {9, 11, 10, 11};

    cfg.motor.mit_kp = { 237.0, 237.0, 237.0, 237.0,
                         185.0, 185.0, 185.0, 185.0,
                         180.0, 180.0, 187.0, 187.0};
    
    cfg.motor.mit_kd = { 15.0, 15.0, 15.0, 15.0,
                          7.0,  7.0, 7.0, 7.0,
                         10.0, 10.0,  9.07,  9.07};

    cfg.motor.print_motor_ids = {0, 1, 2, 3, 4, 5,
                                 6, 7, 8, 9, 10, 11};

    // 以下 12 维策略配置均按模型 DOF 序号填写
    cfg.policy.stand_pose_rad = {
        0.0, 0.0, -0.2, -0.2,
        0.0, 0.0,  0.2,  0.2,
       -0.05, -0.05, 0.0, 0.0
    };


    // 按模型 DOF 顺序限制 raw_action * action_scale 后的动作偏移: [lower, upper]。
    // 代码计算: target = stand_pose_rad + clamp(raw_action * action_scale, [lower, upper])。
    cfg.policy.action_clip = {
        {-0.22, 0.22},
        {-0.22, 0.22},

        {-0.28, 0.35},
        {-0.28, 0.35},

        {-0.16, 0.16},
        {-0.16, 0.16},

        {-0.22, 0.38},
        {-0.22, 0.38},

        {-0.14, 0.2},
        {-0.14, 0.2},

        {-0.12, 0.12},
        {-0.12, 0.12},
    };
    cfg.policy.action_scale = {
        0.16, 0.16,
        0.32, 0.32,
        0.1, 0.1,
        0.36, 0.36,
        0.18, 0.18,
        0.1, 0.1
    };
    cfg.policy.raw_action_clip = 1.0;

    // joint_min/max 是相对 stand_pose_rad 的偏移限位。
    cfg.policy.joint_min_rad.assign(12, -0.7);
    cfg.policy.joint_max_rad.assign(12,  0.7);

    //  按照 DOF 顺序配置电机方向，1 表示方向一致，-1 表示方向相反；为空时全部按 1
    cfg.joint_mapping.motor_to_model_direction = {
         1, -1, 1,  1, -1, -1,
        -1,  1, 1, -1, -1, -1
    };

    /******************************************************************* */
    cfg.policy.command_scale      = {1.0, 1.0, 1.0};
    cfg.policy.body_ang_vel_scale = {0.2, 0.2, 0.2};
    cfg.policy.step_dt = 0.02;
    cfg.policy.gait_phase_period = 0.74;
    cfg.policy.gait_phase_stand_threshold = 0.05;
    cfg.policy.gait_phase_move_threshold = 0.15;


    // 电机观测缩放系数
    cfg.policy.dof_pos_scale.assign(12, 1.0);
    cfg.policy.dof_vel_scale.assign(12, 0.05);

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
    robot.shutdown();
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

    std::cout << "[INFO] Waiting 5 seconds before entering policy loop...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    if (g_stop_requested.load()) {
        safe_shutdown(robot, false);
        return 0;
    }

    std::cout << "[INFO] Starting Xbox polling thread...\n";
    if (!controller.start_polling(std::chrono::milliseconds(20))) {
        std::cerr << "[ERROR] " << controller.last_error() << "\n";
        safe_shutdown(robot, true);
        return 1;
    }

    std::cout << "[INFO] Entering Xbox policy loop. Press Ctrl+C to stop.\n";

    /* 使用 steady_clock 控制 50hz 的推理频率 */
    using Clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(kPolicyPeriodSec));
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
        robot.set_target_velocity(command.vx, 0.0, 0.0);
        // robot.set_target_velocity(0, command.vx, 0.0);


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
