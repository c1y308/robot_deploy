#include "ControlTypes.hpp"
#include "EthercatAdapterIGH.hpp"
#include "motor_control.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

volatile std::sig_atomic_t g_should_stop = 0;

constexpr char kIfName[] = "enp8s0";
constexpr int kNumMotors = 12;

constexpr int kWaitReadyTimeoutMs = 20000;  // 等待全部从站进入 OP 的超时时间
constexpr int kWaitReadyPollMs = 100;       // 检查从站状态的轮询周期
constexpr int kWarmupMs = 1000;             // 停机后等待控制状态稳定的时间
constexpr int kModeSwitchWaitMs = 1000;     // 切换到 CSP 模式后等待模式生效的时间
constexpr int kRestartWaitMs = 1000;        // RESTART 后等待电机完成使能的时间
constexpr int kCommandPeriodMs = 10;        // 插值轨迹下发周期
constexpr int kSquatDurationMs = 300;      // 从站立到下蹲的轨迹时长
constexpr int kHoldBottomMs = 2000;         // 下蹲到底后保持不动的时间
constexpr int kStandDurationMs = 300;      // 从下蹲恢复到站立的轨迹时长
constexpr char kErrorLogFile[] = "squat_stand_interp_error.txt";

// “机器人蹲下最终时刻”12 个电机位置，单位为 deg。
// constexpr std::array<double, kNumMotors> kSquatTargetDeg = {
//     360, 360

// };
constexpr std::array<double, kNumMotors> kSquatTargetDeg = {
    8.6, 21.745, -12.8, 50, 90, -94,
    0.0, -17, -1, -50, 0, 0
};

void signal_handler(int)
{
    g_should_stop = 1;
}

bool wait_all_slaves_ready(const std::shared_ptr<myactua::EthercatAdapterIGH>& adapter)
{
    const auto start_time = Clock::now();
    const auto deadline = start_time + std::chrono::milliseconds(kWaitReadyTimeoutMs);
    auto next_log_time = start_time;

    while (Clock::now() < deadline && !g_should_stop) {
        adapter->receivePhysical();
        adapter->sendPhysical();

        int ready_count = 0;
        for (int i = 0; i < kNumMotors; ++i) {
            if (adapter->isConfigured(i)) {
                ++ready_count;
            }
        }

        if (ready_count == kNumMotors) {
            std::cout << "[ready] all " << kNumMotors << " slaves are in OP" << std::endl;
            return true;
        }

        const auto now = Clock::now();
        if (now >= next_log_time) {
            std::cout << "[ready] " << ready_count << "/" << kNumMotors << " slaves ready"
                      << std::endl;
            next_log_time = now + std::chrono::milliseconds(kWaitReadyPollMs);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return false;
}

void send_mode_all(myactua::MYACTUA& controller, myactua::ControlMode mode)
{
    for (int i = 0; i < kNumMotors; ++i) {
        controller.send_command(
            myactua::ControlCommand(myactua::CommandType::SET_MODE, i, {}, mode));
    }
}

std::vector<double> array_to_vector(const std::array<double, kNumMotors>& source)
{
    return std::vector<double>(source.begin(), source.end());
}

double smoothstep(double alpha)
{
    const double clamped = std::clamp(alpha, 0.0, 1.0);
    return clamped * clamped * (3.0 - 2.0 * clamped);
}

std::vector<double> interpolate_pose(const std::vector<double>& start_deg,
                                     const std::vector<double>& end_deg,
                                     double alpha)
{
    std::vector<double> pose_deg(start_deg.size(), 0.0);
    const double blend = smoothstep(alpha);
    for (std::size_t i = 0; i < start_deg.size(); ++i) {
        pose_deg[i] = start_deg[i] + (end_deg[i] - start_deg[i]) * blend;
    }
    return pose_deg;
}

void stream_pose_segment(myactua::MYACTUA& controller,
                         const std::vector<double>& start_deg,
                         const std::vector<double>& end_deg,
                         int duration_ms,
                         const char* stage_name,
                         std::ofstream& error_log)
{
    std::cout << "[motion] " << stage_name << " for " << duration_ms << " ms" << std::endl;

    const auto start_time = Clock::now();
    auto next_wakeup = start_time;

    while (!g_should_stop) {
        const auto now = Clock::now();
        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();

        if (elapsed_ms >= duration_ms) {
            break;
        }

        const double alpha =
            static_cast<double>(elapsed_ms) / static_cast<double>(duration_ms);
        const std::vector<double> pose_deg = interpolate_pose(start_deg, end_deg, alpha);

        controller.send_command(
            myactua::ControlCommand(myactua::CommandType::SET_SETPOINTS, -1, pose_deg));

        const std::vector<double> measured_rad = controller.get_joint_q_rad();
        const std::size_t motor_count = std::min(pose_deg.size(), measured_rad.size());
        for (std::size_t i = 0; i < motor_count; ++i) {
            const double current_deg = myactua::MYACTUA::rad_to_deg(measured_rad[i]);
            const double error_deg = pose_deg[i] - current_deg;
            error_log << stage_name << '\t'
                      << elapsed_ms << '\t'
                      << i << '\t'
                      << pose_deg[i] << '\t'
                      << current_deg << '\t'
                      << error_deg << '\n';
        }
        error_log.flush();

        next_wakeup += std::chrono::milliseconds(kCommandPeriodMs);
        std::this_thread::sleep_until(next_wakeup);
    }

    controller.send_command(
        myactua::ControlCommand(myactua::CommandType::SET_SETPOINTS, -1, end_deg));

    const std::vector<double> measured_rad = controller.get_joint_q_rad();
    const std::size_t motor_count = std::min(end_deg.size(), measured_rad.size());
    for (std::size_t i = 0; i < motor_count; ++i) {
        const double current_deg = myactua::MYACTUA::rad_to_deg(measured_rad[i]);
        const double error_deg = end_deg[i] - current_deg;
        error_log << stage_name << '\t'
                  << duration_ms << '\t'
                  << i << '\t'
                  << end_deg[i] << '\t'
                  << current_deg << '\t'
                  << error_deg << '\n';
    }
    error_log.flush();
}

}  // namespace

int main()
{
    std::signal(SIGINT, signal_handler);

    std::ofstream error_log(kErrorLogFile, std::ios::out | std::ios::trunc);
    if (!error_log.is_open()) {
        std::cerr << "[error] failed to open log file: " << kErrorLogFile << std::endl;
        return -1;
    }
    error_log << std::fixed << std::setprecision(6);
    error_log << "stage\telapsed_ms\tmotor_id\ttarget_deg\tcurrent_deg\terror_deg\n";
    std::cout << "[log] write interpolation error to " << kErrorLogFile << std::endl;

    auto adapter = std::make_shared<myactua::EthercatAdapterIGH>();
    myactua::MYACTUA controller(adapter, kNumMotors);

    std::cout << "[init] connect to " << kIfName << std::endl;
    if (!controller.connect(kIfName)) {
        std::cerr << "[error] failed to connect EtherCAT interface" << std::endl;
        return -1;
    }

    if (!wait_all_slaves_ready(adapter)) {
        std::cerr << "[error] not all slaves reached OP within "
                  << kWaitReadyTimeoutMs << " ms" << std::endl;
        return -1;
    }

    controller.set_print_info({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});
    controller.start();

    std::cout << "[flow] stop all motors" << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
    std::this_thread::sleep_for(std::chrono::milliseconds(kWarmupMs));

    std::cout << "[flow] switch all motors to CSP" << std::endl;
    send_mode_all(controller, myactua::ControlMode::CSP);
    std::this_thread::sleep_for(std::chrono::milliseconds(kModeSwitchWaitMs));

    const std::vector<double> zero_pose_deg(kNumMotors, 0.0);
    const std::vector<double> squat_pose_deg = array_to_vector(kSquatTargetDeg);

    // 先发送零位目标，再使能电机，保证动作从初始姿态开始。
    controller.send_command(
        myactua::ControlCommand(myactua::CommandType::SET_SETPOINTS, -1, zero_pose_deg));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << "[flow] restart motors" << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::RESTART, -1));
    std::this_thread::sleep_for(std::chrono::milliseconds(kRestartWaitMs));

    stream_pose_segment(
        controller, zero_pose_deg, squat_pose_deg, kSquatDurationMs, "squat", error_log);

    if (!g_should_stop) {
        std::cout << "[hold] keep squat pose for " << kHoldBottomMs << " ms" << std::endl;
        controller.send_command(
            myactua::ControlCommand(myactua::CommandType::SET_SETPOINTS, -1, squat_pose_deg));
        std::this_thread::sleep_for(std::chrono::milliseconds(kHoldBottomMs));
    }

    if (!g_should_stop) {
        stream_pose_segment(
            controller, squat_pose_deg, zero_pose_deg, kStandDurationMs, "stand", error_log);
    }

    std::cout << "[flow] stop all motors" << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
    controller.shutdown();

    std::cout << "[done] squat/stand demo finished" << std::endl;
    return 0;
}
