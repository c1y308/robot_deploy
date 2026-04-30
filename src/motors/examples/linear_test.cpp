#include "ControlTypes.hpp"
#include "EthercatAdapterIGH.hpp"
#include "motor_control.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

volatile std::sig_atomic_t g_should_stop = 0;

// EtherCAT 主站网卡名与系统中的实际设备名保持一致。
constexpr char kIfName[] = "enp8s0";
constexpr int kNumMotors = 2;
constexpr double kPi = 3.14159265358979323846;

// 测试流程相关的时间参数。
constexpr int kWarmupMs = 1000;
constexpr int kWaitReadyTimeoutMs = 10000;
constexpr int kWaitReadyPollMs = 100;
constexpr int kModeSwitchWaitMs = 1000;
constexpr int kRestartWaitMs = 1000;
constexpr int kCommandPeriodMs = 10;      // 100 Hz setpoint streaming
constexpr int kPolicyPeriodMs = 50;       // 20 Hz "RL policy" updates
constexpr int kTrajectoryDurationMs = 20000;

// 轨迹幅值/偏置/最大相邻采样点速度约束。
constexpr double kAmplitudeRad = 0.35;
constexpr double kBiasRad = 0.0;
constexpr double kMaxVelocityRadPerSec = 1.5;

// 仅对这些电机发送变化轨迹，其他电机保持在初始位置。
const std::vector<int> kActiveMotorIds = {0, 1};

void signal_handler(int)
{
    g_should_stop = 1;
}

bool contains_motor(int motor_id)
{
    return std::find(kActiveMotorIds.begin(), kActiveMotorIds.end(), motor_id) !=
           kActiveMotorIds.end();
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
    // 统一切换全部电机模式，避免不同轴处于不同控制模式。
    for (int i = 0; i < kNumMotors; ++i) {
        controller.send_command(
            myactua::ControlCommand(myactua::CommandType::SET_MODE, i, {}, mode));
    }
}

double clamp_target(double value_rad, double base_rad)
{
    // 将目标限制在初始位置附近，减少测试时的大幅度运动风险。
    const double lower = base_rad - kAmplitudeRad;
    const double upper = base_rad + kAmplitudeRad;
    return std::clamp(value_rad, lower, upper);
}

double synthetic_policy_output(double elapsed_sec, double base_rad, int motor_id)
{
    // 用多频正弦叠加来模拟强化学习策略连续变化的关节角输出。
    const double phase = 0.35 * static_cast<double>(motor_id);
    const double wave_a = std::sin(2.0 * kPi * 0.30 * elapsed_sec + phase);
    const double wave_b = 0.35 * std::sin(2.0 * kPi * 0.83 * elapsed_sec + 0.7 + phase);
    const double wave_c = 0.15 * std::cos(2.0 * kPi * 0.17 * elapsed_sec + 1.2);
    return clamp_target(base_rad + kBiasRad + kAmplitudeRad * (wave_a + wave_b + wave_c),
                        base_rad);
}

double interpolate_linear(double start, double end, double alpha)
{
    // 在相邻两个策略输出点之间做线性插值，生成高频平滑位置命令。
    return start + (end - start) * alpha;
}

bool validate_trajectory_span()
{
    // 预先检查低频策略采样点之间的跳变量，保证轨迹本身满足速度约束。
    const double delta_limit =
        kMaxVelocityRadPerSec * (static_cast<double>(kPolicyPeriodMs) / 1000.0);

    for (int motor_id : kActiveMotorIds) {
        double prev = synthetic_policy_output(0.0, 0.0, motor_id);
        for (int t_ms = kPolicyPeriodMs; t_ms <= kTrajectoryDurationMs; t_ms += kPolicyPeriodMs) {
            const double current =
                synthetic_policy_output(static_cast<double>(t_ms) / 1000.0, 0.0, motor_id);
            const double delta = std::abs(current - prev);
            if (delta > delta_limit) {
                std::cerr << "[trajectory] policy knot delta too large on motor " << motor_id
                          << ": " << delta << " rad > " << delta_limit << " rad" << std::endl;
                return false;
            }
            prev = current;
        }
    }

    return true;
}

std::vector<double> rad_targets_to_deg(const std::vector<double>& rad_targets)
{
    std::vector<double> deg_targets(rad_targets.size(), 0.0);
    for (std::size_t i = 0; i < rad_targets.size(); ++i) {
        deg_targets[i] = myactua::MYACTUA::rad_to_deg(rad_targets[i]);
    }
    return deg_targets;
}

void sleep_until_next(Clock::time_point wakeup_time)
{
    std::this_thread::sleep_until(wakeup_time);
}

}  // namespace

int main()
{
    std::signal(SIGINT, signal_handler);

    // 启动前先验证轨迹参数，避免发送过陡的测试曲线。
    if (!validate_trajectory_span()) {
        return -1;
    }

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

    controller.set_print_info(kActiveMotorIds);
    controller.start();

    // 先停机再切换模式，让测试从一个确定的状态开始。
    std::cout << "[flow] stop all motors before switching mode" << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
    std::this_thread::sleep_for(std::chrono::milliseconds(kWarmupMs));

    std::cout << "[flow] switch all motors to CSP" << std::endl;
    send_mode_all(controller, myactua::ControlMode::CSP);
    std::this_thread::sleep_for(std::chrono::milliseconds(kModeSwitchWaitMs));

    std::vector<double> initial_positions = controller.get_joint_q_rad();
    if (initial_positions.size() != static_cast<std::size_t>(kNumMotors)) {
        std::cerr << "[error] failed to read initial joint positions" << std::endl;
        controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
        controller.shutdown();
        return -1;
    }

    // 初始阶段保持当前位置，避免 RESTART 后立即出现位置跳变。
    std::vector<double> setpoints_rad = initial_positions;
    controller.send_command(
        myactua::ControlCommand(
            myactua::CommandType::SET_SETPOINTS, -1, rad_targets_to_deg(setpoints_rad)));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << "[flow] restart motors and hold current pose" << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::RESTART, -1));
    std::this_thread::sleep_for(std::chrono::milliseconds(kRestartWaitMs));

    std::cout << "[test] stream interpolated CSP targets for "
              << kTrajectoryDurationMs / 1000.0 << " s" << std::endl;

    const auto start_time = Clock::now();
    auto next_wakeup = start_time;
    double peak_command_step = 0.0;
    int loop_count = 0;

    while (!g_should_stop) {
        const auto now = Clock::now();
        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();

        if (elapsed_ms >= kTrajectoryDurationMs) {
            break;
        }

        const int segment_index = static_cast<int>(elapsed_ms / kPolicyPeriodMs);
        const int segment_start_ms = segment_index * kPolicyPeriodMs;
        const int segment_end_ms = segment_start_ms + kPolicyPeriodMs;
        const double alpha =
            static_cast<double>(elapsed_ms - segment_start_ms) / static_cast<double>(kPolicyPeriodMs);

        for (int motor_id = 0; motor_id < kNumMotors; ++motor_id) {
            if (!contains_motor(motor_id)) {
                setpoints_rad[motor_id] = initial_positions[motor_id];
                continue;
            }

            const double base_rad = initial_positions[motor_id];
            // 取相邻两个“策略时刻”的目标点，并在当前控制周期内线性插值。
            const double knot_start = synthetic_policy_output(
                static_cast<double>(segment_start_ms) / 1000.0, base_rad, motor_id);
            const double knot_end = synthetic_policy_output(
                static_cast<double>(segment_end_ms) / 1000.0, base_rad, motor_id);
            const double interpolated = interpolate_linear(knot_start, knot_end, alpha);

            // 记录单周期目标变化量，用来评估命令流是否足够平滑。
            peak_command_step =
                std::max(peak_command_step, std::abs(interpolated - setpoints_rad[motor_id]));
            setpoints_rad[motor_id] = interpolated;
        }

        const std::vector<double> setpoints_deg = rad_targets_to_deg(setpoints_rad);
        controller.send_command(
            myactua::ControlCommand(myactua::CommandType::SET_SETPOINTS, -1, setpoints_deg));

        if ((loop_count % 20) == 0) {
            // 定期打印目标值、实际值和误差，便于观察电机跟踪效果。
            const auto measured = controller.get_joint_q_rad();
            if (!measured.empty()) {
                for (int motor_id : kActiveMotorIds) {
                    if (motor_id < static_cast<int>(measured.size())) {
                        std::cout << "[trace] t=" << elapsed_ms << " ms"
                                  << " cmd=" << setpoints_deg[motor_id]
                                  << " deg"
                                  << " (" << setpoints_rad[motor_id] << " rad)"
                                  << " meas=" << measured[motor_id]
                                  << " rad"
                                  << " err=" << (setpoints_rad[motor_id] - measured[motor_id])
                                  << " rad" << std::endl;
                    }
                }
            }
        }

        ++loop_count;
        next_wakeup += std::chrono::milliseconds(kCommandPeriodMs);
        sleep_until_next(next_wakeup);
    }

    // 测试结束后先保持最后目标，再执行停机。
    std::cout << "[test] hold final interpolated target for 1 s" << std::endl;
    controller.send_command(
        myactua::ControlCommand(
            myactua::CommandType::SET_SETPOINTS, -1, rad_targets_to_deg(setpoints_rad)));
    std::this_thread::sleep_for(std::chrono::seconds(1));

    controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
    controller.shutdown();

    std::cout << "[done] peak command increment per " << kCommandPeriodMs
              << " ms cycle: " << peak_command_step << " rad" << std::endl;
    return 0;
}
