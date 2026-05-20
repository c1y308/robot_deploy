#include "ControlTypes.hpp"
#include "EthercatAdapterIGH.hpp"
#include "motor_control.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

namespace {

volatile std::sig_atomic_t g_should_stop = 0;

constexpr char kIfName[] = "enp8s0";
constexpr int kNumMotors = 12;

constexpr int kWaitReadyTimeoutMs = 20000;
constexpr int kWaitReadyPollMs = 100;
constexpr int kWarmupMs = 1000;
constexpr int kModeSwitchWaitMs = 1500;
constexpr int kRestartWaitMs = 1000;
constexpr int kPreloadSetpointMs = 200;
constexpr int kCommandPeriodMs = 10;
constexpr int kInitialHoldMs = 3000;
constexpr int kMoveDurationMs = 5000;
constexpr int kHoldTargetMs = 20000;
constexpr int kShutdownHoldMs = 500;

constexpr double kDefaultKp = 250.0;
constexpr double kDefaultKd = 10.0;

// 12 个电机的 MIT/PVT 位置目标偏移，单位 deg；目标 = 上电读取位置 + offset。
// constexpr std::array<double, kNumMotors> kTargetOffsetsDeg = {
//     0.0, 0.0, -25.7831, -25.7831,0.0, 0.0,
//     45.8366,  45.8366, 17.1887, 17.1887, 0.0, 0.0
// };
constexpr std::array<double, kNumMotors> kTargetOffsetsDeg = {
    0.0,   25.7831, 0,   45.8366, 0,   18,
    0.0,  -25.7831, 0,  -45.8366, 18, 0
};

void signal_handler(int)

{
    g_should_stop = 1;
}

void sleep_ms(int ms)
{
    constexpr int kSleepSliceMs = 20;
    for (int slept = 0; slept < ms && !g_should_stop; slept += kSleepSliceMs) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(std::min(kSleepSliceMs, ms - slept)));
    }
}

}  // namespace

int main()
{
    std::signal(SIGINT, signal_handler);

    auto adapter = std::make_shared<myactua::EthercatAdapterIGH>();
    myactua::MYACTUA controller(adapter, kNumMotors);

    std::cout << "\n========== 12-motor MIT position test ==========" << std::endl;
    std::cout << "[init] connect to " << kIfName << std::endl;
    if (!controller.connect(kIfName)) {
        std::cerr << "[error] failed to connect EtherCAT interface" << std::endl;
        return -1;
    }

    if (!controller.wait_all_slaves_ready(
            kWaitReadyTimeoutMs, kWaitReadyPollMs, [] { return g_should_stop != 0; })) {
        std::cerr << "[error] not all slaves reached OP within "
                  << kWaitReadyTimeoutMs << " ms" << std::endl;
        return -1;
    }

    std::vector<int> all_ids(kNumMotors);
    for (int i = 0; i < kNumMotors; ++i) all_ids[i] = i;
    controller.set_print_info(all_ids);

    controller.start();

    std::cout << "[flow] stop all motors before mode switch" << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
    sleep_ms(kWarmupMs);

    std::cout << "[flow] switch all motors to PVT/MIT" << std::endl;
    for (int i = 0; i < kNumMotors; ++i) {
        controller.send_command(myactua::ControlCommand(
            myactua::CommandType::SET_MODE, i, {}, myactua::ControlMode::PVT));
    }
    sleep_ms(kModeSwitchWaitMs);

    /* 获取初始位置值 */
    std::vector<double> home_rad;
    {
        constexpr int kMaxAttempts = 50;
        constexpr int kRetrySleepMs = 50;
        for (int attempt = 0; attempt < kMaxAttempts && !g_should_stop; ++attempt) {
            home_rad = controller.get_joint_q_rad();
            if (home_rad.size() == static_cast<std::size_t>(kNumMotors)) break;
            sleep_ms(kRetrySleepMs);
        }
    }
    if (home_rad.size() != static_cast<std::size_t>(kNumMotors)) {
        std::cerr << "[error] failed to read initial joint positions" << std::endl;
        controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
        controller.shutdown();
        return -1;
    }

    std::vector<double> home_deg(kNumMotors);
    std::vector<double> target_deg(kNumMotors);
    for (std::size_t i = 0; i < home_rad.size(); ++i) {
        home_deg[i] = myactua::MYACTUA::rad_to_deg(home_rad[i]);
        target_deg[i] = home_deg[i] + kTargetOffsetsDeg[i];
    }

    std::cout << "[info] home / target pose:" << std::fixed << std::setprecision(4);
    for (std::size_t i = 0; i < home_deg.size(); ++i) {
        std::cout << "\n  motor[" << i << "] home=" << home_deg[i] << " deg"
                  << "  target=" << target_deg[i] << " deg";
    }
    std::cout << std::defaultfloat << std::endl;

    std::cout << "[flow] preload MIT setpoints at current pose" << std::endl;
    {
        std::vector<myactua::MitSetpoint> sp;
        sp.reserve(home_deg.size());
        for (double p : home_deg)
            sp.emplace_back(p, 0.0, 0.0, kDefaultKp, kDefaultKd);
        controller.send_command(myactua::ControlCommand(
            myactua::CommandType::SET_MIT_SETPOINTS, -1, sp));
    }
    sleep_ms(kPreloadSetpointMs);

    std::cout << "[flow] restart all motors" << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::RESTART, -1));
    sleep_ms(kRestartWaitMs);

    // ---- hold current pose, kd=0 ----
    if (!g_should_stop) {
        std::cout << "[test] hold current pose for " << kInitialHoldMs
                  << " ms (kp=" << kDefaultKp << ", kd=0)" << std::endl;

        std::vector<myactua::MitSetpoint> sp;
        sp.reserve(home_deg.size());
        for (double p : home_deg) sp.emplace_back(p, 0.0, 0.0, kDefaultKp, 0.0);

        auto t0 = std::chrono::steady_clock::now();
        auto next = t0;
        while (!g_should_stop) {
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - t0).count() >= kInitialHoldMs)
                break;
            controller.send_command(myactua::ControlCommand(
                myactua::CommandType::SET_MIT_SETPOINTS, -1, sp));
            next += std::chrono::milliseconds(kCommandPeriodMs);
            std::this_thread::sleep_until(next);
        }
    }

    // ---- move to target pose ----
    if (!g_should_stop) {
        std::cout << "[test] move to target pose for " << kMoveDurationMs
                  << " ms (kp=" << kDefaultKp << ", kd=" << kDefaultKd << ")" << std::endl;

        auto t0 = std::chrono::steady_clock::now();
        auto next = t0;
        while (!g_should_stop) {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now() - t0).count();
            if (elapsed_ms >= kMoveDurationMs) break;

            double alpha = static_cast<double>(elapsed_ms) / static_cast<double>(kMoveDurationMs);
            double blend = alpha * alpha * (3.0 - 2.0 * alpha);

            std::vector<myactua::MitSetpoint> sp;
            sp.reserve(home_deg.size());
            for (std::size_t i = 0; i < home_deg.size(); ++i) {
                double p = home_deg[i] + (target_deg[i] - home_deg[i]) * blend;
                sp.emplace_back(p, 0.0, 0.0, kDefaultKp, kDefaultKd);
            }
            controller.send_command(myactua::ControlCommand(
                myactua::CommandType::SET_MIT_SETPOINTS, -1, sp));

            next += std::chrono::milliseconds(kCommandPeriodMs);
            std::this_thread::sleep_until(next);
        }

        if (!g_should_stop) {
            std::vector<myactua::MitSetpoint> sp;
            sp.reserve(target_deg.size());
            for (double p : target_deg) sp.emplace_back(p, 0.0, 0.0, kDefaultKp, kDefaultKd);
            controller.send_command(myactua::ControlCommand(
                myactua::CommandType::SET_MIT_SETPOINTS, -1, sp));
        }
    }

    // ---- hold target pose ----
    if (!g_should_stop) {
        std::cout << "[test] hold target pose for " << kHoldTargetMs
                  << " ms (kp=" << kDefaultKp << ", kd=" << kDefaultKd << ")" << std::endl;

        std::vector<myactua::MitSetpoint> sp;
        sp.reserve(target_deg.size());
        for (double p : target_deg) sp.emplace_back(p, 0.0, 0.0, kDefaultKp, kDefaultKd);

        auto t0 = std::chrono::steady_clock::now();
        auto next = t0;
        while (!g_should_stop) {
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - t0).count() >= kHoldTargetMs)
                break;
            controller.send_command(myactua::ControlCommand(
                myactua::CommandType::SET_MIT_SETPOINTS, -1, sp));
            next += std::chrono::milliseconds(kCommandPeriodMs);
            std::this_thread::sleep_until(next);
        }
    }

    std::cout << "[flow] stop all motors" << std::endl;
    if (!g_should_stop) {
        std::vector<myactua::MitSetpoint> sp;
        sp.reserve(target_deg.size());
        for (double p : target_deg) sp.emplace_back(p, 0.0, 0.0, kDefaultKp, kDefaultKd);
        controller.send_command(myactua::ControlCommand(
            myactua::CommandType::SET_MIT_SETPOINTS, -1, sp));
        sleep_ms(kShutdownHoldMs);
    }
    controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
    sleep_ms(kShutdownHoldMs);
    controller.shutdown();

    std::cout << "[done] mit_pos_test finished" << std::endl;
    return 0;
}
