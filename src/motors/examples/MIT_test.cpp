#include "motor_control.hpp"
#include "EthercatAdapterIGH.hpp"
#include "ControlTypes.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

namespace {

volatile std::sig_atomic_t g_should_stop = 0;

constexpr char kIfName[] = "enp8s0";
constexpr int kNumMotors = 1;
constexpr int kMotorIndex = 0;  // 电机索引 ID

constexpr int kWaitReadyTimeoutMs = 20000;
constexpr int kWaitReadyPollMs = 100;

constexpr int kWarmupMs = 1000;

constexpr int kModeSwitchWaitMs = 1500;
constexpr int kRestartWaitMs = 1000;

constexpr double kDefaultKp = 250;
constexpr double kDefaultKd = 10;

void signal_handler(int)
{
    g_should_stop = 1;
}

void sleep_ms(int ms)
{
    if (g_should_stop) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

}  // namespace

int main()
{
    std::signal(SIGINT, signal_handler);

    auto adapter = std::make_shared<myactua::EthercatAdapterIGH>();
    myactua::MYACTUA controller(adapter, kNumMotors);

    if (!controller.connect(kIfName)) {
        std::cerr << "[ERROR] EtherCAT " << std::endl;
        return -1;
    }

    std::cout << "[2/7] OP ..." << std::endl;
    if (!controller.wait_all_slaves_ready(kWaitReadyTimeoutMs, kWaitReadyPollMs,
                                          [] { return g_should_stop != 0; })) {
        std::cerr << "[ERROR] OP " << std::endl;
        return -1;
    }

    controller.set_print_info({kMotorIndex});
    controller.start();

    std::cout << "[3/7] ..." << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
    sleep_ms(kWarmupMs);

    std::cout << "[4/7] MIT/PVT ..." << std::endl;
    controller.send_command(myactua::ControlCommand(
        myactua::CommandType::SET_MODE, -1, {}, myactua::ControlMode::PVT));
    sleep_ms(kModeSwitchWaitMs);

    std::vector<double> init_pos = controller.get_joint_q_rad();
    if (init_pos.empty()) {
        std::cerr << "[ERROR] " << std::endl;
        controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
        controller.shutdown();
        return -1;
    }
    double home_deg = myactua::MYACTUA::rad_to_deg(init_pos[kMotorIndex]);
    std::cout << "[INFO] home = " << home_deg << " deg" << std::endl;

    std::cout << "[5/7] ..." << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::RESTART, -1));
    sleep_ms(kRestartWaitMs);

    constexpr double kTargetDeg = 60.0;
    constexpr int kCommandPeriodMs = 10;
    constexpr int kHoldMs = 1000;
    constexpr int kMoveMs = 3000;

    std::cout << "\n========================================" << std::endl;
    std::cout << "  MIT  - hold initial pose (kp=" << kDefaultKp
              << ", kd=" << kDefaultKd << ")" << std::endl;
    std::cout << "========================================\n" << std::endl;

    {
        auto t0 = std::chrono::steady_clock::now();
        auto next = t0;
        while (!g_should_stop) {
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - t0).count() >= kHoldMs)
                break;

            myactua::MitSetpoint sp(home_deg, 0.0, 0.0, kDefaultKp, kDefaultKd);
            controller.send_command(myactua::ControlCommand(
                myactua::CommandType::SET_MIT_SETPOINTS, -1, {sp}));
            next += std::chrono::milliseconds(kCommandPeriodMs);
            std::this_thread::sleep_until(next);
        }
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "  MIT  - move +" << kTargetDeg << " deg (kp=" << kDefaultKp
              << ", kd=" << kDefaultKd << ")" << std::endl;
    std::cout << "========================================\n" << std::endl;

    {
        double target_deg = home_deg + kTargetDeg;
        auto t0 = std::chrono::steady_clock::now();
        auto next = t0;
        while (!g_should_stop) {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now() - t0).count();
            if (elapsed_ms >= kMoveMs) break;

            double alpha = static_cast<double>(elapsed_ms) / static_cast<double>(kMoveMs);
            double blend = alpha * alpha * (3.0 - 2.0 * alpha);
            double p = home_deg + (target_deg - home_deg) * blend;

            myactua::MitSetpoint sp(p, 0.0, 0.0, kDefaultKp, kDefaultKd);
            controller.send_command(myactua::ControlCommand(
                myactua::CommandType::SET_MIT_SETPOINTS, -1, {sp}));
            next += std::chrono::milliseconds(kCommandPeriodMs);
            std::this_thread::sleep_until(next);
        }
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "  MIT  - return to home (kp=" << kDefaultKp
              << ", kd=" << kDefaultKd << ")" << std::endl;
    std::cout << "========================================\n" << std::endl;

    {
        double start_deg = home_deg + kTargetDeg;
        auto t0 = std::chrono::steady_clock::now();
        auto next = t0;
        while (!g_should_stop) {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now() - t0).count();
            if (elapsed_ms >= kMoveMs) break;

            double alpha = static_cast<double>(elapsed_ms) / static_cast<double>(kMoveMs);
            double blend = alpha * alpha * (3.0 - 2.0 * alpha);
            double p = start_deg + (home_deg - start_deg) * blend;

            myactua::MitSetpoint sp(p, 0.0, 0.0, kDefaultKp, kDefaultKd);
            controller.send_command(myactua::ControlCommand(
                myactua::CommandType::SET_MIT_SETPOINTS, -1, {sp}));
            next += std::chrono::milliseconds(kCommandPeriodMs);
            std::this_thread::sleep_until(next);
        }
    }

    std::cout << "\n[6/7] hold home pose" << std::endl;
    {
        myactua::MitSetpoint sp(home_deg, 0.0, 0.0, kDefaultKp, kDefaultKd);
        controller.send_command(myactua::ControlCommand(
            myactua::CommandType::SET_MIT_SETPOINTS, -1, {sp}));
    }
    sleep_ms(500);

    std::cout << "[7/7] stop" << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
    sleep_ms(500);

    controller.shutdown();

    std::cout << "\n========== MIT  ==========" << std::endl;
    return 0;
}