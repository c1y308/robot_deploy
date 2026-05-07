#include "ControlTypes.hpp"
#include "EthercatAdapterIGH.hpp"
#include "motor_control.hpp"

#include <chrono>
#include <csignal>
#include <cstddef>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

volatile std::sig_atomic_t g_should_stop = 0;

constexpr char kIfName[] = "enp8s0";

constexpr int kWaitReadyTimeoutMs = 20000;
constexpr int kWaitReadyPollMs = 100;

constexpr int kWarmupMs = 1000;
constexpr int kModeSwitchWaitMs = 1000;
constexpr int kRestartWaitMs = 1000;
constexpr int kHoldTargetMs = 20000;

// 12 个电机的位置目标，单位为 deg。
// 注意: 当前 MYACTUA API 在 CSP 模式下通过 SET_SETPOINTS 接收角度值(deg)。
// const std::vector<double> kTargetPositionsDeg = {
//     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//     0.0, 0.0, 0.0, 0.0, 0.0, 0.0
// };
constexpr int kNumMotors = 12;
const std::vector<double> kTargetPositionsDeg = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0
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
            std::cout << "[ready] " << ready_count << "/" << kNumMotors
                      << " slaves ready" << std::endl;
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

std::vector<double> rad_to_deg_vector(const std::vector<double>& values_rad)
{
    std::vector<double> values_deg(values_rad.size(), 0.0);
    for (std::size_t i = 0; i < values_rad.size(); ++i) {
        values_deg[i] = myactua::MYACTUA::rad_to_deg(values_rad[i]);
    }
    return values_deg;
}


bool send_target_once(myactua::MYACTUA& controller, const std::vector<double>& target_deg)
{
    if (target_deg.size() != static_cast<std::size_t>(kNumMotors)) {
        std::cerr << "[error] target vector size mismatch" << std::endl;
        return false;
    }

    std::cout << "[motion] send target array to " << kNumMotors
              << " motors without interpolation" << std::endl;
    controller.send_command(
        myactua::ControlCommand(myactua::CommandType::SET_SETPOINTS, -1, target_deg));
    return !g_should_stop;
}

}  // namespace

int main()
{
    std::signal(SIGINT, signal_handler);

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

    std::cout << "[flow] stop all motors before mode switch" << std::endl;
    for(int i = 0; i < kNumMotors; i++){
        controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, i));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kWarmupMs));


    std::cout << "[flow] switch all motors to CSP" << std::endl;
    for (int i = 0; i < kNumMotors; ++i) {
        controller.send_command(
            myactua::ControlCommand(myactua::CommandType::SET_MODE, i, {}, myactua::ControlMode::CSP));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kModeSwitchWaitMs));


    std::cout << "[flow] restart motors" << std::endl;
    for(int i = 0; i < kNumMotors; i++){
        controller.send_command(myactua::ControlCommand(myactua::CommandType::RESTART, i));
    }
    controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, 0));
    std::this_thread::sleep_for(std::chrono::milliseconds(kRestartWaitMs));


    const bool target_sent = send_target_once(controller, kTargetPositionsDeg);
    if (target_sent) {
        std::cout << "[hold] keep target pose for " << kHoldTargetMs << " ms" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(kHoldTargetMs));
    }

    std::cout << "[flow] stop all motors" << std::endl;
    for(int i = 0; i < kNumMotors; i++){
        controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, i));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    controller.shutdown();


    std::cout << "[done] raw_pos_test finished" << std::endl;
    return 0;
}
