#include "motor_base/CommandTypes.hpp"
#include "EthercatAdapterIGH.hpp"
#include "driver/myact/motor_control.hpp"

#include <chrono>
#include <csignal>
#include <cstddef>
#include <cmath>
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

// 12 个电机的位置目标，单位为 deg；下发前转换为公共 API 使用的 rad。
// const std::vector<double> kTargetPositionsDeg = {
//     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//     0.0, 0.0, 0.0, 0.0, 0.0, 0.0
// };
constexpr int kNumMotors = 12;
const std::vector<double> kTargetPositionsDeg = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};

constexpr double kPi = 3.14159265358979323846;

void signal_handler(int)
{
    g_should_stop = 1;
}

void send_mode_all(myactua::MYACTUA& controller, motor_base::MotorControlMode mode)
{
    for (int i = 0; i < kNumMotors; ++i) {
        controller.send_command(motor_base::ControlCommand::set_mode(mode, i));
    }
}

std::vector<double> deg_to_rad_vector(const std::vector<double>& values_deg)
{
    std::vector<double> values_rad(values_deg.size(), 0.0);
    for (std::size_t i = 0; i < values_deg.size(); ++i) {
        values_rad[i] = values_deg[i] * kPi / 180.0;
    }
    return values_rad;
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
        motor_base::ControlCommand::set_position_targets_rad(deg_to_rad_vector(target_deg)));
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

    if (!controller.wait_all_motors_ready(
            kWaitReadyTimeoutMs, kWaitReadyPollMs, [] { return g_should_stop != 0; })) {
        std::cerr << "[error] not all slaves reached OP within "
                  << kWaitReadyTimeoutMs << " ms" << std::endl;
        return -1;
    }

    controller.set_print_info({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});
    controller.start();

    std::cout << "[flow] stop all motors before mode switch" << std::endl;
    for(int i = 0; i < kNumMotors; i++){
        controller.send_command(motor_base::ControlCommand::stop(i));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kWarmupMs));


    std::cout << "[flow] switch all motors to CSP" << std::endl;
    for (int i = 0; i < kNumMotors; ++i) {
        controller.send_command(
            motor_base::ControlCommand::set_mode(motor_base::MotorControlMode::POSITION, i));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kModeSwitchWaitMs));


    std::cout << "[flow] restart motors" << std::endl;
    for(int i = 0; i < kNumMotors; i++){
        controller.send_command(motor_base::ControlCommand::restart(i));
    }
    controller.send_command(motor_base::ControlCommand::stop(0));
    std::this_thread::sleep_for(std::chrono::milliseconds(kRestartWaitMs));


    const bool target_sent = send_target_once(controller, kTargetPositionsDeg);
    if (target_sent) {
        std::cout << "[hold] keep target pose for " << kHoldTargetMs << " ms" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(kHoldTargetMs));
    }

    std::cout << "[flow] stop all motors" << std::endl;
    for(int i = 0; i < kNumMotors; i++){
        controller.send_command(motor_base::ControlCommand::stop(i));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    controller.shutdown();


    std::cout << "[done] raw_pos_test finished" << std::endl;
    return 0;
}
