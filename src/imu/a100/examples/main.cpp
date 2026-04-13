#include "imu_reader.hpp"
#include "motor_control.hpp"
#include "EthercatAdapterIGH.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

using namespace imu;

namespace {
std::atomic<bool> g_running{true};

IMUReader* g_reader = nullptr;
std::unique_ptr<myactua::MYACTUA> g_motor_controller;

std::array<double, 3> g_gyro_rad_s{0.0, 0.0, 0.0};
std::mutex g_imu_mutex;

constexpr int kNumMotors = 12;
constexpr const char* kIfname = "enp8s0";


void signal_handler(int) {
    g_running.store(false);
    if (g_reader) {
        g_reader->stop();
    }
    if (g_motor_controller) {
        g_motor_controller->send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
        g_motor_controller->shutdown();
    }
    std::cout << "\n[INFO] Exiting..." << std::endl;
}


bool wait_all_slaves_ready(const std::shared_ptr<myactua::EthercatAdapterIGH>& adapter,
                           int timeout_ms = 10000,
                           int poll_ms = 100) {
    const int max_tries = (timeout_ms + poll_ms - 1) / poll_ms;
    for (int t = 0; t < max_tries; ++t) {
        int ready_count = 0;
        for (int i = 0; i < kNumMotors; ++i) {
            if (adapter->isConfigured(i)) {
                ++ready_count;
            }
        }
        if (ready_count == kNumMotors) {
            std::cout << "[MOTOR] All slaves ready: " << ready_count << "/" << kNumMotors << std::endl;
            return true;
        }
        std::cout << "[MOTOR] Ready: " << ready_count << "/" << kNumMotors << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
    }
    return false;
}
}  // namespace


int main() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // 1) 初始化 IMU（默认配置）
    Config_t imu_config;
    IMUReader reader;
    g_reader = &reader;

    reader.set_imu_callback([](const IMUData_t& imu_data) {
        std::lock_guard<std::mutex> lock(g_imu_mutex);
        g_gyro_rad_s[0] = static_cast<double>(imu_data.gyroscope_x);
        g_gyro_rad_s[1] = static_cast<double>(imu_data.gyroscope_y);
        g_gyro_rad_s[2] = static_cast<double>(imu_data.gyroscope_z);
    });

    if (!reader.initialize(imu_config)) {
        std::cerr << "[ERROR] IMU initialize failed." << std::endl;
        return -1;
    }

    std::thread imu_thread([&reader]() { reader.run(); });

    // 2) 初始化电机（固定12个）
    auto adapter = std::make_shared<myactua::EthercatAdapterIGH>();
    g_motor_controller = std::make_unique<myactua::MYACTUA>(adapter, kNumMotors);

    std::cout << "[MOTOR] Connecting EtherCAT on " << kIfname << "..." << std::endl;
    if (!g_motor_controller->connect(kIfname)) {
        std::cerr << "[ERROR] EtherCAT connect failed." << std::endl;
        g_running.store(false);
    } else if (!wait_all_slaves_ready(adapter)) {
        std::cerr << "[ERROR] Not all slaves ready in timeout." << std::endl;
        g_running.store(false);
    } else {
        // 3) 上电初始化后全部置为 STOP 状态
        g_motor_controller->start();
        g_motor_controller->send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
        std::cout << "[MOTOR] Motors initialized and set to STOP." << std::endl;
    }

    // 4) 只做数据读取打印
    while (g_running.load()) {
        std::array<double, 3> gyro;
        {
            std::lock_guard<std::mutex> lock(g_imu_mutex);
            gyro = g_gyro_rad_s;
        }

        if (g_motor_controller) {
            const auto status = g_motor_controller->get_status();
            if (!status.empty()) {
                const auto& m0 = status[0];
                std::cout << "[DATA] gyro(rad/s)=[" << gyro[0] << ", " << gyro[1] << ", " << gyro[2]
                          << "] | M0 pos=" << m0.position
                          << " vel=" << m0.velocity
                          << " tau=" << m0.torque
                          << " sw=0x" << std::hex << m0.status_word << std::dec
                          << std::endl;
            } else {
                std::cout << "[DATA] gyro(rad/s)=[" << gyro[0] << ", " << gyro[1] << ", " << gyro[2]
                          << "] | motor status empty" << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    reader.stop();
    if (imu_thread.joinable()) {
        imu_thread.join();
    }

    if (g_motor_controller) {
        g_motor_controller->send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        g_motor_controller->shutdown();
        g_motor_controller.reset();
    }

    return 0;
}

