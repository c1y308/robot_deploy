#include "motor_control.hpp"
#include "EthercatAdapterIGH.hpp"
#include "ControlTypes.hpp"
#include <iostream>  
#include <memory>    
#include <thread>
#include <chrono>

int main() {
    auto adapter = std::make_shared<myactua::EthercatAdapterIGH>();
    myactua::MYACTUA controller(adapter, 2);

    std::cout << "正在初始化网卡..." << std::endl;
    if (!controller.connect("enp8s0")) {
        std::cerr << "[错误] 无法连接到 EtherCAT 网络！" << std::endl;
        return -1;
    }
    const auto start_time = std::chrono::steady_clock::now();
    const int timeout_ms = 20000;
    const int poll_ms    = 100;
    const int max_tries  = (timeout_ms == 0) ? 1 : (timeout_ms + poll_ms - 1) / poll_ms;
    for (int t = 0; t < max_tries; ++t) {
        int ready_count = 0;
        for (int i = 0; i < 2; ++i) {
            if (adapter->isConfigured(i)) {
                ++ready_count;
            }
        }
        if (ready_count == 2) {
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time).count();
            std::cout << "[RobotInterface] All slaves ready in "
                      << elapsed_ms << " ms\n";
            break;
        }

        std::cout << "[RobotInterface] EtherCAT ready: "
                  << ready_count << "/" << 2 << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
    }
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time).count();
    std::cout << "[RobotInterface] wait_all_slaves_ready timeout after "
              << elapsed_ms << " ms\n";


    std::cout << "启动实时控制线程..." << std::endl;
    controller.start();

    std::cout << "\n========== 控制流程开始 ==========" << std::endl;
    
    // 1. 停止 1 秒
    std::cout << "[阶段1] 停止电机，等待 1 秒..." << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 2. 以速度 50 运行 5 秒
    std::cout << "[阶段2] 正在进行RESTART..." << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::RESTART, -1));
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "正在设置电机 CSV 模式..." << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::SET_MODE,
                                                     0,
                                                     {},
                                                     myactua::ControlMode::CSV));
    controller.send_command(myactua::ControlCommand(myactua::CommandType::SET_MODE,
                                                     1,
                                                     {},
                                                     myactua::ControlMode::CSV));
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "电机以 50 速度运行 5 秒..." << std::endl;
    controller.send_command(myactua::ControlCommand(
    myactua::CommandType::SET_SETPOINTS, -1, {50, 50}));
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // 3. 停止
    std::cout << "[阶段3] 停止电机..." << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    std::cout << "正在设置电机 CSP 模式..." << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::SET_MODE,
                                                     0,
                                                     {},
                                                     myactua::ControlMode::CSP));
    controller.send_command(myactua::ControlCommand(myactua::CommandType::SET_MODE,
                                                     1,
                                                     {},
                                                     myactua::ControlMode::CSP));
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "电机以位置模式运行复位..." << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::RESTART, -1));
    controller.send_command(myactua::ControlCommand(
    myactua::CommandType::SET_SETPOINTS, -1, {0, 0}));
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // 3. 停止
    std::cout << "停止电机..." << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
    std::this_thread::sleep_for(std::chrono::seconds(1));


    std::cout << "\n========== 控制流程结束 ==========" << std::endl;
    // controller.shutdown();
    std::cout << "[完成] 程序结束。" << std::endl;
    return 0;
}
