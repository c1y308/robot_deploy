#include "motor_control.hpp"
#include "EthercatAdapterIGH.hpp"
#include "ControlTypes.hpp"
#include <iostream>  
#include <memory>    
#include <thread>
#include <chrono>

int main() {
    auto adapter = std::make_shared<myactua::EthercatAdapterIGH>();
    myactua::MYACTUA controller(adapter, 6);

    std::cout << "[1/3] 正在初始化网卡..." << std::endl;
    if (!controller.connect("enp8s0")) {
        std::cerr << "[错误] 无法连接到 EtherCAT 网络！" << std::endl;
        return -1;
    }

    std::cout << "[2/3] 连接成功，正在设置电机 CSV 模式..." << std::endl;
    controller.set_mode(myactua::ControlMode::CSV, 0);
    controller.set_mode(myactua::ControlMode::CSV, 1);
    controller.set_mode(myactua::ControlMode::CSV, 2);
    controller.set_mode(myactua::ControlMode::CSV, 3);
    controller.set_mode(myactua::ControlMode::CSV, 4);
    controller.set_mode(myactua::ControlMode::CSV, 5);
    controller.set_mode(myactua::ControlMode::CSV, 6);
    controller.set_mode(myactua::ControlMode::CSV, 7);
    controller.set_mode(myactua::ControlMode::CSV, 8);
    controller.set_mode(myactua::ControlMode::CSV, 9);
    controller.set_mode(myactua::ControlMode::CSV, 10);
    controller.set_mode(myactua::ControlMode::CSV, 11);

    std::cout << "[3/3] 启动实时控制线程..." << std::endl;
    controller.start();

    std::cout << "\n========== 控制流程开始 ==========" << std::endl;
    
    // 1. 停止 1 秒
    std::cout << "[阶段1] 停止电机，等待 1 秒..." << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 2. 以速度 50 运行 5 秒
    std::cout << "[阶段2] 电机以速度 50 运行，等待 5 秒..." << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::RESTART, -1));
    controller.send_command(myactua::ControlCommand(
    myactua::CommandType::SET_SETPOINTS, -1, {50, 50}));
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // 3. 停止
    std::cout << "[阶段3] 停止电机..." << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    std::cout << "\n========== 控制流程结束 ==========" << std::endl;
    // controller.shutdown();
    std::cout << "[完成] 程序结束。" << std::endl;
    return 0;
}
