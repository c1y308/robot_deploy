#include "driver/myact/motor_control.hpp"
#include "EthercatAdapterIGH.hpp"
#include "motor_base/ControlTypes.hpp"
#include <iostream>  
#include <memory>    
#include <thread>
#include <chrono>

int main() {
    auto adapter = std::make_shared<myactua::EthercatAdapterIGH>();
    myactua::MYACTUA controller(adapter, 1);

    std::cout << "正在初始化网卡..." << std::endl;
    if (!controller.connect("enp8s0")) {
        std::cerr << "[错误] 无法连接到 EtherCAT 网络！" << std::endl;
        return -1;
    }

    if (!controller.wait_all_motors_ready(20000, 100)) {
        return -1;
    }

    std::cout << "启动实时控制线程..." << std::endl;
    controller.start();

    std::cout << "\n========== 控制流程开始 ==========" << std::endl;
    
    // 1. 停止 1 秒
    std::cout << "[阶段1] 停止电机，等待 1 秒..." << std::endl;
    controller.send_command(motor_base::ControlCommand::stop());
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 2. 以速度 50 运行 5 秒
    std::cout << "[阶段2] 正在进行RESTART..." << std::endl;
    controller.send_command(motor_base::ControlCommand::restart());
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 3. 停止
    std::cout << "[阶段3] 停止电机..." << std::endl;
    controller.send_command(motor_base::ControlCommand::stop());
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    std::cout << "正在设置电机 CSP 模式..." << std::endl;
    controller.send_command(motor_base::ControlCommand::set_mode(motor_base::MotorControlMode::POSITION));
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "电机以位置模式运行复位..." << std::endl;
    controller.send_command(motor_base::ControlCommand::restart());
    controller.send_command(motor_base::ControlCommand::set_position_targets_rad(std::vector<double>{0}));
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // 3. 停止
    std::cout << "停止电机..." << std::endl;
    controller.send_command(motor_base::ControlCommand::stop());
    std::this_thread::sleep_for(std::chrono::seconds(1));


    std::cout << "\n========== 控制流程结束 ==========" << std::endl;
    // controller.shutdown();
    std::cout << "[完成] 程序结束。" << std::endl;
    return 0;
}
