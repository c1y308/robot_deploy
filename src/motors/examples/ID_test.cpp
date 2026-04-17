#include "motor_control.hpp"
#include "EthercatAdapterIGH.hpp"
#include "ControlTypes.hpp"
#include <iostream>  
#include <memory>    
#include <thread>
#include <chrono>

int main() {
    auto adapter = std::make_shared<myactua::EthercatAdapterIGH>();
    // 实例化控制类，与适配器关联
    myactua::MYACTUA controller(adapter, 12);

    std::cout << "[1/4] 正在初始化网卡..." << std::endl;
    if (!controller.connect("enp8s0")) {
        std::cerr << "[错误] 无法连接到 EtherCAT 网络！" << std::endl;
        return -1;
    }

    std::cout << "[2/4] 等待 12 个从站进入 OP..." << std::endl;
    constexpr int kWaitCycles = 200; // 10s timeout (100 * 100ms)
    bool all_ready = false;
    for (int t = 0; t < kWaitCycles; ++t) {
        int ready_count = 0;
        for (int i = 0; i < 12; ++i) {
            if (adapter->isConfigured(i)) {
                ++ready_count;
            }
        }
        if (ready_count == 12) {
            all_ready = true;
            break;
        }
        std::cout << "  已就绪: " << ready_count << "/12" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!all_ready) {
        std::cerr << "[错误] 从站未在 10s 内全部就绪，请检查接线/供电/物理位置映射。" << std::endl;
        return -1;
    }

    std::cout << "[3/4] 连接成功，正在设置电机 CSV 模式..." << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::SET_MODE, 0, {}, myactua::ControlMode::CSV));
    controller.send_command(myactua::ControlCommand(myactua::CommandType::SET_MODE, 1, {}, myactua::ControlMode::CSV));
    controller.send_command(myactua::ControlCommand(myactua::CommandType::SET_MODE, 2, {}, myactua::ControlMode::CSV));
    controller.send_command(myactua::ControlCommand(myactua::CommandType::SET_MODE, 3, {}, myactua::ControlMode::CSV));
    controller.send_command(myactua::ControlCommand(myactua::CommandType::SET_MODE, 4, {}, myactua::ControlMode::CSV));
    controller.send_command(myactua::ControlCommand(myactua::CommandType::SET_MODE, 5, {}, myactua::ControlMode::CSV));
    controller.send_command(myactua::ControlCommand(myactua::CommandType::SET_MODE, 6, {}, myactua::ControlMode::CSV));
    controller.send_command(myactua::ControlCommand(myactua::CommandType::SET_MODE, 7, {}, myactua::ControlMode::CSV));
    controller.send_command(myactua::ControlCommand(myactua::CommandType::SET_MODE, 8, {}, myactua::ControlMode::CSV));
    controller.send_command(myactua::ControlCommand(myactua::CommandType::SET_MODE, 9, {}, myactua::ControlMode::CSV));
    controller.send_command(myactua::ControlCommand(myactua::CommandType::SET_MODE, 10, {}, myactua::ControlMode::CSV));
    controller.send_command(myactua::ControlCommand(myactua::CommandType::SET_MODE, 11, {}, myactua::ControlMode::CSV));

    std::cout << "[4/4] 启动实时控制线程..." << std::endl;
    controller.start();
    std::cout << "\n========== 控制流程开始 ==========" << std::endl;
    
    std::cout << "[阶段1] 停止电机，等待 3 秒..." << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
    std::this_thread::sleep_for(std::chrono::seconds(3));

    while(true);
    // controller.shutdown();
    std::cout << "[完成] 程序结束。" << std::endl;
    return 0;
}
