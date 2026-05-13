#include "motor_control.hpp"
#include "EthercatAdapterIGH.hpp"
#include "ControlTypes.hpp"
#include <iostream>  
#include <memory>    
#include <thread>
#include <chrono>


int main() {
    int motors_nums = 1;
    auto adapter = std::make_shared<myactua::EthercatAdapterIGH>();
    // 实例化控制类，与适配器关联
    myactua::MYACTUA controller(adapter, motors_nums);

    std::cout << "[1/4] 正在初始化网卡..." << std::endl;
    if (!controller.connect("enp8s0")) {
        std::cerr << "[错误] 无法连接到 EtherCAT 网络！" << std::endl;
        return -1;
    }

    std::cout << "[2/4] 等待从站进入 OP..." << std::endl;
    if (!controller.wait_all_slaves_ready(30000, 100)) {
        std::cerr << "[错误] 从站未在超时时间内全部就绪，请检查接线/供电/物理位置映射。"
                  << std::endl;
        return -1;
    }

    std::cout << "[3/4] 连接成功，正在设置电机 CSP 模式..." << std::endl;
    for(int i = 0; i < motors_nums; ++i) {
        controller.send_command(myactua::ControlCommand(myactua::CommandType::SET_MODE,  i, {}, myactua::ControlMode::CSP));
    }

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
