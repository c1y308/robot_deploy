#include "motor_control.hpp"
#include "EthercatAdapterIGH.hpp"
#include "ControlTypes.hpp"
#include <iostream>  
#include <memory>    
#include <thread>
#include <chrono>

int main() {
    int motors_nums = MYACTUA_NO_FORWARDERS_MOTORS_NUM;
    auto adapter = std::make_shared<myactua::EthercatAdapterIGH>();
    // 实例化控制类，与适配器关联
    myactua::MYACTUA controller(adapter, motors_nums);

    std::cout << "[1/4] 正在初始化网卡..." << std::endl;
    if (!controller.connect("enp8s0")) {
        std::cerr << "[错误] 无法连接到 EtherCAT 网络！" << std::endl;
        return -1;
    }

    std::cout << "[2/4] 等待 " << motors_nums << " 个从站进入 OP..." << std::endl;
    constexpr int kTimeoutMs = 30000;
    constexpr int kPollMs = 100;
    bool all_ready = false;
    const auto ready_wait_start = std::chrono::steady_clock::now();
    const auto deadline = ready_wait_start + std::chrono::milliseconds(kTimeoutMs);
    auto next_log_time = ready_wait_start;
    while (std::chrono::steady_clock::now() < deadline) {
        adapter->receivePhysical();
        adapter->sendPhysical();

        int ready_count = 0;
        for (int i = 0; i < motors_nums; ++i) {
            if (adapter->isConfigured(i)) {
                ++ready_count;
            }
        }
        if (ready_count == motors_nums) {
            all_ready = true;
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - ready_wait_start).count();
            std::cout << "[ready] 全部从站就绪，耗时 " << elapsed_ms << " ms" << std::endl;
            break;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_log_time) {
            std::cout << "  已就绪: " << ready_count << "/" << motors_nums << std::endl;
            next_log_time = now + std::chrono::milliseconds(kPollMs);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (!all_ready) {
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - ready_wait_start).count();
        std::cerr << "[ready] 等待耗时 " << elapsed_ms << " ms" << std::endl;
        std::cerr << "[错误] 从站未在 " << (kTimeoutMs / 1000.0)
                  << "s 内全部就绪，请检查接线/供电/物理位置映射。" << std::endl;
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
