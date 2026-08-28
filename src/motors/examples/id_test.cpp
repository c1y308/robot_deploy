#include "driver/myact/motor_control.hpp"
#include "protocol/ethercat/ethercat_adapter_igh.hpp"
#include "motor_base/command_types.hpp"
#include "motor_base/rt_event_dispatcher.hpp"
#include <array>
#include <cstddef>
#include <iostream>  
#include <memory>    
#include <thread>
#include <chrono>

namespace {

bool wait_all_position_running(myactua::MYACTUA& controller,
                               int motor_count,
                               std::chrono::milliseconds timeout,
                               std::chrono::milliseconds poll_interval)
{
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
        const auto status = controller.get_status();
        bool all_running = status.size() == static_cast<std::size_t>(motor_count);

        for (int i = 0; all_running && i < motor_count; ++i) {
            const auto& motor = status[static_cast<std::size_t>(i)];
            all_running = motor.comm_ok &&
                          motor.enabled &&
                          motor.control_ready &&
                          motor.mode == motor_base::MotorControlMode::POSITION &&
                          motor.target_mode == motor_base::MotorControlMode::POSITION;
        }

        if (all_running) {
            return true;
        }

        std::this_thread::sleep_for(poll_interval);
    }

    return false;
}

}

int main() {
    constexpr int motors_nums = 12;
    auto adapter = std::make_shared<myactua::EthercatAdapterIGH>();
    // 实例化控制类，与适配器关联
    myactua::MYACTUA controller(adapter, motors_nums);
    controller.set_print_info({-1});
    controller.set_event_callback([](const motor_base::RtEvent& event) {
        if (event.type == motor_base::RtEventType::STATUS_CHANNEL_BUSY) {
            return;
        }
        std::cerr << "[MYACTUA] rt event type=" << static_cast<int>(event.type)
                  << ", motor=" << event.motor_index
                  << ", reason=" << event.reason
                  << ", value=" << event.value << std::endl;
    });
    
    std::cout << "[1/4] 正在初始化网卡..." << std::endl;
    if (!controller.connect("enp8s0")) {
        std::cerr << "[错误] 无法连接到 EtherCAT 网络！" << std::endl;
        return -1;
    }

    std::cout << "[2/4] 等待从站进入 OP..." << std::endl;
    if (!controller.wait_all_motors_ready(30000, 100)) {
        std::cerr << "[错误] 从站未在超时时间内全部就绪，请检查接线/供电/物理位置映射。"
                  << std::endl;
        return -1;
    }

    std::cout << "[3/4] 连接成功，正在设置电机 CSP 模式..." << std::endl;
    for(int i = 0; i < motors_nums; ++i) {
        controller.send_command(
            motor_base::ControlCommand::set_mode(motor_base::MotorControlMode::POSITION, i));
    }

    std::cout << "[4/4] 启动实时控制线程..." << std::endl;
    if (!controller.start()) {
        std::cerr << "[错误] 实时调度未激活，拒绝进入运动控制。" << std::endl;
        return -1;
    }
    std::cout << "\n========== 控制流程开始 ==========" << std::endl;
    
    std::cout << "[阶段1] 停止电机，等待 3 秒..." << std::endl;
    controller.send_command(motor_base::ControlCommand::stop());
    std::this_thread::sleep_for(std::chrono::seconds(3));

    /********************************************************************************** */
    std::cout << "[阶段2] 重新启动电机，等待 POSITION 模式就绪..." << std::endl;
    if (controller.send_command(motor_base::ControlCommand::restart()) !=
        motor_base::CommandSubmitResult::ACCEPTED) {
        std::cerr << "[错误] 电机重新启动命令提交失败。" << std::endl;
        return -1;
    }
    if (!wait_all_position_running(controller,
                                   motors_nums,
                                   std::chrono::seconds(5),
                                   std::chrono::milliseconds(20))) {
        std::cerr << "[错误] 电机未在超时时间内进入 POSITION 运行状态。" << std::endl;
        return -1;
    }

    const std::array<double, motors_nums> zero_positions_rad = {};

    const std::array<double, motors_nums> target_positions_rad = {
        0.0, 0.0, 0.0, 0.0, 0.0, -3.14,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    };

    std::cout << "[阶段3] 所有电机回到零位..." << std::endl;
    if (controller.send_command(
            motor_base::ControlCommand::set_position_targets_rad_fixed(
                zero_positions_rad.data(),
                zero_positions_rad.size())) !=
        motor_base::CommandSubmitResult::ACCEPTED) {
        std::cerr << "[错误] 零位目标提交失败。" << std::endl;
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::cout << "[阶段4] 下发一次目标位置数组..." << std::endl;
    if (controller.send_command(
            motor_base::ControlCommand::set_position_targets_rad_fixed(
                target_positions_rad.data(),
                target_positions_rad.size())) !=
        motor_base::CommandSubmitResult::ACCEPTED) {
        std::cerr << "[错误] 目标位置数组提交失败。" << std::endl;
        return -1;
    }
    /********************************************************************************** */



    while(true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    // controller.shutdown();
    std::cout << "[完成] 程序结束。" << std::endl;
    return 0;
}
