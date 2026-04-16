#include "motor_control.hpp"
#include "EthercatAdapterIGH.hpp"
#include "ControlTypes.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace {

const char* mode_to_string(myactua::ControlMode mode)
{
    switch (mode) {
        case myactua::ControlMode::NONE:
            return "NONE";
        case myactua::ControlMode::CSP:
            return "CSP";
        case myactua::ControlMode::CSV:
            return "CSV";
        case myactua::ControlMode::CST:
            return "CST";
        default:
            return "UNKNOWN";
    }
}

bool wait_all_slaves_ready(const std::shared_ptr<myactua::EthercatAdapterIGH>& adapter,
                           int num_motors,
                           int timeout_ms = 20000,
                           int poll_ms = 100)
{
    const int max_tries = (timeout_ms + poll_ms - 1) / poll_ms;

    for (int t = 0; t < max_tries; ++t) {
        int ready_count = 0;
        for (int i = 0; i < num_motors; ++i) {
            if (adapter->isConfigured(i)) {
                ++ready_count;
            }
        }

        std::cout << "[连接检查] EtherCAT ready: "
                  << ready_count << "/" << num_motors << std::endl;
        if (ready_count == num_motors) {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
    }

    return false;
}

}  // namespace

int main(int argc, char** argv)
{
    const std::string ifname = "enp8s0";
    const int num_motors =  2;

    auto adapter = std::make_shared<myactua::EthercatAdapterIGH>();
    myactua::MYACTUA controller(adapter, num_motors);

    std::cout << "[1/5] 正在连接 EtherCAT..." << std::endl;
    if (!controller.connect(ifname.c_str())) {
        std::cerr << "[错误] 无法连接到 EtherCAT 网络！" << std::endl;
        return -1;
    }

    std::cout << "[2/5] 等待从站就绪..." << std::endl;
    if (!wait_all_slaves_ready(adapter, num_motors)) {
        std::cerr << "[错误] 超时：未等到所有从站就绪。" << std::endl;
        return -1;
    }

    std::cout << "[3/5] 启动实时线程..." << std::endl;
    controller.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << "[4/5] 下发 STOP(广播到全部电机)..." << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "[5/5] 重启..." << std::endl;
    controller.send_command(myactua::ControlCommand(myactua::CommandType::RESTART, -1));
    std::this_thread::sleep_for(std::chrono::seconds(10));
    controller.shutdown();
    std::cout << "\n[完成] 已执行 STOP 并完成状态读取。" << std::endl;
    return 0;
}
