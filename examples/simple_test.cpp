#include "myactua/motor_control.hpp"
#include "myactua/EthercatAdapterIGH.hpp" // 或 EthercatAdapterSOEM
#include <iostream>  
#include <memory>    
#include <vector>
#include <unistd.h>

int main() {
    // 实例化
    auto adapter = std::make_shared<myactua::EthercatAdapterIGH>();
    myactua::MYACTUA controller(adapter, 12);
    std::cout << "[1/3] 正在初始化网卡: " << std::endl;
    // 连接与配置模式
    if (!controller.connect("enp8s0")) {
        std::cerr << "[错误] 无法连接到 EtherCAT 网络！" << std::endl;
        return -1;
    }
    std::cout << "[2/3] 连接成功,正在设置电机1 CSV 模式..." << std::endl;
    controller.set_mode(myactua::ControlMode::CSV, 0);
    controller.set_mode(myactua::ControlMode::CSV, 1);
    std::vector<double> setpoints = { -20.0 , 20.0};

    std::cout << "[3/3] 进入控制循环" << std::endl;
    std::cout << "电机将停止3秒->运行3秒,重复5次" << std::endl;
    
    const int duration_ms = 3000;
    const int total_cycles = 5;
    int counter = 0;
    int cycle_count = 0;
    int phase = 0;
    bool first = true;
    
    controller.stop(-1);
    std::cout << "\n[停止] 停止电机..." << std::endl;
    
    while (cycle_count < total_cycles) {
        int current_phase = (counter / duration_ms) % 2;
        
        if (current_phase != phase) {
            phase = current_phase;
            switch (phase) {
                case 0:
                    std::cout << "\n[停止] 停止电机..." << std::endl;
                    //controller.stop(-1);
                    cycle_count++;
                    break;
                case 1:
                    std::cout << "\n[运行] 启动电机..." << std::endl;
                    //controller.restart(-1);
                    break;
            }
        }
        
        controller.update(setpoints);
        usleep(1000);
        counter++;
    }

    static int once = 1;
    if(once == 1){
        controller.stop(-1);
        once = 0;
    }
    while(true){
        controller.update(setpoints);
        usleep(1000);
    }
    //std::cout << "\n[完成] 已完成5次循环，程序结束。" << std::endl;
    //return 0;
}

