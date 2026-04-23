#pragma once
#include "EthercatAdapterIGH.hpp"
#include "imu_reader.hpp"
#include "motor_control.hpp"

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace myactua {
class EthercatAdapterIGH;
class MYACTUA;
}

namespace imu {
class IMUReader;
}

namespace inference {

/* 接口层的配置结构体 */
struct RobotInterfaceConfig {

    /* 电机 配置*/
    int num_motors = 2;
    std::string ethercat_ifname = "enp8s0";
    /* 等待所有从站就绪的超时和轮询时间 */
    int wait_all_slaves_timeout_ms = 20000;
    int wait_all_slaves_poll_ms = 100;
    /* 是否在终端打印电机信息 */
    bool print_motors_info = true;  // false则将 print_motor_ids 清零
    std::vector<int> print_motor_ids = {0, 1};  // 仅打印这些ID的电机信息，ID从0开始，-1表示全部
    /* 电机转动角度限制 */
    std::vector<double> joint_min_rad;
    std::vector<double> joint_max_rad;
    /* 初始姿态各电机的角度值 */
    std::vector<double> stand_pose_rad;



    /* IMU 配置 */
    std::string imu_device = "/dev/ttyUSB0";
    int imu_baudrate       = 921600;
    bool imu_print_imu     = false;
    bool imu_print_ahrs    = true;
    bool imu_print_stats   = false;
};


class RobotInterface {
public:
    /* 构造函数中加载 RobotInterfaceConfig 到 config_ 进行参数配置 */
    explicit RobotInterface(RobotInterfaceConfig config = {});
    ~RobotInterface();

    /* 仅当电机与 IMU 都已初始化完成时返回 true */
    bool is_initialized() const {
        return motors_initialized_.load() && imu_initialized_.load();
    }


    bool initial_and_start_motors();
    void deinit_motors();
    bool stop_motors(int slave_index = -1);
    bool restart_motors(int slave_index = -1);

    /* 关节目标输入单位为 rad */
    bool apply_action(const std::vector<double>& target_q_rad);
    /* 将关节缓慢复位到 stand_pose_rad，输入/配置单位为 rad */
    bool reset_joints();
    bool is_motors_initialized() const { return motors_initialized_.load(); }
    std::vector<double> get_joint_q() const;    // rad
    std::vector<double> get_joint_vel() const;  // rad/s
    std::vector<double> get_joint_tau() const;  // raw driver unit


    
    bool initial_and_start_imu();
    void deinit_imu();
    bool is_imu_initialized() const { return imu_initialized_.load(); }
    std::array<double, 4> get_quat() const;     // [w, x, y, z]
    std::array<double, 3> get_ang_vel() const;  // [wx, wy, wz], rad/s


private:
    /* 机器人接口配置 */
    RobotInterfaceConfig config_;

    /* 以太网适配器 */
    std::shared_ptr<myactua::EthercatAdapterIGH> adapter_;
    /* 电机控制器 智能指针 */
    std::unique_ptr<myactua::MYACTUA> controller_;
    bool wait_all_slaves_ready() const;


    std::atomic<bool> motors_initialized_{false};
    // Gate apply_action: startup policy requires explicit restart after initial STOP.
    std::atomic<bool> motion_enabled_{false};


    /* IMU 读取器 智能指针 */
    std::unique_ptr<imu::IMUReader> imu_reader_;
    std::atomic<bool> imu_initialized_{false};
    bool initial_start_imu_reader();
    void stop_imu_reader();

    mutable std::mutex imu_mutex_;
    std::array<double, 4> quat_{1.0, 0.0, 0.0, 0.0};
    std::array<double, 3> ang_vel_{0.0, 0.0, 0.0};
};

}  // namespace inference
