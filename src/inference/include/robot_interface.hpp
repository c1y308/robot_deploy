#pragma once
#include "ControlTypes.hpp"
#include "EthercatAdapterIGH.hpp"
#include "imu_reader.hpp"
#include "motor_control.hpp"

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace myactua {
class EthercatAdapterIGH;
class MYACTUA;
}

namespace imu {
class IMUReader;
}

namespace inference {

struct RobotInterfaceConfig {

    /* 电机 配置*/
    int num_motors = 2;
    std::string ethercat_ifname = "enp8s0";
    /* 等待所有从站就绪的超时和轮询时间 */
    int wait_all_slaves_timeout_ms = 20000;
    int wait_all_slaves_poll_ms = 100;
    bool print_motors_info = true;

    std::vector<int> print_motor_ids = {0, 1}; 
    
    /* IMU 配置 */
    std::string imu_device = "/dev/ttyUSB0";
    int imu_baudrate       = 921600;
    bool imu_print_imu     = false;
    bool imu_print_ahrs    = true;
    bool imu_print_stats   = false;


    // Optional limits in radians, size == num_motors.
    std::vector<double> joint_min_rad;
    std::vector<double> joint_max_rad;

    // Default safe/stand posture, in radians. Empty means all zeros.
    std::vector<double> stand_pose_rad;
};


class RobotInterface {
public:
    /* 构造函数中加载 RobotInterfaceConfig */
    explicit RobotInterface(RobotInterfaceConfig config = {});
    ~RobotInterface();


    bool initial_and_start_motors();
    void deinit_motors();
    bool initial_and_start_imu();
    void deinit_imu();


    bool apply_action(const std::vector<double>& target_q_rad);
    bool reset_joints();


    bool is_initialized() const {
        return motors_initialized_.load() && imu_initialized_.load();
    }
    bool is_motors_initialized() const { return motors_initialized_.load(); }
    bool is_imu_initialized() const { return imu_initialized_.load(); }


    std::vector<double> get_joint_q() const;    // rad
    std::vector<double> get_joint_vel() const;  // rad/s
    std::vector<double> get_joint_tau() const;  // raw driver unit

    std::array<double, 4> get_quat() const;     // [w, x, y, z]
    std::array<double, 3> get_ang_vel() const;  // [wx, wy, wz], rad/s


private:
    bool wait_all_slaves_ready() const;
    
    bool initial_start_imu_reader();
    void stop_imu_reader();

    /* 机器人接口配置 */
    RobotInterfaceConfig config_;

    /* 以太网适配器 */
    std::shared_ptr<myactua::EthercatAdapterIGH> adapter_;
    /* 电机控制器 智能指针 */
    std::unique_ptr<myactua::MYACTUA> controller_;
    /* IMU 读取器 智能指针 */
    std::unique_ptr<imu::IMUReader> imu_reader_;
    std::thread imu_thread_;

    mutable std::mutex imu_mutex_;
    std::array<double, 4> quat_{1.0, 0.0, 0.0, 0.0};
    std::array<double, 3> ang_vel_{0.0, 0.0, 0.0};

    std::atomic<bool> motors_initialized_{false};
    std::atomic<bool> imu_initialized_{false};
};

}  // namespace inference
