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
    int num_motors = 12;
    std::string ethercat_ifname = "enp8s0";
    /* 等待所有从站就绪的超时和轮询时间 */
    int wait_all_slaves_timeout_ms = 10000;
    int wait_all_slaves_poll_ms = 100;

    /* IMU 配置 */
    bool enable_imu = true;
    std::string imu_device = "/dev/ttyUSB0";
    int imu_baudrate = 921600;


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

    bool init_motors();
    void deinit_motors();

    std::vector<double> get_joint_q() const;    // rad
    std::vector<double> get_joint_vel() const;  // rad/s
    std::vector<double> get_joint_tau() const;  // raw driver unit

    std::array<double, 4> get_quat() const;     // [w, x, y, z]
    std::array<double, 3> get_ang_vel() const;  // [wx, wy, wz], rad/s

    bool apply_action(const std::vector<double>& target_q_rad);
    bool reset_joints();

    bool is_initialized() const { return initialized_.load(); }

private:
    static double raw_pos_to_rad(double raw_pos);
    static double raw_vel_to_rad_s(double raw_vel);
    static double rad_to_csp_deg(double rad);

    bool wait_all_slaves_ready() const;
    bool start_imu_reader();
    void stop_imu_reader();

    /* 机器人接口配置 */
    RobotInterfaceConfig config_;

    /* 以太网适配器 */
    std::shared_ptr<myactua::EthercatAdapterIGH> adapter_;
    /* 电机控制器 */
    std::unique_ptr<myactua::MYACTUA> controller_;
    /* IMU 读取器 */
    std::unique_ptr<imu::IMUReader> imu_reader_;
    std::thread imu_thread_;

    mutable std::mutex imu_mutex_;
    std::array<double, 4> quat_{1.0, 0.0, 0.0, 0.0};
    std::array<double, 3> ang_vel_{0.0, 0.0, 0.0};

    std::atomic<bool> initialized_{false};
};

}  // namespace inference
