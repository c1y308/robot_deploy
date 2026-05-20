#pragma once
#include "EthercatAdapterIGH.hpp"
#include "imu_reader.hpp"
#include "motor_control.hpp"

#include <array>
#include <atomic>
#include <chrono>
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

class TorchPolicyRunner;

/* 接口层的配置结构体 */
struct RobotInterfaceConfig {

    /* 电机 配置*/
    int num_motors = 12;
    std::string ethercat_ifname = "enp8s0";
    /* 等待所有从站就绪的超时和轮询时间 */
    int wait_all_slaves_timeout_ms = 30000;
    int wait_all_slaves_poll_ms    = 100;
    /* 是否在终端打印电机信息 */
    bool print_motors_info = true;  // false将 print_motor_ids 清零
    std::vector<int> print_motor_ids = {-1};  // 仅打印这些ID的电机信息，ID从0开始，-1表示全部
    /* 相对 stand_pose_rad 的关节弧度偏移限制，按模型 DOF 序号填写 */
    std::vector<double> joint_min_rad;
    std::vector<double> joint_max_rad;
    /* 初始姿态关节弧度值，按模型 DOF 序号填写 */
    std::vector<double> stand_pose_rad = {};
    /* 默认使用 MIT/PVT 模式；若改为 CSP，apply_action 将沿用位置指令路径 */
    myactua::ControlMode motor_control_mode = myactua::ControlMode::PVT;
    /* MIT/PVT 模式刚度/阻尼，必须由调用侧显式配置 */
    std::vector<double> mit_kp;
    std::vector<double> mit_kd;



    /* IMU 配置 */
    std::string imu_device = "/dev/ttyUSB0";
    int imu_baudrate       = 921600;

    bool imu_print_imu     = false;
    bool imu_print_ahrs    = false;
    bool imu_print_stats   = false;


    /* TorchScript 策略配置 */
    /* .pt 模型文件路径，由 torch.jit.script/trace 导出 */
    std::string policy_model_path;
    /* 模型关节维度到电机逻辑索引的映射，长度必须为 12 且不可重复 */
    std::vector<int> model_to_motor_index;
    /* 电机物理/控制器 ID 到模型关节方向的符号；1 表示方向一致，-1 表示方向相反；为空时全部按 1 */
    std::vector<int> motor_to_model_direction;
    /* 网络原始输出动作的截断范围：[-action_clip, action_clip] */
    double action_clip = 1.0;
    /* 截断后的动作缩放系数，长度必须为 12；按模型 DOF 序号使用 */
    std::vector<double> action_scale;
    /* 步态相位周期，单位 s；phase = fmod(elapsed / policy_cycle_time_s, 1) */
    double policy_cycle_time_s = 0.02;
    /* DOF Pos 缩放，长度必须为 12；按模型 DOF 序号使用 */
    std::vector<double> dof_pos_scale;
    /* DOF Vel 缩放，长度必须为 12；按模型 DOF 序号使用 */
    std::vector<double> dof_vel_scale;
    /* 速度指令缩放，对应 vx、vy、yaw_rate */
    std::array<double, 3> command_scale = {
        0.1,
        0.1,
        0.1
    };
    /* 机身角速度缩放，对应 AHRS roll_speed、pitch_speed、heading_speed */
    std::array<double, 3> body_ang_vel_scale = {
        0.2,
        0.2,
        0.2
    };
    /* 机身欧拉角缩放，对应 AHRS roll、pitch、heading */
    std::array<double, 3> euler_scale = {
        1,
        1,
        1
    };
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


    /* 电机部分 */
    bool initial_and_start_motors();
    bool stop_motors(int slave_index = -1);
    bool restart_motors(int slave_index = -1);
    void deinit_motors();
    bool apply_action(const std::vector<double>& target_q_model_rad);  // 模型 DOF 顺序目标角(rad)
    bool reset_joints();  // /* 复位到模型 DOF 顺序配置的 stand_pose_rad，单位为 rad */

    bool is_motors_initialized() const { return motors_initialized_.load(); }
    std::vector<double> get_joint_q() const;    // rad
    std::vector<double> get_joint_vel() const;  // rad/s
    std::vector<double> get_joint_tau() const;  // raw driver unit


    /* IMU部分 */
    bool initial_and_start_imu();
    void deinit_imu();
    bool is_imu_initialized() const { return imu_initialized_.load(); }
    std::array<double, 4> get_quat() const;     // [w, x, y, z]
    std::array<double, 3> get_ang_vel() const;  // [wx, wy, wz], rad/s
    std::array<double, 3> get_body_ang_vel() const;  // [roll_speed, pitch_speed, heading_speed], rad/s
    std::array<double, 3> get_euler() const;          // [roll, pitch, heading], rad



    /* 模型部分 */
    bool load_policy();
    void unload_policy();
    void reset_policy_state();
    bool policy_step(double vx, double vy, double yaw_rate);


private:
    static constexpr int kPolicyDof = 12;  // 模型输出的动作维度，必须与电机数量一致
#if POLICY_V3
    static constexpr int kPolicySingleObservationSize = 47;  // 单帧模型观测维度
    static constexpr int kPolicyFrameStack = 15;             // 模型输入使用 15 帧观测
#else
    static constexpr int kPolicySingleObservationSize = 45;  // 单周期各 observation term 总维度
    static constexpr int kPolicyFrameStack = 5;              // policy.pt 每个 term 使用 5 帧历史
#endif
    static constexpr int kPolicyObservationSize =
        kPolicySingleObservationSize * kPolicyFrameStack;

    /* 机器人接口配置 */
    RobotInterfaceConfig config_;

    /* 以太网适配器 智能指针 */
    std::shared_ptr<myactua::EthercatAdapterIGH> adapter_;
    std::unique_ptr<myactua::MYACTUA> controller_;  // 电机控制器指针
    bool validate_motor_config() const;


    std::atomic<bool> motors_initialized_{false};
    // Gate apply_action: startup policy requires explicit restart after initial STOP.
    std::atomic<bool> motion_enabled_{false};


    /* IMU 读取器 智能指针 */
    std::unique_ptr<imu::IMUReader> imu_reader_;
    std::atomic<bool> imu_initialized_{false};
    std::atomic<bool> ahrs_ready_{false};

    mutable std::mutex imu_mutex_;
    std::array<double, 4> quat_{1.0, 0.0, 0.0, 0.0};
    std::array<double, 3> ang_vel_{0.0, 0.0, 0.0};
    std::array<double, 3> body_ang_vel_{0.0, 0.0, 0.0};
    std::array<double, 3> euler_{0.0, 0.0, 0.0};


    /*  Policy 运行器 智能指针 */
    std::unique_ptr<TorchPolicyRunner> policy_runner_;
    mutable std::mutex policy_mutex_;
    std::array<float, kPolicyDof> last_action_raw_{};  // 记录上一周期动作值
    std::array<float, kPolicyObservationSize> observation_history_{};
    bool observation_history_ready_ = false;
    std::chrono::steady_clock::time_point policy_start_time_{};

    bool validate_policy_config() const;
    bool build_policy_observation(double vx,
                                  double vy,
                                  double yaw_rate,
                                  std::array<float, kPolicyObservationSize>& observation);
    bool handle_policy_step_failure(const std::string& message);
};

}  // namespace inference
