#pragma once
#include "policy/policy_runtime.hpp"
#include "recorder/inference_recorder.hpp"
#include "policy/policy_observation_config.hpp"
#include "robot/robot_imu_session.hpp"
#include "robot/robot_motor_session.hpp"
#include "robot/robot_config.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace inference {

namespace robot_detail {
class ActionProcessor;
class JointMapping;
class ObservationBuilder;
}

class RobotInterface {
public:
    /* 构造函数只保存配置；initialize/shutdown/policy_step/apply_action 需由同一控制线程串行调用。 */
    explicit RobotInterface(RobotInterfaceConfig config = {});
    ~RobotInterface();

    bool initialize();
    bool reset_joints();  /* 复位到模型 DOF 顺序配置的 stand_pose_rad，单位为 rad */
    bool is_initialized() const { return initialized_.load(); }

    void shutdown();

    bool apply_action(const std::vector<double>& target_q_model_rad);  // 模型 DOF 顺序目标角(rad)

    void set_target_velocity(double vx, double vy, double yaw_rate);
    std::array<double, 3> get_target_velocity() const;
    bool policy_step();


private:
    struct PolicyCommandSnapshot {
        std::int64_t timestamp_ns{0};
        std::array<double, policy_observation::kDof> target_pos_rad{};
        std::array<double, policy_observation::kDof> target_effort_permille{};
        bool command_applied{false};
    };

    /* 机器人接口配置 */
    RobotInterfaceConfig config_;

    /* 值成员 */
    RobotMotorSession    motor_session_;
    RobotImuSession      imu_session_;

    /* 运行期对象 */
    std::shared_ptr<const robot_detail::JointMapping> joint_mapping_;
    std::unique_ptr<robot_detail::ActionProcessor>    action_processor_;
    std::unique_ptr<robot_detail::ObservationBuilder> observation_builder_;

    PolicyRuntime policy_runtime_;
    mutable std::mutex target_velocity_mutex_;
    std::array<double, 3> target_velocity_{0.0, 0.0, 0.0};  // [vx, vy, yaw_rate]
    InferenceRecorder inference_recorder_;
    bool inference_recorder_failed_ = false;

    std::atomic<bool> initialized_{false};
    std::atomic<bool> policy_command_loop_running_{false};
    std::atomic<bool> policy_command_loop_failed_{false};
    std::thread policy_command_thread_;
    mutable std::mutex policy_command_mutex_;
    std::vector<double> latest_policy_target_q_model_rad_;
    PolicyCommandSnapshot latest_policy_command_;
    std::string policy_command_loop_error_;

    bool validate_policy_config() const;

    bool load_policy();
    void unload_policy();

    bool initialize_model_processors();

    void initialize_policy_runtime_state();

    void record_inference(const InferenceRecord& record);

    bool handle_policy_step_failure(const std::string& message);

    bool start_policy_command_loop();
    void stop_policy_command_loop();
    void policy_command_loop();
    void set_latest_policy_target(const std::vector<double>& target_q_model_rad);
    PolicyCommandSnapshot latest_policy_command_snapshot() const;
    void fail_policy_command_loop(std::string message);
    bool policy_command_loop_healthy(std::string& error) const;
};

}  // namespace inference
