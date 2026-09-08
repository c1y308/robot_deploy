#pragma once
#include "spsc_latest_channel/spsc_latest_channel.hpp"
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
    /* 构造函数只保存配置；配置由 load_deploy_config() 加载并完整校验。
       initialize/shutdown/policy_step/apply_action 需由同一控制线程串行调用。 */
    explicit RobotInterface(RobotInterfaceConfig config);
    ~RobotInterface();

    bool initialize();
    bool is_initialized() const { return initialized_.load(); }


    bool reset_joints();  /* 复位到模型 DOF 顺序配置的 action.default_joint_pos_rad，单位为 rad */
    bool policy_step();
    bool apply_action(const std::vector<double>& target_q_model_rad);  // 模型 DOF 顺序目标角(rad)
    void shutdown();

    void set_target_velocity(double vx, double vy, double yaw_rate);
    std::array<double, 3> get_target_velocity() const;


private:
    struct PolicyTargetFrame {
        std::array<double, policy_observation::kDof> target_q_model_rad{};
        std::uint64_t policy_seq{0};
        std::int64_t  observation_time_ns{0};
        std::int64_t  valid_until_ns{0};
        InferenceRecord inference_record{};
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

    mutable std::mutex    target_velocity_mutex_;
    std::array<double, 3> target_velocity_{0.0, 0.0, 0.0};  // [vx, vy, yaw_rate]

    InferenceRecorder inference_recorder_;
    bool inference_recorder_failed_ = false;

    std::atomic<bool> initialized_{false};
    std::atomic<bool> policy_command_worker_running_{false};
    std::atomic<bool> policy_command_worker_failed_{false};

    std::uint64_t next_policy_seq_{1};  // 策略帧序号，1 起始，0 保留为无效值

    // reset_joints() 最后一次成功下发的电机目标，供首个策略帧前保持姿态。
    std::vector<double> startup_hold_target_motor_rad_;

    std::thread policy_command_worker_thread_;
    // policy_step() 传递给 policy_command_worker 的最新目标及其不可续租截止期
    robot_base::SpscLatestChannel<PolicyTargetFrame>
    policy_target_channel_;

    // worker 补齐首条成功提交的命令后，将同一策略帧的完整日志传回控制线程。
    robot_base::SpscLatestChannel<InferenceRecord>
    completed_policy_record_channel_;

    mutable std::mutex policy_command_error_mutex_;
    std::string        policy_command_worker_error_;

    bool load_policy();
    void unload_policy();

    bool initialize_model_processors();

    void initialize_policy_runtime_state();

    void record_inference(const InferenceRecord& record);

    bool handle_policy_step_failure(const std::string& message);

    bool start_policy_command_worker();
    void stop_policy_command_worker();
    void policy_command_worker_loop();

    void record_latest_completed_policy_frame();
    void fail_policy_command_worker(std::string message);
    bool policy_command_worker_healthy(std::string& error) const;
};

}  // namespace inference
