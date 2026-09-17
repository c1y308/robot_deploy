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

enum class ShutdownResult {
    Confirmed,
    ReleasedAfterCommLoss,
    RetryRequired,
};

namespace robot_detail {
class ActionProcessor;
class ObservationBuilder;
}

class RobotInterface {
public:
    /* 构造函数只保存配置；配置由 load_deploy_config() 加载并完整校验。
       initialize/shutdown/policy_step 需由同一控制线程串行调用。 */
    explicit RobotInterface(RobotInterfaceConfig config);
    ~RobotInterface();

    bool initialize();
    bool is_initialized() const { return initialized_.load(); }


    bool policy_step();
    // RetryRequired retains RT and resources for another confirmation attempt.
    // ReleasedAfterCommLoss releases resources without claiming STOP confirmation.
    ShutdownResult shutdown();

    void set_target_velocity(double vx, double vy, double yaw_rate);
    std::array<double, 3> get_target_velocity() const;


private:
    friend struct RobotInterfacePolicyTimingTestAccess;

    enum class PolicyStepPhase : std::uint8_t {
        Idle,
        Precheck,
        SensorSnapshot,
        Observation,
        Inference,
        PostInference,
        Publish,
    };

    struct PolicyTargetFrame {
        std::array<double, policy_observation::kDof> target_q_model_rad{};
        std::uint64_t policy_seq{0};
        std::uint64_t target_seq{0};
        std::int64_t  published_at_ns{0};
        std::int64_t  valid_until_ns{0};
        InferenceRecord inference_record{};
    };

    struct PolicyResultAdmission {
        std::int64_t obs_to_action_age_us{0};
        std::int64_t target_hold_age_us{0};
        bool dropped{false};
    };

    // B1 初始测试门限：只用于发布准入，不随策略周期动态调整。
    static constexpr std::int64_t kMaxObsToActionAgeNs = 40'000'000;

    /* 机器人接口配置 */
    RobotInterfaceConfig config_;

    /* 值成员 */
    RobotMotorSession    motor_session_;
    RobotImuSession      imu_session_;

    /* 运行期对象 */
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
    std::atomic<PolicyStepPhase> policy_step_phase_{PolicyStepPhase::Idle};
    std::atomic<std::int64_t> policy_step_phase_started_ns_{0};

    std::uint64_t next_policy_seq_{1};  // 每轮正式推理递增，包括 drop
    std::uint64_t next_target_seq_{1};  // 仅有效发布时递增，0 表示未发布
    std::int64_t last_policy_target_published_ns_{0};  // 仅 policy 线程访问
    std::atomic<std::int64_t> first_policy_inference_started_ns_{0};

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

    bool reset_joints();  // 初始化时平滑复位到模型 DOF 顺序的 default_joint_pos_rad。

    void initialize_policy_runtime_state();
    void reset_policy_command_state() noexcept;

    bool validate_policy_sensor_timing(std::int64_t motor_timestamp_ns,
                                       std::int64_t imu_timestamp_ns,
                                       std::int64_t now_ns,
                                       std::string& error) const;
    std::uint64_t begin_policy_inference(std::int64_t now_ns) noexcept;
    PolicyResultAdmission admit_policy_result(std::int64_t observation_time_ns,
                                              std::int64_t decision_now_ns) const noexcept;
    // staged_target 必须属于 policy_target_channel_ 的生产者写槽。
    void publish_policy_target(PolicyTargetFrame& staged_target,
                               std::int64_t published_at_ns) noexcept;
    void complete_policy_result(const InferenceRecord& record);
    std::int64_t startup_policy_deadline_ns() const noexcept;
    bool startup_policy_target_expired(std::int64_t now_ns) const noexcept;
    static bool policy_deadline_expired(std::int64_t deadline_ns,
                                        std::int64_t now_ns) noexcept;
    motor_base::CommandTiming policy_command_timing(
        const PolicyTargetFrame& target, std::int64_t produced_at_ns) const noexcept;

    void record_inference(const InferenceRecord& record);

    bool handle_policy_step_failure(const std::string& message);

    bool start_policy_command_worker();
    void stop_policy_command_worker();
    void policy_command_worker_loop();

    void record_latest_completed_policy_frame();
    void fail_policy_command_worker(std::string message);
    bool policy_command_worker_healthy(std::string& error) const;
    void set_policy_step_phase(PolicyStepPhase phase) noexcept;
    std::string policy_deadline_error(const PolicyTargetFrame& target,
                                      std::int64_t now_ns) const;
};

}  // namespace inference
