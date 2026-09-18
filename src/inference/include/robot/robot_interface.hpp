#pragma once
#include "policy/policy_runtime.hpp"
#include "recorder/inference_recorder.hpp"
#include "policy/policy_observation_config.hpp"
#include "robot/policy_command_worker.hpp"
#include "robot/robot_imu_session.hpp"
#include "robot/robot_motor_session.hpp"
#include "robot/robot_config.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

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
       initialize 只调用一次；启停、速度缓存及 policy_step 由同一所有者线程串行访问。
       初始化失败后仍可重复 shutdown，析构也会执行清理。 */
    explicit RobotInterface(RobotInterfaceConfig config);
    ~RobotInterface();

    bool initialize();
    bool is_initialized() const {
        return initialized_ && worker_.is_running();
    }


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

    std::array<double, 3> target_velocity_{0.0, 0.0, 0.0};  // [vx, vy, yaw_rate]

    InferenceRecorder inference_recorder_;
    bool inference_recorder_failed_ = false;

    bool initialized_{false};
    // 阶段诊断仅由控制线程维护和读取。
    PolicyStepPhase policy_step_phase_{PolicyStepPhase::Idle};
    std::int64_t policy_step_phase_started_ns_{0};

    std::int64_t last_policy_target_published_ns_{0};  // 仅 policy 线程访问
    std::atomic<std::int64_t> first_policy_inference_started_ns_{0};

    // worker_ 引用以上多个成员，声明顺序必须在其之后。
    robot_detail::PolicyCommandWorker worker_;

    bool load_policy();
    void unload_policy();

    bool initialize_model_processors();

    // 平滑复位到 default_joint_pos_rad，输出最后成功提交的电机目标。
    bool reset_joints(std::array<double, motor_base::kMaxMotors>& final_target_motor_rad);

    void initialize_policy_runtime_state();
    void reset_policy_command_state() noexcept;

    bool validate_policy_sensor_timing(std::int64_t motor_timestamp_ns,
                                       std::int64_t imu_timestamp_ns,
                                       std::int64_t now_ns,
                                       std::string& error) const;

    std::uint64_t begin_policy_inference(std::int64_t now_ns) noexcept;

    PolicyResultAdmission admit_policy_result(std::int64_t observation_time_ns,
                                              std::int64_t decision_now_ns) const noexcept;

    void record_policy_frame(const InferenceRecord& record);
    void record_completed_policy_frame();
    
    bool handle_policy_step_failure(const std::string& message);

    void set_policy_step_phase(PolicyStepPhase phase) noexcept;
};

}  // namespace inference
