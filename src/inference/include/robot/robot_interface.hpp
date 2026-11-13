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

class ActionProcessor;
class ObservationBuilder;

class RobotInterface {
public:
    /* 构造函数只保存配置；配置由 load_deploy_config() 加载并完整校验 */
    explicit RobotInterface(RobotInterfaceConfig config);
    ~RobotInterface() noexcept;
	
    /* initialize 完成模型准备、锁页、日志/硬件启动及姿态复位，只调用一次 */
    bool initialize();
    bool is_initialized() const {
        return initialized_ && policy_command_worker_ && policy_command_worker_->is_running();
    }

    /* 启停、速度缓存及 policy_step 由同一所有者线程串行访问 */
    bool policy_step();
	
    /* 设置速度 */
    void set_target_velocity(double vx, double vy, double yaw_rate);
    
    /* 显式关闭失败可返回重试；析构仅尝试一次，失败则终止进程。 */
    ShutdownResult shutdown();


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
    // 阶段诊断仅由控制线程维护和读取。
    PolicyStepPhase policy_step_phase_{PolicyStepPhase::Idle};
    std::int64_t    policy_step_phase_started_ns_{0};
    void set_policy_step_phase(PolicyStepPhase phase) noexcept;

    struct PolicyResultAdmission {
        std::int64_t obs_to_action_age_us{0};
        std::int64_t target_hold_age_us{0};
        bool dropped{false};
    };

    // B1 初始测试门限：只用于发布准入，不随策略周期动态调整。
    static constexpr std::int64_t kMaxObsToActionAgeNs = 40'000'000;

    /* 机器人接口配置；子模块只读引用，必须先于子模块构造、晚于子模块析构。 */
    const RobotInterfaceConfig config_;

    /* 值成员 */
    RobotMotorSession    motor_session_;
    RobotImuSession      imu_session_;

    /* 运行期对象 */
    std::unique_ptr<const JointMapping> joint_mapping_;
    std::unique_ptr<ActionProcessor>    action_processor_;
    std::unique_ptr<ObservationBuilder> observation_builder_;

    PolicyRuntime policy_runtime_;

    std::array<double, 3> target_velocity_{0.0, 0.0, 0.0};  // [vx, vy, yaw_rate]

    InferenceRecorder inference_recorder_;
    bool inference_recorder_failed_ = false;

    bool initialized_{false};

    std::int64_t last_policy_target_published_ns_{0};  // 仅 policy 线程访问
    std::atomic<std::int64_t> first_policy_inference_started_ns_{0};

    // 复位成功后构造；worker 引用以上多个成员，必须先于它们析构。
    std::unique_ptr<PolicyCommandWorker> policy_command_worker_;

    bool initialize_processors();

    // 从电机反馈获取复位起始姿态，脚踝 FK 使用配置中的默认姿态作为初始猜测。
    bool get_start_model_pose(const std::array<double, motor_base::kMotorNum>& current_motor_q,
                              std::array<double, kDof>& start_model_q,
                              std::string& error) const;

    // 平滑复位到 default_joint_pos_rad，输出最后成功提交的电机目标。
    bool reset_joints(std::array<double, motor_base::kMotorNum>& final_target_motor_rad);

    bool validate_policy_sensor_timing(std::int64_t motor_timestamp_ns,
                                       std::int64_t imu_timestamp_ns,
                                       std::int64_t now_ns,
                                       std::string& error) const;

    PolicyResultAdmission admit_policy_result(std::int64_t observation_time_ns,
                                              std::int64_t decision_now_ns) const noexcept;

    void record_policy_frame(const InferenceRecord& record);
    void record_completed_policy_frame(PolicyCommandWorker& worker);
    
    bool handle_policy_step_failure(const std::string& message);

};

}  // namespace inference
