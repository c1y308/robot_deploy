#pragma once

#include "motor_base/command_types.hpp"
#include "policy/policy_observation_config.hpp"
#include "recorder/inference_record.hpp"
#include "robot/robot_config.hpp"
#include "spsc_latest_channel/spsc_latest_channel.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace inference {

class RobotMotorSession;
struct RobotInterfacePolicyTimingTestAccess;

namespace robot_detail {

class ActionProcessor;

struct PolicyTargetFrame {
    std::int64_t    published_at_ns{0};
    InferenceRecord inference_record{};
};

/* policy_command_worker：消费目标通道中的模型关节角目标，
   结合电机反馈计算脚踝力矩与其它电机的阻抗 setpoint，通过 motor_session_ 下发。
   worker 持有线程、通道、首帧保持目标及运行/故障状态；首帧截止期通过共享推理时间计算。 */
class PolicyCommandWorker {
public:
    PolicyCommandWorker(
        const RobotInterfaceConfig&          config,
        RobotMotorSession&                   motor_session,
        const std::atomic<std::int64_t>&     first_policy_inference_started_ns);
    ~PolicyCommandWorker();

    PolicyCommandWorker(const PolicyCommandWorker&) = delete;
    PolicyCommandWorker& operator=(const PolicyCommandWorker&) = delete;

    // 由控制线程串行调用；启动前共享状态已重置，电机已运行。
    // action_processor 必须存活到 stop() 返回。
    bool start(ActionProcessor&    action_processor,
               std::array<double, motor_base::kMaxMotors> startup_hold_target_motor_rad);
    void request_stop() noexcept;
    void stop();
    
    bool is_running() const noexcept { return running_.load(); }
    bool healthy(std::string& error) const;

    // 控制线程在 worker 停止后重置通道。
    void reset_channels() noexcept;
    // 仅策略生产者调用：直接填写写槽，再采样时间发布。
    // publish_target() 后写槽引用失效，返回实际发布时间。
    PolicyTargetFrame& acquire_target_write_slot() noexcept;
    std::int64_t publish_target() noexcept;
    bool try_consume_completed_record(InferenceRecord& record) noexcept;

private:
    friend struct ::inference::RobotInterfacePolicyTimingTestAccess;

    void loop();

    std::int64_t startup_policy_deadline_ns() const noexcept;
    static bool policy_deadline_expired(std::int64_t deadline_ns,
                                        std::int64_t now_ns) noexcept;
    motor_base::CommandTiming policy_command_timing(
        const PolicyTargetFrame& target,
        std::int64_t produced_at_ns,
        std::int64_t oldest_feedback_timestamp_ns) const noexcept;

    std::atomic<bool> running_{false};
    std::atomic<bool> failed_{false};
    mutable std::mutex error_mutex_;
    std::string error_;

    void fail(std::string message);
    std::string policy_deadline_error(const PolicyTargetFrame& target,
                                      std::int64_t now_ns) const;

    const RobotInterfaceConfig& config_;
    RobotMotorSession& motor_session_;
    ActionProcessor*   action_processor_{nullptr};

    robot_base::SpscLatestChannel<PolicyTargetFrame> target_channel_;
    robot_base::SpscLatestChannel<InferenceRecord>   completed_record_channel_;

    const std::atomic<std::int64_t>&    first_policy_inference_started_ns_;
    // reset_joints 最后成功提交的目标，供首个策略帧前保持姿态。
    std::array<double, motor_base::kMaxMotors> startup_hold_target_motor_rad_{};

    std::thread worker_thread_;
};

}  // namespace robot_detail
}  // namespace inference
