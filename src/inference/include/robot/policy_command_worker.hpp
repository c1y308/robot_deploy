#pragma once

#include "motor_base/command_types.hpp"
#include "motor_base/status_channel.hpp"
#include "policy/policy_observation_config.hpp"
#include "recorder/inference_recorder.hpp"
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

class ActionProcessor;
struct PolicyMotorCommand;

struct PolicyTargetFrame {
    std::uint64_t policy_seq{0};
    std::int64_t  published_at_ns{0};
    std::int64_t  policy_valid_until_ns{0};
    std::int64_t  obs_to_action_age_us{0};
    std::array<double, kDof> target_model_rad{};
};

/* policy_command_worker：消费目标通道中的模型关节角目标，
   结合电机反馈计算脚踝力矩与其它电机的阻抗 setpoint，通过 motor_session_ 下发。
   worker 持有线程、通道、首帧保持目标及运行/故障状态；首帧截止期通过共享推理时间计算。 */
class PolicyCommandWorker {
public:
    // 固定依赖由外部持有，必须比 worker 存活更久。
    PolicyCommandWorker(
        const RobotInterfaceConfig&          config,
        RobotMotorSession&                   motor_session,
        ActionProcessor&                     action_processor,
        const std::atomic<std::int64_t>&     first_policy_inference_started_ns);
    ~PolicyCommandWorker();

    PolicyCommandWorker(const PolicyCommandWorker&) = delete;
    PolicyCommandWorker& operator=(const PolicyCommandWorker&) = delete;

    // 由控制线程串行调用；启动前共享状态已重置，电机已运行。
    bool start(std::array<double, motor_base::kMotorNum> startup_hold_target_motor_rad);
    void request_stop() noexcept;
    void stop();
    
    // 表示循环是否允许继续执行，不用于判断线程是否已 join。
    bool is_running() const noexcept { return running_.load(); }
    bool check_health(std::string& error) const;

    // 仅策略生产者调用：拷入数据后采样时间发布，返回实际发布时间。
    // 启用 recorder 时 record 必须非空；发布时间和截止期由 worker 设置。
    std::int64_t publish_target(
        const PolicyTargetFrame& target,
        const InferenceRecord*   record = nullptr) noexcept;
    bool try_consume_completed_record(InferenceRecord& record) noexcept;

private:
    friend struct ::inference::RobotInterfacePolicyTimingTestAccess;

    struct PolicyTargetWithRecordFrame {
        PolicyTargetFrame target{};
        InferenceRecord   record{};
    };

    void run();
    bool refresh_startup_hold();
    bool try_consume_target(PolicyTargetWithRecordFrame& frame) noexcept;
    void complete_and_publish_record(
        const InferenceRecord& record,
        const PolicyMotorCommand& command,
        const motor_base::CommandTiming& timing);

    motor_base::CommandTiming build_command_timing(
        const PolicyTargetFrame& target,
        std::int64_t produced_at_ns,
        std::int64_t feedback_valid_until_ns) const noexcept;

    std::atomic<bool> running_{false};
    std::atomic<bool> failed_{false};
    mutable std::mutex error_mutex_;
    std::string error_;

    void fail(std::string message);
    std::string format_target_timeout_error(
        const PolicyTargetFrame& target,
        std::int64_t now_ns) const;

    const RobotInterfaceConfig& config_;
    RobotMotorSession& motor_session_;
    ActionProcessor&   action_processor_;
    const std::int64_t target_timeout_ns_;          // 策略目标发布后，最多可以使用多久
    const std::int64_t command_timeout_ns_;         // 每次生成的电机命令，最多有效多久

    // 目标与日志共用一个通道；启用 recorder 时填写和处理日志记录。
    robot_base::SpscLatestChannel<PolicyTargetWithRecordFrame> target_channel_;

    // 日志回传：policy_cmd 线程生产，策略主线程消费。
    robot_base::SpscLatestChannel<InferenceRecord>   record_channel_;

    const std::atomic<std::int64_t>&    first_policy_inference_started_ns_;
    // reset_joints 最后成功提交的目标，供首个策略帧前保持姿态。
    std::array<double, motor_base::kMotorNum> startup_hold_target_motor_rad_{};

    // 线程句柄；线程函数 run() 由 start() 启动，stop() 请求停止并 join。
    std::thread worker_thread_;
};

}  // namespace inference
