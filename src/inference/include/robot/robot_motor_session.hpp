#pragma once

#include "motor_base/status_channel.hpp"
#include "robot/robot_config.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

namespace myactua {
class EthercatAdapterIGH;
class MyActMotorController;
}

namespace motor_base {
struct ControlCommand;
struct RtEvent;
class MotorControllerBase;
}

namespace inference {

enum class ShutdownResult {
    Confirmed,
    ReleasedAfterCommLoss,
    RetryRequired,
};

enum class MotorCommunicationState {
    Healthy,
    LinkDown,
    WkcIncomplete,
};

// inference层需要的电机数据，从底层的 MotorStatusSnapshot 获取
struct MotorStateSnapshot {
    std::int64_t timestamp_ns{0};

    std::array<double, motor_base::kMotorNum> position_rad{};
    std::array<double, motor_base::kMotorNum> velocity_rad_s{};
    std::array<double, motor_base::kMotorNum> torque_percent{};

    std::array<std::uint8_t, motor_base::kMotorNum> comm_ok{};
    std::array<std::uint8_t, motor_base::kMotorNum> enabled{};
};

class RobotMotorSession {
public:
    RobotMotorSession(MotorConfig config,
                      SafetyConfig safety,
                      RuntimeThreadingConfig runtime = {},
                      SensorGuardConfig sensor_guard = {});
    ~RobotMotorSession();

    RobotMotorSession(const RobotMotorSession&) = delete;
    RobotMotorSession& operator=(const RobotMotorSession&) = delete;

    bool initialize(bool defer_communication_protection = false);
    motor_base::CommandSubmitResult request_stop(int motor_index = -1);
    bool stop(int motor_index = -1);
    bool restart(int motor_index = -1);
    // False retains the running controller for another confirmation attempt.
    bool deinitialize();
    // 调用前结束命令生产线程；确认先前的 STOP，按通信状态决定是否释放控制器。
    ShutdownResult shutdown(const motor_base::CommandSubmitResult& request);
    void enable_communication_protection() noexcept;

    bool is_initialized() const noexcept { return initialized_.load(); }
    bool motion_enabled() const noexcept { return motion_enabled_.load(); }
    bool communication_fault_latched() const noexcept;
    MotorCommunicationState communication_state() const noexcept;

    // 依据当前电机模式下发位置指令
    // absolute_deadline_ns=0 保持原有独立命令有效期；正值额外封顶截止期。
    bool apply_targets_rad(const std::array<double, motor_base::kMotorNum>& target_motor_rad,
                           std::int64_t absolute_deadline_ns = 0);
    bool apply_impedance_setpoints_realtime(
        const std::array<motor_base::ImpedanceSetpoint,
                         motor_base::kMotorNum>& setpoints,
        std::size_t count,
        const motor_base::CommandTiming& timing);

    // 以下反馈接口仅由 policy_cmd 线程调用；无新帧时保留缓存。
    bool refresh_command_feedback(std::string& error);
    // 只读引用在下一次刷新或会话释放前使用完毕。
    const std::array<motor_base::MotorStatusSnapshot,
                     motor_base::kMotorNum>& command_feedback() const noexcept;
    // 校验当前缓存，成功时输出最老反馈的截止期；不刷新反馈。
    bool validate_motor_feedback_timing(
        std::int64_t now_ns,
        std::int64_t& feedback_valid_until_ns,
        std::string& error) const;

    MotorStateSnapshot  get_motor_snapshot() const;

private:
    friend struct RobotInterfacePolicyTimingTestAccess;
    motor_base::CommandTiming target_command_timing(
        std::int64_t produced_at_ns, std::int64_t absolute_deadline_ns) const noexcept;
    bool wait_for_stop(const motor_base::CommandSubmitResult& request);
    void release_controller();
    bool submit_command(const motor_base::ControlCommand& command,
                        const char* context);

    MotorConfig config_;
    SafetyConfig safety_;
    RuntimeThreadingConfig runtime_;
    const std::int64_t max_feedback_age_ns_;

    std::shared_ptr<myactua::EthercatAdapterIGH>     adapter_;
    std::unique_ptr<myactua::MyActMotorController> controller_;

    std::atomic<bool> initialized_{false};
    std::atomic<bool> motion_enabled_{false};
    bool rt_started_{false}; // Owner thread; also covers incomplete initialization.

    // policy/inference 线程专属 RT feedback 通道的缓存帧：
    // 仅 get_motor_snapshot()（policy 线程）读写，无新帧时保留上一帧有效反馈
    mutable std::array<motor_base::MotorStatusSnapshot,
                       motor_base::kMotorNum> latest_feedback_{};
    mutable bool has_policy_feedback_{false};

    // policy_cmd 线程独占；生命周期重置须在命令线程停止时进行。
    std::array<motor_base::MotorStatusSnapshot,
               motor_base::kMotorNum> command_feedback_{};
    bool has_command_feedback_{false};
};

}  // namespace inference
