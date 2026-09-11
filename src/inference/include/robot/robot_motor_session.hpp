#pragma once

#include "motor_base/status_channel.hpp"
#include "robot/robot_config.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace myactua {
class EthercatAdapterIGH;
}

namespace motor_base {
struct ControlCommand;
struct RtEvent;
class MotorControllerBase;
}

namespace inference {

// inference层需要的电机数据，从底层的 MotorStatusSnapshot 获取
struct MotorStateSnapshot {
    std::int64_t timestamp_ns{0};

    std::vector<double> position_rad;
    std::vector<double> velocity_rad_s;
    std::vector<double> torque_percent;

    std::vector<std::uint8_t> comm_ok;
    std::vector<std::uint8_t> enabled;
    std::vector<std::uint8_t> faulted;
};

class RobotMotorSession {
public:
    RobotMotorSession(MotorConfig config,
                      SafetyConfig safety,
                      RuntimeThreadingConfig runtime = {});
    ~RobotMotorSession();

    RobotMotorSession(const RobotMotorSession&) = delete;
    RobotMotorSession& operator=(const RobotMotorSession&) = delete;

    bool initialize();
    motor_base::CommandSubmitResult request_stop(int motor_index = -1);
    bool stop(int motor_index = -1);
    bool restart(int motor_index = -1);
    // False retains the running controller for another confirmation attempt.
    bool deinitialize();

    bool is_initialized() const noexcept { return initialized_.load(); }
    bool motion_enabled() const noexcept { return motion_enabled_.load(); }

    // 依据当前电机模式下发位置指令
    bool apply_targets_rad(const std::vector<double>& target_motor_rad);
    bool apply_impedance_setpoints_realtime(
        const std::array<motor_base::ImpedanceSetpoint,
                         motor_base::kMaxMotorCommandSetpoints>& setpoints,
        std::size_t count,
        const motor_base::CommandTiming& timing);

    bool try_consume_latest_status_command(
        std::array<motor_base::MotorStatusSnapshot,
                   motor_base::kMaxMotorCommandSetpoints>& feedback);

    MotorStateSnapshot  get_motor_snapshot() const;
    std::vector<double> get_joint_q() const;

private:
    friend class RobotInterface;
    bool wait_for_stop(const motor_base::CommandSubmitResult& request);
    void release_stopped_controller();
    bool submit_command(const motor_base::ControlCommand& command,
                        const char* context);

    MotorConfig config_;
    SafetyConfig safety_;
    RuntimeThreadingConfig runtime_;

    std::shared_ptr<myactua::EthercatAdapterIGH>     adapter_;
    std::unique_ptr<motor_base::MotorControllerBase> controller_;

    std::atomic<bool> initialized_{false};
    std::atomic<bool> motion_enabled_{false};
    bool rt_started_{false}; // Owner thread; also covers incomplete initialization.

    // policy/inference 线程专属 RT feedback 通道的缓存帧：
    // 仅 get_motor_snapshot()（policy 线程）读写，无新帧时保留上一帧有效反馈
    mutable std::array<motor_base::MotorStatusSnapshot,
                       motor_base::kMaxMotorCommandSetpoints> latest_feedback_{};
    mutable bool has_policy_feedback_{false};
};

}  // namespace inference
