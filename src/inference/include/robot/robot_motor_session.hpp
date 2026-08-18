#pragma once

#include "robot/robot_config.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

namespace myactua {
class EthercatAdapterIGH;
}

namespace motor_base {
struct ControlCommand;
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
    explicit RobotMotorSession(MotorConfig config = {});
    ~RobotMotorSession();

    RobotMotorSession(const RobotMotorSession&) = delete;
    RobotMotorSession& operator=(const RobotMotorSession&) = delete;

    bool initialize_and_start();
    bool stop(int motor_index = -1);
    bool restart(int motor_index = -1);
    void deinitialize();

    bool is_initialized() const noexcept { return initialized_.load(); }
    bool motion_enabled() const noexcept { return motion_enabled_.load(); }

    // 依据当前电机模式下发位置指令
    bool apply_targets_rad(const std::vector<double>& target_motor_rad);
    // 通过MIT模式下发阻抗指令
    bool apply_impedance_setpoints(const std::vector<motor_base::ImpedanceSetpoint>& setpoints);

    MotorStateSnapshot  get_motor_snapshot() const;
    std::vector<double> get_joint_q() const;
    std::vector<double> get_joint_vel() const;
    std::vector<double> get_joint_torque_percent() const;

private:
    bool validate_config() const;
    bool submit_command(const motor_base::ControlCommand& command,
                        const char* context);

    MotorConfig config_;

    std::shared_ptr<myactua::EthercatAdapterIGH>     adapter_;
    std::unique_ptr<motor_base::MotorControllerBase> controller_;

    std::atomic<bool> initialized_{false};
    std::atomic<bool> motion_enabled_{false};
};

}  // namespace inference
