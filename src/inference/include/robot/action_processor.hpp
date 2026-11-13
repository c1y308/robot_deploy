#pragma once

#include "kinematics/ankle_motor_fk.hpp"
#include "kinematics/ankle_motor_ik.hpp"
#include "motor_base/command_types.hpp"
#include "policy/policy_observation_config.hpp"
#include "robot/joint_mapping.hpp"
#include "robot/robot_config.hpp"

#include <array>
#include <cstddef>
#include <string>

namespace inference {

struct MotorStateSnapshot;

struct PolicyMotorCommand {
    std::array<motor_base::ImpedanceSetpoint, motor_base::kMotorNum> setpoints{};
    std::size_t setpoint_count{0};
};

class ActionProcessor {
public:
    using ModelJointPositions = std::array<double, kDof>;

    // config 和已构造的 mapping 在运行期不可变，且必须比本对象存活更久。
    ActionProcessor(const RobotInterfaceConfig& config,
                    const JointMapping&         mapping);

    // 纯计算：原始动作裁剪、缩放、偏移限位，再叠加默认姿态。
    ModelJointPositions make_model_target(const std::array<float, kDof>& raw_action) const noexcept;

    // 物理电机顺序的位置保持目标：使用配置增益，速度和前馈力矩为零。
    static std::array<motor_base::ImpedanceSetpoint, motor_base::kMotorNum>
    make_position_impedance_setpoints(
        const std::array<double, motor_base::kMotorNum>& target_motor_rad,
        const MotorConfig& motor_config) noexcept;

    bool build_motor_position_targets(const ModelJointPositions& target_model_rad,
                             std::array<double, motor_base::kMotorNum>& target_motor_rad,
                             std::string& error);

    bool build_policy_motor_command(
        const ModelJointPositions&  target_model_rad,
        const MotorStateSnapshot&   motor_feedback,
        PolicyMotorCommand&         command,
        std::string& error);

private:
    struct LowPass2Coefficients {
        double b0 = 0.0;
        double b1 = 0.0;
        double b2 = 0.0;
        double a1 = 0.0;
        double a2 = 0.0;
    };

    struct LowPass2State {
        double x1 = 0.0;
        double x2 = 0.0;
        double y1 = 0.0;
        double y2 = 0.0;

    };

    struct AnkleTorqueState {
        ankle_motor_fk::Solver fk_solver;
        LowPass2State pitch_filter;
        LowPass2State roll_filter;

        AnkleTorqueState(double roll, double pitch) : fk_solver(roll, pitch) {}
    };

    LowPass2Coefficients compute_lowpass2_coefficients() const;

    template <typename PositionAt>
    bool validate_ankle_motor_limits(
        PositionAt position_at,
        const char* source,
        std::string& error) const;

    bool ankle_ik(const ModelJointPositions& target_model_rad,
                        std::array<double, motor_base::kMotorNum>& target_motor_rad,
                        ankle_motor_ik::Solver& solver,
                        std::string& error);

    bool build_ankle_torque_command(
        ankle_motor_ik::AnkleSide side,
        const ModelJointPositions& target_model_rad,
        const MotorStateSnapshot& motor_feedback,
        PolicyMotorCommand& command,
        std::string& error);

    const RobotInterfaceConfig& config_;
    const JointMapping&         mapping_;

    /* 计算得到的 2 阶低通滤波系数 */
    LowPass2Coefficients low_pass_coeffs_;

    ankle_motor_ik::Solver left_ankle_ik_{ankle_motor_ik::AnkleSide::Left};
    ankle_motor_ik::Solver right_ankle_ik_{ankle_motor_ik::AnkleSide::Right};

    AnkleTorqueState left_ankle_torque_state_;
    AnkleTorqueState right_ankle_torque_state_;
};

}  // namespace inference
