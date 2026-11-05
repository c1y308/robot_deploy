#pragma once

#include "kinematics/ankle_motor_fk.hpp"
#include "kinematics/ankle_motor_ik.hpp"
#include "motor_base/status_channel.hpp"
#include "policy/policy_observation_config.hpp"
#include "robot/joint_mapping.hpp"
#include "robot/robot_config.hpp"

#include <array>
#include <memory>
#include <string>

namespace inference {

struct LowPass2Coefficients {
    double b0 = 0.0;
    double b1 = 0.0;
    double b2 = 0.0;
    double a1 = 0.0;
    double a2 = 0.0;
};

class ActionProcessor {
public:
    struct FixedPolicyMotorCommand {
        std::array<motor_base::ImpedanceSetpoint,
                   motor_base::kMaxMotorCommandSetpoints> setpoints{};
        std::size_t setpoint_count{0};
    };

    using ModelJointPositions = std::array<double, kModelDof>;

    // 纯计算：原始动作裁剪、缩放、偏移限位，再叠加默认姿态。
    static ModelJointPositions make_model_target(
        const PolicyAction& raw_action, const ActionConfig& config) noexcept;

    // mapping 由 JointMapping::create() 成功构造，运行期不可变。
    ActionProcessor(std::shared_ptr<const JointMapping> mapping,
                    ModelJointPositions default_joint_pos_rad,
                    AnkleMotorLimitConfig ankle_motor_limits,
                    std::array<double, motor_base::kMaxMotors> motor_kp,
                    std::array<double, motor_base::kMaxMotors> motor_kd,
                    AnkleTorqueControlConfig torque_config);

    bool make_motor_target(const ModelJointPositions& target_model_rad,
                             std::array<double, motor_base::kMaxMotors>& target_motor_rad,
                             std::string& error);

    bool build_reset_start_model_pose(const std::array<double, motor_base::kMaxMotors>& current_motor_q,
                                      const ModelJointPositions& target_model_q,
                                      ModelJointPositions& start_model_q,
                                      std::string& error) const;

    bool build_policy_impedance_command(
        const ModelJointPositions& target_model_rad,
        const std::array<motor_base::MotorStatusSnapshot,
                         motor_base::kMaxMotorCommandSetpoints>& motor_feedback,
        FixedPolicyMotorCommand& command,
        std::string& error);

private:
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

    bool apply_ankle_ik(const ModelJointPositions& target_model_rad,
                        std::array<double, motor_base::kMaxMotors>& target_motor_rad,
                        ankle_motor_ik::Solver& solver,
                        std::string& error);

    bool apply_ankle_torque_control(
        const ModelJointPositions& target_model_rad,
        const std::array<motor_base::MotorStatusSnapshot,
                         motor_base::kMaxMotorCommandSetpoints>& motor_feedback,
        const AnkleMap& ankle_map,
        AnkleTorqueState& state,
        FixedPolicyMotorCommand& command,
        std::string& error);

    std::shared_ptr<const JointMapping> mapping_;
    AnkleMotorLimitConfig ankle_motor_limits_;
    std::array<double, motor_base::kMaxMotors> motor_kp_;
    std::array<double, motor_base::kMaxMotors> motor_kd_;
    AnkleTorqueControlConfig torque_config_;
    LowPass2Coefficients low_pass_coeffs_;

    ankle_motor_ik::Solver left_ankle_ik_{ankle_motor_ik::AnkleSide::Left};
    ankle_motor_ik::Solver right_ankle_ik_{ankle_motor_ik::AnkleSide::Right};

    AnkleTorqueState left_ankle_torque_;
    AnkleTorqueState right_ankle_torque_;
};

}  // namespace inference
