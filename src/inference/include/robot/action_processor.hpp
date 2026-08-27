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
#include <vector>

namespace inference::robot_detail {

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
        std::array<double, policy_observation::kDof> target_effort_permille{};
        std::size_t setpoint_count{0};
    };

    using FixedModelTarget =
        std::array<double, policy_observation::kDof>;

    ActionProcessor(std::shared_ptr<const JointMapping> mapping,
                    PolicyConfig policy_config,
                    AnkleMotorLimitConfig ankle_motor_limits,
                    std::vector<double> motor_kp,
                    std::vector<double> motor_kd,
                    AnkleTorqueControlConfig torque_config);

    void reset_runtime_state();

    bool build_motor_targets(const std::vector<double>& target_q_model_rad,
                             std::vector<double>& target_motor_rad,
                             std::string& error);

    bool build_reset_start_model_pose(const std::vector<double>& current_motor_q,
                                      const std::vector<double>& target_model_q,
                                      std::vector<double>& start_model_q,
                                      std::string& error) const;

    bool build_policy_impedance_command(
        const FixedModelTarget& target_q_model_rad,
        const std::array<motor_base::MotorStatusSnapshot,
                         motor_base::kMaxMotorCommandSetpoints>& motor_feedback,
        FixedPolicyMotorCommand& command,
        std::string& error);

private:
    struct AnkleIkState {
        ankle_motor_ik::Solver solver;
        double last_upper_motor = 0.0;
        double last_lower_motor = 0.0;
        bool solved = false;

        void reset();
    };

    struct LowPass2State {
        double x1 = 0.0;
        double x2 = 0.0;
        double y1 = 0.0;
        double y2 = 0.0;

        void reset();
    };

    struct AnkleTorqueState {
        ankle_motor_fk::Solver fk_solver;
        LowPass2State pitch_filter;
        LowPass2State roll_filter;

        void reset(double roll = 0.0, double pitch = 0.0);
    };

    int dof_count() const noexcept;

    bool apply_ankle_ik(const std::vector<double>& target_q_model_rad,
                        std::vector<double>& target_motor_rad,
                        const AnkleParallelMap& ankle_map,
                        AnkleIkState& state,
                        std::string& error);

    bool apply_ankle_torque_control(
        const FixedModelTarget& target_q_model_rad,
        const std::array<motor_base::MotorStatusSnapshot,
                         motor_base::kMaxMotorCommandSetpoints>& motor_feedback,
        const AnkleParallelMap& ankle_map,
        AnkleTorqueState& state,
        FixedPolicyMotorCommand& command,
        std::string& error);

    std::shared_ptr<const JointMapping> mapping_;
    PolicyConfig policy_config_;
    AnkleMotorLimitConfig ankle_motor_limits_;
    std::vector<double> motor_kp_;
    std::vector<double> motor_kd_;
    AnkleTorqueControlConfig torque_config_;
    LowPass2Coefficients low_pass_coeffs_;

    AnkleIkState left_ankle_ik_;
    AnkleIkState right_ankle_ik_;

    AnkleTorqueState left_ankle_torque_;
    AnkleTorqueState right_ankle_torque_;
};

}  // namespace inference::robot_detail
