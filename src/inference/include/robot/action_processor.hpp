#pragma once

#include "kinematics/ankle_motor_fk.hpp"
#include "kinematics/ankle_motor_ik.hpp"
#include "robot/joint_mapping.hpp"
#include "robot/robot_config.hpp"

#include <memory>
#include <string>
#include <vector>

namespace inference {
struct MotorStateSnapshot;
}

namespace inference::robot_detail {

class ActionProcessor {
public:
    struct PolicyMotorCommand {
        std::vector<motor_base::ImpedanceSetpoint> setpoints;
        std::vector<double> target_effort_permille;
    };

    ActionProcessor(std::shared_ptr<const JointMapping> mapping,
                    PolicyConfig policy_config);

    void reset_runtime_state();

    bool build_motor_targets(const std::vector<double>& target_q_model_rad,
                             std::vector<double>& target_motor_rad,
                             std::string& error);

    bool build_reset_start_model_pose(const std::vector<double>& current_motor_q,
                                      const std::vector<double>& target_model_q,
                                      std::vector<double>& start_model_q,
                                      std::string& error) const;

    bool build_policy_impedance_command(
        const std::vector<double>& target_q_model_rad,
        const MotorStateSnapshot& motor_state,
        const std::vector<double>& motor_kp,
        const std::vector<double>& motor_kd,
        const AnkleTorqueControlConfig& torque_config,
        PolicyMotorCommand& command,
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
    bool has_relative_limits() const;

    bool apply_ankle_ik(const std::vector<double>& target_q_model_rad,
                        std::vector<double>& target_motor_rad,
                        const AnkleParallelMap& ankle_map,
                        AnkleIkState& state,
                        std::string& error);

    bool apply_ankle_torque_control(
        const std::vector<double>& target_q_model_rad,
        const MotorStateSnapshot& motor_state,
        const char* ankle_name,
        const AnkleParallelMap& ankle_map,
        const AnkleTorqueControlConfig& torque_config,
        AnkleTorqueState& state,
        PolicyMotorCommand& command,
        std::string& error);

    std::shared_ptr<const JointMapping> mapping_;
    PolicyConfig policy_config_;
    AnkleIkState left_ankle_ik_;
    AnkleIkState right_ankle_ik_;
    AnkleTorqueState left_ankle_torque_;
    AnkleTorqueState right_ankle_torque_;
};

}  // namespace inference::robot_detail
