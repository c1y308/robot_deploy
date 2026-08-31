#include "kinematics/ankle_motor_ik.hpp"
#include "robot/robot_motor_session.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>
#include <vector>

namespace {

constexpr int kDof = 12;
constexpr int kLeftUpperMotor = 4;
constexpr int kLeftLowerMotor = 5;
constexpr int kRightUpperMotor = 10;
constexpr int kRightLowerMotor = 11;
constexpr int kNonAnkleMotorCount = kDof - 4;

constexpr double kTargetPitchDeg = 0;
constexpr double kTargetRollDeg  = 15;

constexpr double kTransitionSeconds = 2.0;
constexpr double kZeroSettleSeconds = 2.0;
constexpr double kHoldSeconds = 10.0;
constexpr int kControlHz = 50;
constexpr int kTransitionSteps =
    static_cast<int>(kTransitionSeconds * static_cast<double>(kControlHz));
constexpr int kZeroSettleSteps =
    static_cast<int>(kZeroSettleSeconds * static_cast<double>(kControlHz));
constexpr int kHoldSteps =
    static_cast<int>(kHoldSeconds * static_cast<double>(kControlHz));
constexpr std::chrono::milliseconds kControlPeriod{1000 / kControlHz};

const std::array<int, kDof> kMotorToModelDirection = {
    -1, -1, 1,  1, -1, -1,
    -1,  1, 1, -1, -1, -1
};

const std::array<int, kNonAnkleMotorCount> kNonAnkleMotors = {
    0, 1, 2, 3, 6, 7, 8, 9
};

struct PoseTarget {
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
};

std::atomic<bool> g_stop_requested{false};

void signal_handler(int)
{
    g_stop_requested.store(true);
}

bool finite_vector(const std::vector<double>& values)
{
    for (double value : values) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}

bool safety_countdown()
{
    for (int remaining = 3; remaining > 0; --remaining) {
        if (g_stop_requested.load()) {
            return false;
        }
        std::cout << "[ANKLE_IK_TEST] Starting hardware in "
                  << remaining << " seconds. Press Ctrl+C to cancel.\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return !g_stop_requested.load();
}

inference::MotorConfig make_motor_config()
{
    inference::MotorConfig config;
    config.num_motors = kDof;
    config.ethercat_ifname = "enp8s0";
    config.control_mode = motor_base::MotorControlMode::IMPEDANCE;
    config.mit_kp = {
        180.0, 230.0, 180.0, 230.0, 180.0, 180.0,
        180.0, 230.0, 180.0, 230.0, 180.0, 180.0
    };
    config.mit_kd = {
        10.54, 10.54, 10.54, 10.54, 10.54, 10.54,
        10.54, 10.54, 10.54, 10.54, 10.54, 10.54
    };
    return config;
}

bool read_current_motor_targets(inference::RobotMotorSession& motors,
                                std::vector<double>& target_motor_rad)
{
    const inference::MotorStateSnapshot snapshot = motors.get_motor_snapshot();
    if (static_cast<int>(snapshot.position_rad.size()) != kDof) {
        std::cerr << "[ANKLE_IK_TEST] Expected " << kDof
                  << " motor positions, got "
                  << snapshot.position_rad.size() << ".\n";
        return false;
    }
    if (!finite_vector(snapshot.position_rad)) {
        std::cerr << "[ANKLE_IK_TEST] Initial motor positions contain non-finite values.\n";
        return false;
    }

    target_motor_rad = snapshot.position_rad;
    return true;
}

void set_non_ankle_zero_targets(std::vector<double>& target_motor_rad)
{
    for (int motor_index : kNonAnkleMotors) {
        target_motor_rad[motor_index] = 0.0;
    }
}

bool send_initial_non_ankle_zero(inference::RobotMotorSession& motors,
                                 std::vector<double>& target_motor_rad)
{
    set_non_ankle_zero_targets(target_motor_rad);
    if (!motors.apply_targets_rad(target_motor_rad)) {
        std::cerr << "[ANKLE_IK_TEST] Failed to hold non-ankle motors at zero.\n";
        return false;
    }
    return true;
}

bool solve_pose_targets(double roll_rad,
                        double pitch_rad,
                        ankle_motor_ik::Solver& left_solver,
                        ankle_motor_ik::Solver& right_solver,
                        std::vector<double>& target_motor_rad)
{
    const ankle_motor_ik::MotorAngles left = left_solver.solve(roll_rad, pitch_rad);
    const ankle_motor_ik::MotorAngles right = right_solver.solve(roll_rad, pitch_rad);
    if (!left.reachable() || !right.reachable()) {
        std::cerr << "[ANKLE_IK_TEST] IK unreachable at roll "
                  << ankle_motor_ik::rad_to_deg(roll_rad)
                  << " deg, pitch "
                  << ankle_motor_ik::rad_to_deg(pitch_rad)
                  << " deg.\n";
        return false;
    }
    if (!std::isfinite(left.motor1) ||
        !std::isfinite(left.motor2) ||
        !std::isfinite(right.motor1) ||
        !std::isfinite(right.motor2)) {
        std::cerr << "[ANKLE_IK_TEST] IK returned non-finite motor target.\n";
        return false;
    }

    target_motor_rad[kLeftUpperMotor] =
        kMotorToModelDirection[kLeftUpperMotor] * left.motor1;
    target_motor_rad[kLeftLowerMotor] =
        kMotorToModelDirection[kLeftLowerMotor] * left.motor2;
    target_motor_rad[kRightUpperMotor] =
        kMotorToModelDirection[kRightUpperMotor] * right.motor1;
    target_motor_rad[kRightLowerMotor] =
        kMotorToModelDirection[kRightLowerMotor] * right.motor2;
    return true;
}

bool send_pose_target(inference::RobotMotorSession& motors,
                      double roll_rad,
                      double pitch_rad,
                      ankle_motor_ik::Solver& left_solver,
                      ankle_motor_ik::Solver& right_solver,
                      std::vector<double>& target_motor_rad)
{
    if (!solve_pose_targets(roll_rad,
                            pitch_rad,
                            left_solver,
                            right_solver,
                            target_motor_rad)) {
        return false;
    }
    if (!motors.apply_targets_rad(target_motor_rad)) {
        std::cerr << "[ANKLE_IK_TEST] Failed to apply motor targets.\n";
        return false;
    }
    return true;
}

bool run_transition(inference::RobotMotorSession& motors,
                    const PoseTarget& start_pose,
                    const PoseTarget& end_pose,
                    ankle_motor_ik::Solver& left_solver,
                    ankle_motor_ik::Solver& right_solver,
                    std::vector<double>& target_motor_rad)
{
    auto next_tick = std::chrono::steady_clock::now();
    for (int step = 1; step <= kTransitionSteps; ++step) {
        if (g_stop_requested.load()) {
            return false;
        }

        const double alpha =
            static_cast<double>(step) / static_cast<double>(kTransitionSteps);
        const double roll_rad =
            start_pose.roll_rad * (1.0 - alpha) + end_pose.roll_rad * alpha;
        const double pitch_rad =
            start_pose.pitch_rad * (1.0 - alpha) + end_pose.pitch_rad * alpha;
        if (!send_pose_target(motors,
                              roll_rad,
                              pitch_rad,
                              left_solver,
                              right_solver,
                              target_motor_rad)) {
            return false;
        }

        next_tick += kControlPeriod;
        std::this_thread::sleep_until(next_tick);
    }
    return !g_stop_requested.load();
}

bool hold_pose(inference::RobotMotorSession& motors,
               const PoseTarget& pose,
               ankle_motor_ik::Solver& left_solver,
               ankle_motor_ik::Solver& right_solver,
               std::vector<double>& target_motor_rad)
{
    auto next_tick = std::chrono::steady_clock::now();
    for (int step = 0; step < kHoldSteps; ++step) {
        if (g_stop_requested.load()) {
            return false;
        }
        if (!send_pose_target(motors,
                              pose.roll_rad,
                              pose.pitch_rad,
                              left_solver,
                              right_solver,
                              target_motor_rad)) {
            return false;
        }
        if (step == kHoldSteps / 2) {
            const inference::MotorStateSnapshot snapshot = motors.get_motor_snapshot();
            std::cout << "[ANKLE_IK_TEST] Hold halfway motor positions: "
                      << "M" << kLeftUpperMotor << "="
                      << snapshot.position_rad[kLeftUpperMotor] << " rad, "
                      << "M" << kLeftLowerMotor << "="
                      << snapshot.position_rad[kLeftLowerMotor] << " rad, "
                      << "M" << kRightUpperMotor << "="
                      << snapshot.position_rad[kRightUpperMotor] << " rad, "
                      << "M" << kRightLowerMotor << "="
                      << snapshot.position_rad[kRightLowerMotor] << " rad.\n";
        }

        next_tick += kControlPeriod;
        std::this_thread::sleep_until(next_tick);
    }
    return !g_stop_requested.load();
}

bool settle_zero_pitch(inference::RobotMotorSession& motors,
                       ankle_motor_ik::Solver& left_solver,
                       ankle_motor_ik::Solver& right_solver,
                       std::vector<double>& target_motor_rad)
{
    auto next_tick = std::chrono::steady_clock::now();
    for (int step = 0; step < kZeroSettleSteps; ++step) {
        if (g_stop_requested.load()) {
            return false;
        }
        if (!send_pose_target(motors,
                              0.0,
                              0.0,
                              left_solver,
                              right_solver,
                              target_motor_rad)) {
            return false;
        }

        next_tick += kControlPeriod;
        std::this_thread::sleep_until(next_tick);
    }
    return !g_stop_requested.load();
}

}  // namespace

int main()
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    if (!safety_countdown()) {
        std::cout << "[ANKLE_IK_TEST] Startup canceled.\n";
        return 0;
    }

    inference::RobotMotorSession motors(make_motor_config());

    std::cout << "[ANKLE_IK_TEST] Initializing motors...\n";
    if (!motors.initialize_and_start()) {
        std::cerr << "[ANKLE_IK_TEST] initialize_and_start failed.\n";
        return 1;
    }

    std::cout << "[ANKLE_IK_TEST] Restarting motors...\n";
    if (!motors.restart(-1)) {
        std::cerr << "[ANKLE_IK_TEST] restart failed.\n";
        motors.deinitialize();
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::vector<double> target_motor_rad;
    if (!read_current_motor_targets(motors, target_motor_rad)) {
        motors.deinitialize();
        return 1;
    }

    std::cout << "[ANKLE_IK_TEST] Hold non-ankle motors M0-M3/M6-M9 at 0 rad.\n";
    if (!send_initial_non_ankle_zero(motors, target_motor_rad)) {
        motors.deinitialize();
        return 1;
    }

    ankle_motor_ik::Solver left_solver(
        kMotorToModelDirection[kLeftUpperMotor] * target_motor_rad[kLeftUpperMotor],
        kMotorToModelDirection[kLeftLowerMotor] * target_motor_rad[kLeftLowerMotor]);
    ankle_motor_ik::Solver right_solver(
        kMotorToModelDirection[kRightUpperMotor] * target_motor_rad[kRightUpperMotor],
        kMotorToModelDirection[kRightLowerMotor] * target_motor_rad[kRightLowerMotor]);

    const PoseTarget zero_pose{};
    const PoseTarget positive_pose{
        ankle_motor_ik::deg_to_rad(kTargetRollDeg),
        ankle_motor_ik::deg_to_rad(kTargetPitchDeg)
    };
    const PoseTarget negative_pose{
        ankle_motor_ik::deg_to_rad(-kTargetRollDeg),
        ankle_motor_ik::deg_to_rad(-kTargetPitchDeg)
    };

    bool ok = true;
    if (ok) {
        std::cout << "[ANKLE_IK_TEST] Return both ankles to roll=0 deg, pitch=0 deg before +"
                  << kTargetPitchDeg << " deg pitch motion.\n";
        ok = settle_zero_pitch(motors,
                               left_solver,
                               right_solver,
                               target_motor_rad);
    }
    if (ok) {
        std::cout << "[ANKLE_IK_TEST] Move both ankles to roll="
                  << kTargetRollDeg << " deg, pitch=+"
                  << kTargetPitchDeg << " deg.\n";
        ok = run_transition(motors,
                            zero_pose,
                            positive_pose,
                            left_solver,
                            right_solver,
                            target_motor_rad);
    }
    if (ok) {
        std::cout << "[ANKLE_IK_TEST] Hold roll="
                  << kTargetRollDeg << " deg, pitch=+"
                  << kTargetPitchDeg << " deg for 5 seconds.\n";
        ok = hold_pose(motors,
                       positive_pose,
                       left_solver,
                       right_solver,
                       target_motor_rad);
    }
    if (ok) {
        std::cout << "[ANKLE_IK_TEST] Move both ankles to roll="
                  << -kTargetRollDeg << " deg, pitch=-"
                  << kTargetPitchDeg << " deg.\n";
        ok = run_transition(motors,
                            positive_pose,
                            negative_pose,
                            left_solver,
                            right_solver,
                            target_motor_rad);
    }
    if (ok) {
        std::cout << "[ANKLE_IK_TEST] Hold roll="
                  << -kTargetRollDeg << " deg, pitch=-"
                  << kTargetPitchDeg << " deg for 5 seconds.\n";
        ok = hold_pose(motors,
                       negative_pose,
                       left_solver,
                       right_solver,
                       target_motor_rad);
    }
    if (ok) {
        std::cout << "[ANKLE_IK_TEST] Return both ankles to roll=0 deg, pitch=0 deg.\n";
        ok = run_transition(motors,
                            negative_pose,
                            zero_pose,
                            left_solver,
                            right_solver,
                            target_motor_rad);
    }

    std::cout << "[ANKLE_IK_TEST] Stopping motors and releasing hardware...\n";
    motors.deinitialize();

    if (g_stop_requested.load()) {
        std::cout << "[ANKLE_IK_TEST] Interrupted.\n";
        return 1;
    }
    if (!ok) {
        std::cerr << "[ANKLE_IK_TEST] Failed.\n";
        return 1;
    }

    std::cout << "[ANKLE_IK_TEST] Completed.\n";
    return 0;
}
