#include "config/deploy_config.hpp"
#include "robot/action_processor.hpp"
#include "robot/observation_builder.hpp"
#include "robot/policy_command_worker.hpp"
#include "robot/robot_imu_session.hpp"
#include "robot/robot_motor_session.hpp"
#include "tool/tool.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

using Clock = std::chrono::steady_clock;
using MotorPositions = std::array<double, motor_base::kMotorNum>;
using ModelPositions = inference::ActionProcessor::ModelJointPositions;
using PolicyCommandWorker = inference::PolicyCommandWorker;

constexpr std::array<const char*, inference::kDof> kJointNames = {
    "left_hip_roll", "right_hip_roll", "left_hip_pitch", "right_hip_pitch",
    "left_hip_yaw", "right_hip_yaw", "left_knee_pitch", "right_knee_pitch",
    "left_ankle_pitch", "right_ankle_pitch", "left_ankle_roll", "right_ankle_roll"
};

constexpr double kDisplacementRad = 0.3;
constexpr double kZeroToleranceRad = 0.02;
constexpr auto kCommandPeriod = std::chrono::milliseconds(2);
constexpr auto kZeroDuration = std::chrono::milliseconds(3000);
constexpr auto kMoveDuration = std::chrono::milliseconds(1000);
constexpr auto kHoldDuration = std::chrono::milliseconds(1000);

volatile std::sig_atomic_t g_stop_signal = 0;

void signal_handler(int signal)
{
    g_stop_signal = signal;
}

bool valid_feedback(const inference::MotorStateSnapshot& state,
                    std::int64_t max_age_ns,
                    bool require_enabled)
{
    const auto now_ns = robot_base::monotonic_now_ns();
    if (state.timestamp_ns <= 0 || state.timestamp_ns > now_ns ||
        now_ns - state.timestamp_ns > max_age_ns ||
        !robot_base::finite_array(state.position_rad)) {
        std::cerr << "[MOTOR_SEQUENCE] Missing, stale or invalid motor feedback.\n";
        return false;
    }
    for (std::size_t id = 0; id < state.position_rad.size(); ++id) {
        if (!state.comm_ok[id] || (require_enabled && !state.enabled[id])) {
            std::cerr << "[MOTOR_SEQUENCE] Motor " << id
                      << " is offline or disabled.\n";
            return false;
        }
    }
    return true;
}

// Hold phases use identical from/to arrays. Commands are refreshed throughout
// both moves and holds; a single target followed by sleep would expire in 10 ms.
bool stream_motion(inference::RobotMotorSession& motors,
                   const MotorPositions& from,
                   const MotorPositions& to,
                   std::chrono::milliseconds duration,
                   std::int64_t max_age_ns,
                   const inference::RobotInterfaceConfig& config)
{
    const auto& motor_config = config.motor;
    const auto command_timeout_ns = robot_base::seconds_to_ns(
        config.safety.control_command_timeout_ms / 1000.0);
    const auto start = Clock::now();
    auto next_tick = start;
    while (!g_stop_signal) {
        std::string error;
        if (!motors.refresh_feedback_for_policy(error)) {
            std::cerr << "[MOTOR_SEQUENCE] " << error << '\n';
            return false;
        }
        if (!valid_feedback(motors.get_feedback_for_policy(), max_age_ns, true)) {
            return false;
        }
        const double elapsed_s =
            std::chrono::duration<double>(Clock::now() - start).count();
        const double duration_s = std::chrono::duration<double>(duration).count();
        const double alpha = std::clamp(elapsed_s / duration_s, 0.0, 1.0);
        const double blend = alpha * alpha * (3.0 - 2.0 * alpha);
        MotorPositions target{};
        for (std::size_t id = 0; id < target.size(); ++id) {
            target[id] = from[id] + blend * (to[id] - from[id]);
        }
        motor_base::CommandTiming timing;
        timing.produced_at_ns = robot_base::monotonic_now_ns();
        timing.valid_until_ns = timing.produced_at_ns + command_timeout_ns;
        bool submitted = false;
        if (motor_config.control_mode == motor_base::MotorControlMode::POSITION) {
            submitted = motors.submit_position_targets_rad(target, timing);
        } else if (motor_config.control_mode == motor_base::MotorControlMode::IMPEDANCE) {
            const auto setpoints = inference::ActionProcessor::make_position_impedance_setpoints(
                target, motor_config);
            submitted = motors.submit_impedance_setpoints(setpoints, timing);
        }
        if (!submitted) {
            std::cerr << "[MOTOR_SEQUENCE] Failed to submit motor targets.\n";
            return false;
        }
        if (alpha >= 1.0) {
            return true;
        }
        next_tick = std::max(next_tick + kCommandPeriod, Clock::now());
        std::this_thread::sleep_until(next_tick);
    }
    return false;
}

bool stream_model_motion(inference::RobotMotorSession& motors,
                         PolicyCommandWorker& worker,
                         const ModelPositions& from,
                         const ModelPositions& to,
                         std::chrono::milliseconds duration,
                         const inference::RobotInterfaceConfig& config,
                         std::uint64_t& sequence)
{
    const auto start = Clock::now();
    auto next_tick = start;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(config.policy.step_dt));
    const auto max_age_ns = robot_base::seconds_to_ns(
        config.sensor_guard.max_motor_sample_age_s);
    while (!g_stop_signal) {
        std::string error;
        if (!worker.check_health(error)) {
            std::cerr << "[MOTOR_SEQUENCE] Policy command worker failed: " << error << '\n';
            return false;
        }
        if (!motors.refresh_feedback_for_policy(error)) {
            std::cerr << "[MOTOR_SEQUENCE] " << error << '\n';
            return false;
        }
        const auto& state = motors.get_feedback_for_policy();
        if (!valid_feedback(state, max_age_ns, true)) {
            return false;
        }
        const double alpha = std::clamp(
            std::chrono::duration<double>(Clock::now() - start).count() /
            std::chrono::duration<double>(duration).count(), 0.0, 1.0);
        const double blend = alpha * alpha * (3.0 - 2.0 * alpha);
        inference::PolicyTargetFrame frame;
        frame.policy_seq = ++sequence;
        for (std::size_t dof = 0; dof < to.size(); ++dof) {
            frame.target_model_rad[dof] =
                from[dof] + blend * (to[dof] - from[dof]);
        }
        // Inject angles at the same boundary as policy_step's processed model
        // targets. The production worker applies mapping/ankle control and
        // preserves its normal feedback-age and command/target deadlines.
        worker.publish_target(frame);
        if (alpha >= 1.0) {
            return true;
        }
        next_tick = std::max(next_tick + period, Clock::now());
        std::this_thread::sleep_until(next_tick);
    }
    return false;
}

bool report_model_position(inference::RobotMotorSession& motors,
                           inference::ObservationBuilder& feedback_builder,
                           const inference::JointMapping& mapping,
                           const inference::RobotInterfaceConfig& config,
                           std::size_t dof,
                           double target)
{
    std::string error;
    if (!motors.refresh_feedback_for_policy(error)) {
        std::cerr << "[MOTOR_SEQUENCE] " << error << '\n';
        return false;
    }
    const auto& state = motors.get_feedback_for_policy();
    if (!valid_feedback(state, robot_base::seconds_to_ns(
            config.sensor_guard.max_motor_sample_age_s), true)) {
        return false;
    }
    // Only the joint position terms are read. No IMU or model is started.
    // Unit position scales and a zero reference expose absolute model angles
    // through the same direction/FK conversion used for policy observations.
    inference::AhrsStateSnapshot placeholder_ahrs;
    placeholder_ahrs.projected_gravity_valid = true;
    inference::PolicyObservation observation{};
    if (!feedback_builder.build(state, placeholder_ahrs, {}, observation, error)) {
        std::cerr << "[MOTOR_SEQUENCE] Model feedback conversion failed: " << error << '\n';
        return false;
    }
    using namespace inference::policy_observation;
    const std::size_t position_offset = kFrameStack *
        (kBaseAngVelSize + kProjectedGravitySize + kVelocityCommandsSize +
         (config.policy.gait.enabled ? kGaitPhaseSize : 0));
    const double measured = observation[position_offset + (kFrameStack - 1) * inference::kDof + dof];
    std::cout << "[MOTOR_SEQUENCE] Model DOF " << dof << " " << kJointNames[dof]
              << ": target=" << target << " rad, measured=" << measured
              << " rad, error=" << target - measured << " rad";
    if (!mapping.is_model_dof(static_cast<int>(dof))) {
        const int motor = mapping.get_motor_id(static_cast<int>(dof));
        std::cout << ", M" << motor << " target=" << mapping.get_motor_direction(motor) * target
                  << " rad, measured=" << state.position_rad[motor] << " rad";
    } else {
        const auto& ankle = (static_cast<int>(dof) == mapping.left_ankle().model_pitch_dof ||
                             static_cast<int>(dof) == mapping.left_ankle().model_roll_dof)
                                ? mapping.left_ankle() : mapping.right_ankle();
        std::cout << ", ankle torque control: M" << ankle.upper_motor_index
                  << " measured=" << state.position_rad[ankle.upper_motor_index]
                  << " rad, M" << ankle.lower_motor_index
                  << " measured=" << state.position_rad[ankle.lower_motor_index] << " rad";
    }
    std::cout << '\n' << std::flush;
    return true;
}

bool run_model_sequence(inference::RobotMotorSession& motors,
                        PolicyCommandWorker& worker,
                        inference::ObservationBuilder& feedback_builder,
                        const inference::JointMapping& mapping,
                        const inference::RobotInterfaceConfig& config)
{
    const ModelPositions zero{};
    std::uint64_t sequence = 0;
    for (std::size_t dof = 0; dof < zero.size() && !g_stop_signal; ++dof) {
        ModelPositions displaced = zero;
        displaced[dof] = kDisplacementRad;
        std::cout << "[MOTOR_SEQUENCE] Model DOF " << dof << " " << kJointNames[dof]
                  << ": 0 -> +" << kDisplacementRad << " rad -> 0\n" << std::flush;
        if (!stream_model_motion(motors, worker, zero, displaced, kMoveDuration, config, sequence) ||
            !stream_model_motion(motors, worker, displaced, displaced, kHoldDuration, config, sequence) ||
            !report_model_position(motors, feedback_builder, mapping, config, dof, kDisplacementRad) ||
            !stream_model_motion(motors, worker, displaced, zero, kMoveDuration, config, sequence) ||
            !stream_model_motion(motors, worker, zero, zero, kHoldDuration, config, sequence) ||
            !report_model_position(motors, feedback_builder, mapping, config, dof, 0.0)) {
            return false;
        }
    }
    return !g_stop_signal;
}

bool run_sequence(inference::RobotMotorSession& motors,
                  inference::ActionProcessor& action_processor,
                  inference::ObservationBuilder& feedback_builder,
                  const inference::JointMapping& mapping,
                  const inference::RobotInterfaceConfig& config)
{
    const auto max_age_ns = robot_base::seconds_to_ns(config.sensor_guard.max_motor_sample_age_s);
    if (g_stop_signal || !motors.initialize() || g_stop_signal) {
        return false;
    }

    // Capture the physical motor positions while stopped, before enabling.
    std::string error;
    if (!motors.refresh_feedback_for_policy(error)) {
        std::cerr << "[MOTOR_SEQUENCE] " << error << '\n';
        return false;
    }
    const auto& initial_state = motors.get_feedback_for_policy();
    if (!valid_feedback(initial_state, max_age_ns, false)) {
        return false;
    }
    const MotorPositions initial = initial_state.position_rad;
    const MotorPositions zero{};
    std::cout << std::fixed << std::setprecision(4);
    for (std::size_t id = 0; id < initial.size(); ++id) {
        std::cout << "[MOTOR_SEQUENCE] ID " << id << " start=" << initial[id]
                  << " rad, direction=" << mapping.get_motor_direction(static_cast<int>(id)) << '\n';
    }
    std::cout << "[MOTOR_SEQUENCE] Move all motors to physical zero (3 s), hold (1 s).\n"
              << std::flush;
    if (g_stop_signal || !motors.restart() ||
        !stream_motion(motors, initial, zero, kZeroDuration, max_age_ns, config) ||
        !stream_motion(motors, zero, zero, kHoldDuration, max_age_ns, config)) {
        return false;
    }

    if (!motors.refresh_feedback_for_policy(error)) {
        std::cerr << "[MOTOR_SEQUENCE] " << error << '\n';
        return false;
    }
    const auto& zero_state = motors.get_feedback_for_policy();
    if (!valid_feedback(zero_state, max_age_ns, true)) {
        return false;
    }
    for (std::size_t id = 0; id < zero.size(); ++id) {
        if (std::abs(zero_state.position_rad[id]) > kZeroToleranceRad) {
            std::cerr << "[MOTOR_SEQUENCE] Motor " << id
                      << " did not reach zero: measured=" << zero_state.position_rad[id]
                      << " rad, tolerance=" << kZeroToleranceRad << " rad.\n";
            return false;
        }
    }

    std::atomic<std::int64_t> first_policy_started_ns{0};
    PolicyCommandWorker worker(config, motors, action_processor, first_policy_started_ns);
    if (!worker.start(zero)) {
        return false;
    }
    first_policy_started_ns.store(robot_base::monotonic_now_ns(), std::memory_order_release);
    const bool completed = run_model_sequence(motors, worker, feedback_builder, mapping, config);
    motors.request_stop();
    worker.stop();
    return completed;
}

}  // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::string config_path = ROBOT_DEPLOY_CONFIG_PATH;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--help") == 0) {
            std::cout << "usage: motor_sequence_test [--config <deploy.yaml>]\n"
                         "First move all physical motors to 0 rad (3 s), hold (1 s).\n"
                         "Require all zero-position errors <= 0.02 rad before testing.\n"
                         "Then MODEL DOFs 0..11: 0 -> +" << kDisplacementRad <<
                         " rad -> 0; other model joints hold zero.\n"
                         "Uses the same PolicyCommandWorker/ActionProcessor as policy_test.\n"
                         "Direct joints use mapped impedance targets; ankle pitch/roll use paired-motor torque control.\n"
                         "Move/return: 1 s each; holds: 1 s; target period: YAML step_dt.\n"
                         "Reports model target, measured model angle and physical motor feedback.\n"
                         "Ctrl+C stops all motors without a return motion.\n";
            return 0;
        }
        if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            config_path = argv[++i];
        } else {
            std::cerr << "usage: motor_sequence_test [--config <deploy.yaml>]\n";
            return 1;
        }
    }

    inference::RobotInterfaceConfig config;
    std::string error;
    if (!inference::load_deploy_config(config_path, config, error)) {
        std::cerr << "[MOTOR_SEQUENCE] Config error: " << error << '\n';
        return 1;
    }
    config.motor.print_motors_info = false;
    config.recorder.enabled = false;
    std::unique_ptr<const inference::JointMapping> mapping;
    try {
        mapping = std::make_unique<const inference::JointMapping>(
            static_cast<int>(inference::kDof), config.joint_mapping);
    } catch (const std::invalid_argument& error) {
        std::cerr << "[MOTOR_SEQUENCE] Invalid joint mapping: " << error.what() << '\n';
        return 1;
    }
    inference::ActionProcessor action_processor(config, *mapping);
    // 独立配置只用于反馈测量：零参考姿态、单位位置缩放得到绝对关节角。
    auto feedback_config = config;
    feedback_config.default_joint_pos_rad.fill(0.0);
    feedback_config.observation_scales.dof_pos_scale.fill(1.0);
    inference::ObservationBuilder feedback_builder(feedback_config, *mapping);
    inference::RobotMotorSession motors(config);
    const bool completed = run_sequence(
        motors, action_processor, feedback_builder, *mapping, config);

    std::cout << "[MOTOR_SEQUENCE] Stopping all motors.\n" << std::flush;
    const auto shutdown_result = motors.shutdown(motors.request_stop());
    if (shutdown_result != inference::ShutdownResult::Confirmed) {
        std::cerr << (shutdown_result == inference::ShutdownResult::ReleasedWithCommLoss
            ? "[MOTOR_SEQUENCE] EtherCAT communication lost; STOP not confirmed; controller released.\n"
            : "[MOTOR_SEQUENCE] Stop not confirmed; RT retained. Destruction will attempt shutdown once and terminate if it fails.\n");
        return 1;
    }
    if (g_stop_signal) {
        std::cout << "[MOTOR_SEQUENCE] Interrupted; motors stopped.\n";
        return 128 + g_stop_signal;
    }
    if (!completed) {
        std::cerr << "[MOTOR_SEQUENCE] Test failed; motors stopped.\n";
        return 1;
    }
    std::cout << "[MOTOR_SEQUENCE] All 12 model joint sequences completed; motors stopped.\n";
    return 0;
}
