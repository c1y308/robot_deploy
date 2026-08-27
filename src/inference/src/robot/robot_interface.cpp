#include "robot/robot_interface.hpp"
#include "base/tool.hpp"
#include "robot/action_processor.hpp"
#include "robot/joint_mapping.hpp"
#include "robot/observation_builder.hpp"
#include "robot/target_interpolator.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>


namespace inference {

using robot_base::finite_array;
using robot_base::finite_vector;

namespace {

/* 检查 action 截断范围中的上下界是否全部为有限值。 */
bool finite_action_clip_ranges(const std::vector<std::array<double, 2>>& ranges)
{
    return std::all_of(ranges.begin(), ranges.end(), [](const auto& range) {
        return std::isfinite(range[0]) && std::isfinite(range[1]);
    });
}

void fill_record_motor_state(const MotorStateSnapshot& motor_state,
                             std::size_t motor_count,
                             InferenceRecord& record)
{
    record.state_timestamp_ns = motor_state.timestamp_ns;
    for (std::size_t i = 0; i < motor_count; ++i) {
        record.rx_pos_rad[i] = motor_state.position_rad[i];
        record.rx_vel_rad_s[i] = motor_state.velocity_rad_s[i];
        record.torque_percent[i] = motor_state.torque_percent[i];
        record.comm_ok[i] = motor_state.comm_ok[i];
        record.enabled[i] = motor_state.enabled[i];
    }
}

}  // namespace

/* 保存外部传入的接口配置，后续由初始化函数按模块使用。 */
RobotInterface::RobotInterface(RobotInterfaceConfig config)
    : config_(std::move(config)),
      motor_session_(config_.motor),    // 构造 motor_session_
      imu_session_(config_.imu)         // 构造 imu_session_
{
}

/* 析构时释放策略、IMU 和电机资源，保证后台线程退出。 */
RobotInterface::~RobotInterface() {
    shutdown();
}

/* 编排机器人完整运行态：电机、策略、IMU 和站立姿态复位。 */
bool RobotInterface::initialize() {
    if (initialized_.load()) {
        return true;
    }

    shutdown();

    if (!validate_policy_config()) {
        shutdown();
        return false;
    }
    if (!initialize_model_processors()) {
        shutdown();
        return false;
    }
    if (!motor_session_.initialize_and_start()) {
        shutdown();
        return false;
    }
    if (!load_policy()) {
        shutdown();
        return false;
    }
    if (!motor_session_.restart(-1)) {
        shutdown();
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (!imu_session_.initialize_and_start()) {
        shutdown();
        return false;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (!reset_joints()) {
        shutdown();
        return false;
    }

    initialized_.store(true);
    if (!start_policy_command_worker()) {
        initialized_.store(false);
        shutdown();
        return false;
    }

    return true;
}


/* 幂等停机：先卸载策略和日志，再释放 IMU，最后释放电机。 */
void RobotInterface::shutdown() {
    stop_policy_command_worker();
    initialized_.store(false);
    unload_policy();
    imu_session_.deinitialize();
    motor_session_.deinitialize();
}


/* 检查策略模型路径、映射、缩放、限位和观测参数是否有效。 */
bool RobotInterface::validate_policy_config() const {
    auto fail = [](const std::string& message) {
        std::cerr << "[RobotInterface] invalid policy config: "
                  << message << "\n";
        return false;
    };

    if (config_.motor.num_motors != static_cast<int>(PolicyRuntime::kDof)) {
        return fail("num_motors must be 12");
    }
    if (config_.motor.control_mode != motor_base::MotorControlMode::IMPEDANCE) {
        return fail("policy ankle torque control requires motor.control_mode IMPEDANCE");
    }
    if (config_.policy.model_path.empty()) {
        return fail("policy.model_path is empty");
    }
    if (config_.policy.action_clip.size() != PolicyRuntime::kDof) {
        return fail("action_clip must have 12 ranges");
    }
    if (config_.policy.stand_pose_rad.size() != PolicyRuntime::kDof) {
        return fail("stand_pose_rad must have 12 values");
    }
    if (config_.policy.action_scale.size() != PolicyRuntime::kDof) {
        return fail("action_scale must have 12 values");
    }
    if (config_.policy.dof_pos_scale.size() != PolicyRuntime::kDof ||
        config_.policy.dof_vel_scale.size() != PolicyRuntime::kDof) {
        return fail("dof_pos_scale and dof_vel_scale must have 12 values");
    }

    if (!finite_vector(config_.policy.stand_pose_rad) ||
        !finite_action_clip_ranges(config_.policy.action_clip) ||
        !finite_vector(config_.policy.action_scale) ||
        !finite_vector(config_.policy.dof_pos_scale) ||
        !finite_vector(config_.policy.dof_vel_scale)) {
        return fail("all vector policy values must be finite");
    }
    if (!finite_array(config_.policy.command_scale) ||
        !finite_array(config_.policy.body_ang_vel_scale)) {
        return fail("command/body_ang_vel scales must be finite");
    }
    if (!finite_array(config_.ankle_motor_limits.min_rad) ||
        !finite_array(config_.ankle_motor_limits.max_rad)) {
        return fail("ankle motor physical limits must be finite");
    }
    if (!std::isfinite(config_.policy.raw_action_clip) ||
        config_.policy.raw_action_clip <= 0.0) {
        return fail("policy.raw_action_clip must be a finite positive value");
    }
    if (!finite_array(config_.ankle_torque.virtual_kp) ||
        !finite_array(config_.ankle_torque.virtual_kd)) {
        return fail("ankle_torque virtual_kp/virtual_kd must be finite");
    }
    if (config_.ankle_torque.virtual_kp[0] < 0.0 ||
        config_.ankle_torque.virtual_kp[1] < 0.0 ||
        config_.ankle_torque.virtual_kd[0] < 0.0 ||
        config_.ankle_torque.virtual_kd[1] < 0.0) {
        return fail("ankle_torque virtual_kp/virtual_kd must be non-negative");
    }
    if (!std::isfinite(config_.ankle_torque.filter_cutoff_rad_s) ||
        config_.ankle_torque.filter_cutoff_rad_s <= 0.0 ||
        !std::isfinite(config_.ankle_torque.filter_dt_s) ||
        config_.ankle_torque.filter_dt_s <= 0.0) {
        return fail("ankle_torque filter cutoff and dt must be finite positive values");
    }
    if (!std::isfinite(config_.ankle_torque.motor_rated_torque_nm) ||
        config_.ankle_torque.motor_rated_torque_nm <= 0.0) {
        return fail("ankle_torque motor_rated_torque_nm must be a finite positive value");
    }
    if (!std::isfinite(config_.ankle_torque.target_torque_limit_permille) ||
        config_.ankle_torque.target_torque_limit_permille <= 0.0 ||
        config_.ankle_torque.target_torque_limit_permille > 32767.0) {
        return fail("ankle_torque target_torque_limit_permille must be explicitly configured in (0, 32767]");
    }
    if (!std::isfinite(config_.policy.step_dt) ||
        config_.policy.step_dt <= 0.0) {
        return fail("policy.step_dt must be a finite positive value");
    }
    if (!std::isfinite(config_.policy.target_interpolation_duration_s) ||
        config_.policy.target_interpolation_duration_s < 0.0) {
        return fail("policy.target_interpolation_duration_s must be finite and non-negative");
    }
    if constexpr (policy_observation::kEnableGaitPhase) {
        if (!std::isfinite(config_.policy.gait_phase_period) ||
            config_.policy.gait_phase_period <= 0.0) {
            return fail("gait_phase_period must be a finite positive value");
        }
    }

    for (std::size_t i = 0; i < PolicyRuntime::kDof; ++i) {
        if (config_.policy.action_clip[i][0] > config_.policy.action_clip[i][1]) {
            return fail("action_clip lower bound must be <= upper bound for every model DOF");
        }
        if (config_.policy.action_scale[i] <= 0.0) {
            return fail("action_scale must be > 0 for every model DOF");
        }
    }
    for (std::size_t i = 0; i < config_.ankle_motor_limits.min_rad.size(); ++i) {
        if (config_.ankle_motor_limits.min_rad[i] >
            config_.ankle_motor_limits.max_rad[i]) {
            return fail("ankle motor physical limit min must be <= max");
        }
    }

    return true;
}

bool RobotInterface::initialize_model_processors() {
    std::string error;
    auto mapping = robot_detail::JointMapping::create(config_.motor.num_motors,
                                                         config_.joint_mapping,
                                                                 error);
    if (!mapping) {
        std::cerr << "[RobotInterface] initialize_model_processors failed: "
                  << error << "\n";
        return false;
    }

    auto action_processor = std::make_unique<robot_detail::ActionProcessor>(
        mapping,
        config_.policy,
        config_.ankle_motor_limits,
        config_.motor.mit_kp,
        config_.motor.mit_kd,
        config_.ankle_torque);
    auto observation_builder = std::make_unique<robot_detail::ObservationBuilder>(mapping, config_.policy);

    joint_mapping_       = std::move(mapping);
    action_processor_    = std::move(action_processor);
    observation_builder_ = std::move(observation_builder);
    return true;
}

/* 加载 TorchScript 策略模型，并重置上一周期动作和观测历史。 */
bool RobotInterface::load_policy() {
    // 加载前先清掉旧策略与旧 recorder session，避免失败后残留旧运行状态。
    policy_runtime_.shutdown();
    inference_recorder_.stop();
    inference_recorder_failed_ = false;
    initialize_policy_runtime_state();

    if (!policy_runtime_.load(config_.policy)) {
        std::cerr << "[RobotInterface] load_policy failed: "
                  << policy_runtime_.last_error() << "\n";
        return false;
    }

    if (config_.recorder.enabled) {
        if (!inference_recorder_.start(config_.recorder)) {
            std::cerr << "[RobotInterface] failed to open inference log: "
                      << inference_recorder_.last_error() << "\n";
            inference_recorder_failed_ = true;
        } else {
            std::cout << "[RobotInterface] inference log: "
                      << inference_recorder_.log_path() << "\n";
        }
    }
    return true;
}


/* 保存策略使用的机器人目标速度指令，顺序为 [vx, vy, yaw_rate]。 */
void RobotInterface::set_target_velocity(double vx, double vy, double yaw_rate) {
    std::lock_guard<std::mutex> lock(target_velocity_mutex_);
    target_velocity_ = {vx, vy, yaw_rate};
}

/* 获取当前保存的机器人目标速度指令。 */
std::array<double, 3> RobotInterface::get_target_velocity() const {
    std::lock_guard<std::mutex> lock(target_velocity_mutex_);
    return target_velocity_;
}


/* 下发模型 DOF 顺序的目标关节角，内部完成限位、映射和方向转换。 */
bool RobotInterface::apply_action(const std::vector<double>& target_q_model_rad) {
    if (!motor_session_.is_initialized()) {
        return false;
    }
    if (!motor_session_.motion_enabled()) {
        std::cerr << "[RobotInterface] apply_action rejected: motors are stopped. "
                  << "Call initialize() first.\n";
        return false;
    }

    std::vector<double> target_rad;
    if (!action_processor_) {
        std::cerr << "[RobotInterface] apply_action rejected: model processors are not initialized. "
                  << "Call initialize() first.\n";
        return false;
    }

    std::string error;
    if (!action_processor_->build_motor_targets(target_q_model_rad,
                                                target_rad,
                                                error)) {
        std::cerr << "[RobotInterface] apply_action rejected: "
                  << error << "\n";
        return false;
    }
    return motor_session_.apply_targets_rad(target_rad);
}


/* 按平滑插值将关节恢复到 stand_pose_rad 初始姿态。 */
bool RobotInterface::reset_joints() {
    if (!motor_session_.is_initialized()) {
        return false;
    }
    if (!motor_session_.motion_enabled()) {
        std::cerr << "[RobotInterface] reset_joints rejected: motors are stopped. "
                  << "Call initialize() first.\n";
        return false;
    }
    if (!action_processor_) {
        std::cerr << "[RobotInterface] reset_joints rejected: model processors are not initialized. "
                  << "Call initialize() first.\n";
        return false;
    }

    const std::vector<double> target_model_q = config_.policy.stand_pose_rad;

    const std::vector<double> current_motor_q = motor_session_.get_joint_q();
    std::vector<double> start_model_q;
    std::string error;
    if (!action_processor_->build_reset_start_model_pose(current_motor_q,
                                                         target_model_q,
                                                         start_model_q,
                                                         error)) {
        std::cerr << "[RobotInterface] reset_joints rejected: "
                  << error << "\n";
        return false;
    }

    const int ramp_steps = 100;
    const auto dt = std::chrono::milliseconds(20);
    std::vector<double> target_rad;

    for (int k = 1; k <= ramp_steps; ++k) {
        const double alpha = static_cast<double>(k) / static_cast<double>(ramp_steps);
        std::vector<double> q_cmd_model(config_.motor.num_motors, 0.0);
        for (int i = 0; i < config_.motor.num_motors; ++i) {
            q_cmd_model[i] = start_model_q[i] * (1.0 - alpha) + target_model_q[i] * alpha;
        }
        if (!action_processor_->build_motor_targets(q_cmd_model, target_rad, error)) {
            std::cerr << "[RobotInterface] reset_joints failed: "
                      << error << "\n";
            return false;
        }
        if (!motor_session_.apply_targets_rad(target_rad)) {
            return false;
        }
        std::this_thread::sleep_for(dt);
    }

    return true;
}


/* 卸载策略模型并清空策略运行状态。 */
void RobotInterface::unload_policy() {
    policy_runtime_.shutdown();
    inference_recorder_.stop();
    inference_recorder_failed_ = false;
    initialize_policy_runtime_state();
}

void RobotInterface::initialize_policy_runtime_state() {
    policy_runtime_.reset();
    if (action_processor_) {
        action_processor_->reset_runtime_state();
    }
    if (observation_builder_) {
        observation_builder_->reset_runtime_state();
    }
}

void RobotInterface::record_inference(const InferenceRecord& record) {
    if (!config_.recorder.enabled  ||
        inference_recorder_failed_ ||
        !inference_recorder_.running()) {
        return;
    }

    if (!inference_recorder_.try_record(record)) {
        std::cerr << "[RobotInterface] failed to queue inference log: "
                  << inference_recorder_.last_error() << "\n";
        inference_recorder_failed_ = true;
    }
}

bool RobotInterface::start_policy_command_worker()
{
    if (policy_command_worker_running_.load()) {
        return true;
    }
    if (!action_processor_) {
        std::cerr << "[RobotInterface] policy command worker rejected: model processors are not initialized\n";
        return false;
    }
    if (!motor_session_.is_initialized() || !motor_session_.motion_enabled()) {
        std::cerr << "[RobotInterface] policy command worker rejected: motors are not running\n";
        return false;
    }

    PolicyTargetState initial_target;
    initial_target.sequence = ++latest_policy_target_sequence_;
    for (std::size_t i = 0; i < PolicyRuntime::kDof; ++i) {
        initial_target.q_model_rad[i] = config_.policy.stand_pose_rad[i];
    }
    policy_target_channel_.reset_with_value(initial_target);
    policy_command_log_channel_.reset_empty();
    policy_command_log_read_cache_ = PolicyCommandLogState{};
    {
        std::lock_guard<std::mutex> lock(policy_command_error_mutex_);
        policy_command_worker_error_.clear();
    }

    policy_command_worker_failed_.store(false);
    policy_command_worker_running_.store(true);
    try {
        policy_command_worker_thread_ =
            std::thread(&RobotInterface::policy_command_worker_loop, this);
    } catch (const std::exception& error) {
        policy_command_worker_running_.store(false);
        std::cerr << "[RobotInterface] failed to start policy command worker: "
                  << error.what() << "\n";
        return false;
    } catch (...) {
        policy_command_worker_running_.store(false);
        std::cerr << "[RobotInterface] failed to start policy command worker\n";
        return false;
    }

    return true;
}

void RobotInterface::stop_policy_command_worker()
{
    policy_command_worker_running_.store(false);
    if (policy_command_worker_thread_.joinable() &&
        policy_command_worker_thread_.get_id() != std::this_thread::get_id()) {
        policy_command_worker_thread_.join();
    }
}

void RobotInterface::policy_command_worker_loop()
{
    const auto period = std::chrono::nanoseconds(
        std::max<std::int64_t>(
            1,
            static_cast<std::int64_t>(
               std::llround(config_.ankle_torque.filter_dt_s * 1'000'000'000.0))));

    auto next_wake = std::chrono::steady_clock::now();
    robot_detail::FixedTargetInterpolator<PolicyRuntime::kDof> target_interpolator(
        config_.policy.target_interpolation_duration_s);
    std::uint64_t active_target_sequence = 0;

    PolicyTargetState target_state;
    if (!policy_target_channel_.try_consume_latest(target_state)) {
        fail_policy_command_worker("policy command target is not initialized");
        return;
    }
    target_interpolator.reset(target_state.q_model_rad);
    active_target_sequence = target_state.sequence;

    std::array<motor_base::MotorStatusSnapshot,
               motor_base::kMaxMotorCommandSetpoints> motor_feedback;
    bool has_motor_feedback = false;
    robot_detail::ActionProcessor::FixedPolicyMotorCommand command;
    std::string error;

    try {
        while (policy_command_worker_running_.load()) {
            next_wake += period;

            PolicyTargetState latest_target;
            const bool has_new_target =
                policy_target_channel_.try_consume_latest(latest_target);

            const auto loop_now = std::chrono::steady_clock::now();
            if (has_new_target &&
                latest_target.sequence != active_target_sequence) {
                target_interpolator.set_target(latest_target.q_model_rad, loop_now);
                active_target_sequence = latest_target.sequence;
            }
            const auto& smoothed_target_q_model_rad =
                target_interpolator.sample(loop_now);

            std::array<motor_base::MotorStatusSnapshot,
                       motor_base::kMaxMotorCommandSetpoints> latest_feedback;
            if (motor_session_.try_consume_realtime_feedback(latest_feedback)) {
                motor_feedback = latest_feedback;
                has_motor_feedback = true;
            }
            if (!has_motor_feedback) {
                fail_policy_command_worker("policy motor feedback is not initialized");
                break;
            }

            error.clear();
            if (!action_processor_->build_policy_impedance_command(
                    smoothed_target_q_model_rad,
                    motor_feedback,
                    command,
                    error)) {
                fail_policy_command_worker("failed to build policy impedance command: " + error);
                break;
            }

            const bool applied =
                motor_session_.apply_impedance_setpoints_realtime(
                    command.setpoints,
                    command.setpoint_count);

            PolicyCommandLogState log_state;
            log_state.timestamp_ns = steady_now_ns();
            for (std::size_t i = 0; i < PolicyRuntime::kDof; ++i) {
                log_state.target_pos_rad[i] = command.setpoints[i].position_rad;
                log_state.target_effort_permille[i] = command.target_effort_permille[i];
            }
            policy_command_log_channel_.publish(log_state);

            if (!applied) {
                fail_policy_command_worker("failed to apply policy impedance command");
                break;
            }

            std::this_thread::sleep_until(next_wake);
            const auto now = std::chrono::steady_clock::now();
            if (now > next_wake + period) {
                next_wake = now;
            }
        }
    } catch (const std::exception& error) {
        fail_policy_command_worker(
            std::string("policy command worker exception: ") + error.what());
    } catch (...) {
        fail_policy_command_worker("policy command worker exception");
    }
}

void RobotInterface::set_latest_policy_target(const std::vector<double>& target_q_model_rad)
{
    if (target_q_model_rad.size() != PolicyRuntime::kDof) {
        return;
    }

    PolicyTargetState target;
    target.sequence = ++latest_policy_target_sequence_;
    for (std::size_t i = 0; i < PolicyRuntime::kDof; ++i) {
        target.q_model_rad[i] = target_q_model_rad[i];
    }
    policy_target_channel_.publish(target);
}

RobotInterface::PolicyCommandLogState
RobotInterface::latest_policy_command_log_state()
{
    PolicyCommandLogState latest;
    if (policy_command_log_channel_.try_consume_latest(latest)) {
        policy_command_log_read_cache_ = latest;
    }
    return policy_command_log_read_cache_;
}

void RobotInterface::fail_policy_command_worker(std::string message)
{
    const std::string printable_message = message;
    const bool already_failed = policy_command_worker_failed_.exchange(true);
    policy_command_worker_running_.store(false);
    initialized_.store(false);
    {
        std::lock_guard<std::mutex> lock(policy_command_error_mutex_);
        policy_command_worker_error_ = std::move(message);
    }

    if (!already_failed) {
        std::cerr << "[RobotInterface] policy command worker failed: "
                  << printable_message << "\n";
        motor_session_.stop(-1);
    }
}

bool RobotInterface::policy_command_worker_healthy(std::string& error) const
{
    if (policy_command_worker_failed_.load()) {
        std::lock_guard<std::mutex> lock(policy_command_error_mutex_);
        error = policy_command_worker_error_.empty()
                    ? "policy command worker failed"
                    : policy_command_worker_error_;
        return false;
    }
    if (!policy_command_worker_running_.load()) {
        error = "policy command worker is not running";
        return false;
    }
    error.clear();
    return true;
}

/* 执行一次策略闭环：使用保存的目标速度构建观测、模型推理并下发目标关节角。 */
bool RobotInterface::policy_step() {
    const std::array<double, 3> target_velocity = get_target_velocity();

    std::string worker_error;
    if (!policy_command_worker_healthy(worker_error)) {
        return handle_policy_step_failure(worker_error);
    }
    if (!policy_runtime_.is_loaded()) {
        return handle_policy_step_failure("policy is not loaded");
    }
    if (!motor_session_.is_initialized()) {
        return handle_policy_step_failure("motors are not initialized");
    }

    const MotorStateSnapshot motor_state = motor_session_.get_motor_snapshot();
    const std::size_t motor_count = static_cast<std::size_t>(config_.motor.num_motors);
    const ImuStateSnapshot imu_state = imu_session_.get_state();
    
    const PolicyAction last_action   = policy_runtime_.last_action();


    // 构建日志观测信息 
    InferenceRecord record;
    record.frame_index = policy_runtime_.frame_index();
    fill_record_motor_state(motor_state, motor_count, record);

    // 单次策略闭环：同一份状态快照 -> 帧观测 -> 模型推理 -> 目标关节角 -> 电机下发。
    PolicyObservationTerms observation_terms;
    std::string observation_error;
    if (!observation_builder_->build(motor_state,
                                     imu_state,
                                     target_velocity,
                                     last_action,
                                     observation_terms,
                                     observation_error)) {
        return handle_policy_step_failure("failed to build policy observation: " +
                                          observation_error);
    }

    PolicyRuntimeStepResult policy_result;
    if (!policy_runtime_.infer(observation_terms, policy_result)) {
        return handle_policy_step_failure(policy_runtime_.last_error());
    }

    // 记录日志的策略推理时间和原始动作输出
    record.inference_start_ns = policy_result.inference_start_ns;
    record.inference_end_ns   = policy_result.inference_end_ns;
    record.raw_action         = policy_result.raw_action;

    std::vector<double> target_q_model_rad(PolicyRuntime::kDof, 0.0);
    for (std::size_t model_index = 0; model_index < PolicyRuntime::kDof; ++model_index) {

        // 对模型原始输出截断[-1, 1]
        const double clipped_raw_action =
            std::max(-config_.policy.raw_action_clip,
                     std::min(config_.policy.raw_action_clip,
                              static_cast<double>(policy_result.raw_action[model_index])));
        // 进行缩放
        const double scaled_action = clipped_raw_action * config_.policy.action_scale[model_index];
        // 进行截断
        const auto& action_clip = config_.policy.action_clip[model_index];
        const double clipped_action_offset =
            std::max(action_clip[0], std::min(action_clip[1], scaled_action));

        // 叠加模型顺序的站立姿态，得到模型顺序的目标关节角
        target_q_model_rad[model_index] = config_.policy.stand_pose_rad[model_index] + clipped_action_offset;
        record.target_q_model_rad[model_index] = target_q_model_rad[model_index];
    }

    // 把处理后的模型目标值传递给policy_target_channel_，调用publish函数
    set_latest_policy_target(target_q_model_rad);

    // 获取最新的策略命令日志状态，保存到日志记录中
    const PolicyCommandLogState command_log_state = latest_policy_command_log_state();
    for (std::size_t i = 0; i < PolicyRuntime::kDof; ++i) {
        record.target_pos_rad[i] = command_log_state.target_pos_rad[i];
        record.target_effort_permille[i] = command_log_state.target_effort_permille[i];
    }


    // 构建日志的观测信息
    record.command_applied =
        policy_command_worker_running_.load() &&
        !policy_command_worker_failed_.load();
    record.command_timestamp_ns =
        command_log_state.timestamp_ns != 0 ? command_log_state.timestamp_ns : steady_now_ns();
    record_inference(record);
    policy_runtime_.advance_frame();

    policy_runtime_.advance_episode();
    return true;
}

/* 处理策略执行失败：打印错误并停止全部电机。 */
bool RobotInterface::handle_policy_step_failure(const std::string& message) {
    std::cerr << "[RobotInterface] policy_step failed: " << message << "\n";
    // 策略链路任何一步失败都停机，避免继续执行上一周期的目标。
    initialized_.store(false);
    stop_policy_command_worker();
    motor_session_.stop(-1);
    return false;
}

}  // namespace inference
