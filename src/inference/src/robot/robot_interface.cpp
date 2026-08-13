#include "robot/robot_interface.hpp"
#include "robot/action_processor.hpp"
#include "robot/joint_mapping.hpp"
#include "robot/observation_builder.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>


namespace inference {
namespace {

const std::filesystem::path& default_inference_log_dir()
{
    static const std::filesystem::path path{ROBOT_INFERENCE_LOG_DIR};
    return path;
}

/* 检查 3 维数组中的数值是否全部为有限值。 */
bool finite_array3(const std::array<double, 3>& values)
{
    return std::isfinite(values[0]) &&
           std::isfinite(values[1]) &&
           std::isfinite(values[2]);
}

/* 检查动态数组中的数值是否全部为有限值。 */
bool finite_vector(const std::vector<double>& values)
{
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    });
}

/* 检查 action 截断范围中的上下界是否全部为有限值。 */
bool finite_action_clip_ranges(const std::vector<std::array<double, 2>>& ranges)
{
    return std::all_of(ranges.begin(), ranges.end(), [](const auto& range) {
        return std::isfinite(range[0]) && std::isfinite(range[1]);
    });
}

bool check_motor_snapshot_size(const MotorStateSnapshot& motor_state,
                               std::size_t motor_count)
{
    if (motor_state.position_rad.size() != motor_count ||
        motor_state.velocity_rad_s.size() != motor_count ||
        motor_state.torque_percent.size() != motor_count ||
        motor_state.comm_ok.size() != motor_count ||
        motor_state.enabled.size() != motor_count ||
        motor_state.faulted.size() != motor_count) {
        return false;
    }

    return true;
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
        record.faulted[i] = motor_state.faulted[i];
    }
}

}  // namespace

/* 保存外部传入的接口配置，后续由初始化函数按模块使用。 */
RobotInterface::RobotInterface(RobotInterfaceConfig config)
    : config_(std::move(config)),
      motor_session_(config_.motor),    // 构造 motor_session_
      imu_session_(config_.imu)         // 构造 imu_session_
{
        if (config_.recorder.directory == InferenceRecorderConfig{}.directory) {
            config_.recorder.directory = default_inference_log_dir();
        }
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
    return true;
}


/* 幂等停机：先卸载策略和日志，再释放 IMU，最后释放电机。 */
void RobotInterface::shutdown() {
    initialized_.store(false);
    unload_policy();
    imu_session_.deinitialize();
    imu_session_.deinitialize();
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
    if (config_.policy.joint_min_rad.size() != PolicyRuntime::kDof ||
        config_.policy.joint_max_rad.size() != PolicyRuntime::kDof) {
        return fail("joint_min_rad and joint_max_rad must have 12 values");
    }
    if (config_.policy.dof_pos_scale.size() != PolicyRuntime::kDof ||
        config_.policy.dof_vel_scale.size() != PolicyRuntime::kDof) {
        return fail("dof_pos_scale and dof_vel_scale must have 12 values");
    }

    if (!finite_vector(config_.policy.stand_pose_rad) ||
        !finite_action_clip_ranges(config_.policy.action_clip) ||
        !finite_vector(config_.policy.action_scale) ||
        !finite_vector(config_.policy.joint_min_rad) ||
        !finite_vector(config_.policy.joint_max_rad) ||
        !finite_vector(config_.policy.dof_pos_scale) ||
        !finite_vector(config_.policy.dof_vel_scale)) {
        return fail("all vector policy values must be finite");
    }
    if (!finite_array3(config_.policy.command_scale) ||
        !finite_array3(config_.policy.body_ang_vel_scale)) {
        return fail("command/body_ang_vel scales must be finite");
    }
    if (!std::isfinite(config_.policy.step_dt) ||
        config_.policy.step_dt <= 0.0) {
        return fail("policy.step_dt must be a finite positive value");
    }
    if constexpr (policy_observation::kEnableGaitPhase) {
        if (!std::isfinite(config_.policy.gait_phase_period) ||
            config_.policy.gait_phase_period <= 0.0) {
            return fail("gait_phase_period must be a finite positive value");
        }
    }
    std::string joint_mapping_error;
    if (!robot_detail::JointMapping::validate(config_.motor.num_motors,
                                              config_.joint_mapping,
                                              joint_mapping_error)) {
        return fail(joint_mapping_error);
    }

    for (std::size_t i = 0; i < PolicyRuntime::kDof; ++i) {
        if (config_.policy.action_clip[i][0] > config_.policy.action_clip[i][1]) {
            return fail("action_clip lower bound must be <= upper bound for every model DOF");
        }
        if (config_.policy.action_scale[i] <= 0.0) {
            return fail("action_scale must be > 0 for every model DOF");
        }
        if (config_.policy.joint_min_rad[i] > config_.policy.joint_max_rad[i]) {
            return fail("joint_min_rad must be <= joint_max_rad for every model DOF");
        }
        if (config_.policy.joint_min_rad[i] > 0.0 || config_.policy.joint_max_rad[i] < 0.0) {
            return fail("relative joint limits must include 0 for every model DOF");
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

    auto action_processor    = std::make_unique<robot_detail::ActionProcessor>(mapping, config_.policy);
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

    if (!validate_policy_config()) {
        return false;
    }

    if (!policy_runtime_.load(config_.policy)) {
        std::cerr << "[RobotInterface] load_policy failed: "
                  << policy_runtime_.last_error() << "\n";
        return false;
    }

    if (config_.recorder.enabled) {
        InferenceRecorderConfig recorder_config = config_.recorder;
        if (recorder_config.directory == InferenceRecorderConfig{}.directory) {
            recorder_config.directory = default_inference_log_dir();
        }

        if (!inference_recorder_.start(std::move(recorder_config))) {
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

    std::vector<double> target_model(config_.motor.num_motors, 0.0);
    if (static_cast<int>(config_.policy.stand_pose_rad.size()) == config_.motor.num_motors) {
        target_model = config_.policy.stand_pose_rad;
    }

    const std::vector<double> q0_motor = motor_session_.get_joint_q();
    std::vector<double> q0_model;
    std::string error;
    if (!action_processor_->build_reset_start_model_pose(q0_motor,
                                                         target_model,
                                                         q0_model,
                                                         error)) {
        std::cerr << "[RobotInterface] reset_joints rejected: "
                  << error << "\n";
        return false;
    }

    const int ramp_steps = 100;
    const auto dt = std::chrono::milliseconds(20);

    for (int k = 1; k <= ramp_steps; ++k) {
        const double alpha = static_cast<double>(k) / static_cast<double>(ramp_steps);
        std::vector<double> q_cmd_model(config_.motor.num_motors, 0.0);
        for (int i = 0; i < config_.motor.num_motors; ++i) {
            q_cmd_model[i] = q0_model[i] * (1.0 - alpha) + target_model[i] * alpha;
        }
        if (!apply_action(q_cmd_model)) {
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

/* 执行一次策略闭环：使用保存的目标速度构建观测、模型推理并下发目标关节角。 */
bool RobotInterface::policy_step() {
    const std::array<double, 3> target_velocity = get_target_velocity();

    if (!policy_runtime_.is_loaded()) {
        return handle_policy_step_failure("policy is not loaded");
    }
    if (!motor_session_.is_initialized()) {
        return handle_policy_step_failure("motors are not initialized");
    }

    const MotorStateSnapshot motor_state = motor_session_.get_motor_state();
    const std::size_t motor_count = static_cast<std::size_t>(config_.motor.num_motors);
    if (!check_motor_snapshot_size(motor_state, motor_count)) {
        return handle_policy_step_failure(
            "failed to read motor state snapshot: motor state size mismatch");
    }
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

        // 模型输出先按训练约定缩放，再截断缩放后的动作偏移，最后叠加模型顺序的站立姿态。
        const auto& action_clip = config_.policy.action_clip[model_index];
        const double scaled_action =
            static_cast<double>(policy_result.raw_action[model_index]) *
            config_.policy.action_scale[model_index];
        const double clipped_action_offset =
            std::max(action_clip[0],
                     std::min(action_clip[1], scaled_action));

        target_q_model_rad[model_index] = config_.policy.stand_pose_rad[model_index] + clipped_action_offset;
        record.target_q_model_rad[model_index] = target_q_model_rad[model_index];
    }

    std::vector<double> target_rad;
    std::string action_error;
    if (!action_processor_->build_motor_targets(target_q_model_rad,
                                                target_rad,
                                                action_error)) {
        return handle_policy_step_failure("failed to build action target rad: " +
                                          action_error);
    }
    for (std::size_t i = 0; i < PolicyRuntime::kDof; ++i) {
        record.target_pos_rad[i] = target_rad[i];
    }


    // 构建日志的观测信息
    record.command_applied      = motor_session_.apply_targets_rad(target_rad);
    record.command_timestamp_ns = steady_now_ns();
    record_inference(record);
    policy_runtime_.advance_frame();

    if (!record.command_applied) {
        return handle_policy_step_failure("failed to apply policy target action");
    }

    policy_runtime_.advance_episode();
    return true;
}

/* 处理策略执行失败：打印错误并停止全部电机。 */
bool RobotInterface::handle_policy_step_failure(const std::string& message) {
    std::cerr << "[RobotInterface] policy_step failed: " << message << "\n";
    // 策略链路任何一步失败都停机，避免继续执行上一周期的目标。
    initialized_.store(false);
    motor_session_.stop(-1);
    return false;
}

}  // namespace inference
