#include "robot/robot_interface.hpp"
#include "tool/tool.hpp"
#include "robot/action_processor.hpp"
#include "robot/joint_mapping.hpp"
#include "robot/observation_builder.hpp"

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

namespace {

void fill_record_motor_state(const MotorStateSnapshot& motor_state,
                             std::size_t motor_count,
                             InferenceRecord& record)
{
    record.motor_sample_timestamp_ns = motor_state.timestamp_ns;

    for (std::size_t i = 0; i < motor_count; ++i) {
        record.rx_pos_rad[i]     = motor_state.position_rad[i];
        record.rx_vel_rad_s[i]   = motor_state.velocity_rad_s[i];
        record.torque_percent[i] = motor_state.torque_percent[i];
        record.comm_ok[i]        = motor_state.comm_ok[i];
        record.enabled[i]        = motor_state.enabled[i];
    }
    
}

}  // namespace

/* 保存外部传入的接口配置，后续由初始化函数按模块使用。 */
RobotInterface::RobotInterface(RobotInterfaceConfig config)
    : config_(std::move(config)),
      motor_session_(config_.motor, config_.safety),    // 构造 motor_session_
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

    if (!initialize_model_processors()) {
        shutdown();
        return false;
    }
    if (!motor_session_.initialize()) {
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

    if (!imu_session_.initialize()) {
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


/* 检查配置有效性的职责已移到配置加载器（load_deploy_config），
   在进程启动边界一次性完成。 */

bool RobotInterface::initialize_model_processors() {
    std::string error;
    auto mapping = robot_detail::JointMapping::create(static_cast<int>(policy_observation::kDof),
                                                         config_.joint_mapping,
                                                                 error);
    if (!mapping) {
        std::cerr << "[RobotInterface] initialize_model_processors failed: "
                  << error << "\n";
        return false;
    }

    auto action_processor = std::make_unique<robot_detail::ActionProcessor>(
        mapping,
        config_.action,
        config_.ankle_motor_limits,
        config_.motor.mit_kp,
        config_.motor.mit_kd,
        config_.ankle_torque);

    auto observation_builder = std::make_unique<robot_detail::ObservationBuilder>(
        mapping,
        config_.observation_scales,
        config_.action.default_joint_pos_rad,
        config_.policy);

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


/* 按平滑插值将关节恢复到 default_joint_pos_rad 初始姿态。 */
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

    const std::vector<double> target_model_q(
        config_.action.default_joint_pos_rad.begin(),
         config_.action.default_joint_pos_rad.end());

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
    const auto reset_period_us = std::max<std::int64_t>(
        1,
        static_cast<std::int64_t>(std::llround(
            config_.safety.control_command_timeout_ms * 500.0)));
    const auto dt = std::chrono::microseconds(reset_period_us);
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
        if (k < ramp_steps) {
            std::this_thread::sleep_for(dt);
        }
    }

    startup_hold_target_motor_rad_ = target_rad;
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

    next_policy_seq_ = 1;
    policy_target_channel_.reset_empty();
    policy_command_log_channel_.reset_empty();
    policy_command_log_read_cache_ = PolicyCommandLogState{};
    {
        std::lock_guard<std::mutex> lock(policy_command_error_mutex_);
        policy_command_worker_error_.clear();
    }

    // 在线程接管前刷新一次复位命令，为首次 worker 调度保留完整有效期。
    if (!motor_session_.apply_targets_rad(startup_hold_target_motor_rad_)) {
        std::cerr << "[RobotInterface] policy command worker rejected: "
                     "failed to refresh startup reset pose\n";
        return false;
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

    PolicyTargetFrame current_target;
    bool has_current_target = false;

    std::array<motor_base::MotorStatusSnapshot,
               motor_base::kMaxMotorCommandSetpoints> motor_feedback;
    bool has_motor_feedback = false;
    robot_detail::ActionProcessor::FixedPolicyMotorCommand command;
    std::string error;

    try {
        while (policy_command_worker_running_.load()) {
            next_wake += period;

            PolicyTargetFrame latest_target;
            if (policy_target_channel_.try_consume_latest(latest_target)) {
                current_target = latest_target;
                has_current_target = true;
            }

            if (!has_current_target) {
                // 首个正式策略帧到达前，持续刷新最后一次复位命令。
                if (!motor_session_.apply_targets_rad(
                        startup_hold_target_motor_rad_)) {
                    fail_policy_command_worker(
                        "failed to refresh startup reset pose");
                    break;
                }
                std::this_thread::sleep_until(next_wake);
                const auto now = std::chrono::steady_clock::now();
                if (now > next_wake + period) {
                    next_wake = now;
                }
                continue;
            }

            if (current_target.policy_seq == 0) {
                fail_policy_command_worker("policy target sequence is missing");
                break;
            }

            const std::int64_t now_ns = robot_base::monotonic_now_ns();
            if (current_target.valid_until_ns <= now_ns) {
                fail_policy_command_worker("policy target deadline expired");
                break;
            }

            std::array<motor_base::MotorStatusSnapshot,
                       motor_base::kMaxMotorCommandSetpoints> latest_feedback;
            if (motor_session_.try_consume_latest_status_command(latest_feedback)) {
                motor_feedback = latest_feedback;
                has_motor_feedback = true;
            }
            if (!has_motor_feedback) {
                fail_policy_command_worker("policy motor feedback is not initialized");
                break;
            }

            error.clear();
            if (!action_processor_->build_policy_impedance_command(
                    current_target.target_q_model_rad,
                    motor_feedback,
                    command,
                    error)) {
                fail_policy_command_worker("failed to build policy impedance command: " + error);
                break;
            }

            const std::int64_t produced_at_ns = robot_base::monotonic_now_ns();
            if (current_target.valid_until_ns <= produced_at_ns) {
                fail_policy_command_worker("policy target deadline expired");
                break;
            }
            const std::int64_t command_deadline = produced_at_ns +
                robot_base::seconds_to_ns(
                    config_.safety.control_command_timeout_ms / 1000.0);
            motor_base::CommandTiming timing;
            timing.source_policy_seq = current_target.policy_seq;
            timing.produced_at_ns = produced_at_ns;
            timing.valid_until_ns = std::min(current_target.valid_until_ns,
                                             command_deadline);

            const bool applied =
                motor_session_.apply_impedance_setpoints_realtime(
                    command.setpoints,
                    command.setpoint_count,
                    timing);

            PolicyCommandLogState log_state;
            log_state.timestamp_ns = produced_at_ns;
            log_state.policy_seq = current_target.policy_seq;
            log_state.observation_time_ns = current_target.observation_time_ns;
            log_state.policy_valid_until_ns = current_target.valid_until_ns;
            log_state.command_produced_at_ns = produced_at_ns;
            log_state.command_valid_until_ns = timing.valid_until_ns;
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

    AhrsStateSnapshot ahrs_state;
    if (!imu_session_.get_ahrs_snapshot(ahrs_state)) {
        return handle_policy_step_failure("AHRS data is not ready");
    }

    const std::int64_t timing_now_ns = robot_base::monotonic_now_ns();
    const std::int64_t imu_receive_timestamp_ns = ahrs_state.receive_timestamp_ns;

    if (motor_state.timestamp_ns == 0) {
        return handle_policy_step_failure(
            "motor timestamp missing: host_timestamp_ns invariant violated");
    }
    if (!ahrs_state.ahrs_ready) {
        return handle_policy_step_failure("AHRS data is not ready");
    }
    const bool has_imu_receive_timestamp = imu_receive_timestamp_ns != 0;

    const std::int64_t imu_age_ns =
        has_imu_receive_timestamp ? timing_now_ns - imu_receive_timestamp_ns : 0;

    const std::int64_t motor_age_ns = timing_now_ns - motor_state.timestamp_ns;

    const std::int64_t imu_motor_skew_ns =
        has_imu_receive_timestamp ? imu_receive_timestamp_ns - motor_state.timestamp_ns
                                  : 0;

    const std::int64_t max_imu_age_ns =
        robot_base::seconds_to_ns(config_.sensor_guard.max_imu_sample_age_s);
    const std::int64_t max_motor_age_ns =
        robot_base::seconds_to_ns(config_.sensor_guard.max_motor_sample_age_s);
    const std::int64_t max_skew_ns =
        robot_base::seconds_to_ns(config_.sensor_guard.max_sensor_state_skew_s);

    const bool compare_imu_timing = imu_age_ns != 0;
    
    if ((compare_imu_timing && imu_age_ns < 0) || motor_age_ns < 0) {
        return handle_policy_step_failure(
            "negative sensor age: imu_age_us=" + std::to_string(robot_base::ns_to_us(imu_age_ns)) +
            ", motor_age_us=" + std::to_string(robot_base::ns_to_us(motor_age_ns)));
    }
    if ((compare_imu_timing && imu_age_ns > max_imu_age_ns) ||
         motor_age_ns > max_motor_age_ns ||
        (compare_imu_timing && robot_base::abs_ns(imu_motor_skew_ns) > max_skew_ns)) {
        return handle_policy_step_failure(
            "sensor timing guard failed: imu_age_us=" +
            std::to_string(robot_base::ns_to_us(imu_age_ns)) +
            ", motor_age_us=" + std::to_string(robot_base::ns_to_us(motor_age_ns)) +
            ", imu_motor_skew_us=" + std::to_string(robot_base::ns_to_us(imu_motor_skew_ns)));
    }
    
    // 构建日志观测信息 
    InferenceRecord record;
    record.frame_index = observation_builder_->frame_index();
    fill_record_motor_state(motor_state, motor_count, record);
    record.imu_sample_timestamp_ns  = ahrs_state.sample_timestamp_ns;
    record.imu_receive_timestamp_ns = imu_receive_timestamp_ns;

    record.motor_age_us             = robot_base::ns_to_us(motor_age_ns);

    // 单次策略闭环：同一份状态快照 -> 帧观测 -> 模型推理 -> 目标关节角 -> 电机下发。
    PolicyObservation policy_observation;
    std::string observation_error;
    if (!observation_builder_->build(motor_state,
                                     ahrs_state,
                                     target_velocity,
                                     policy_observation,
                                     observation_error)) {
        return handle_policy_step_failure("failed to build policy observation: " +
                                          observation_error);
    }

    PolicyRuntimeStepResult policy_result;
    if (!policy_runtime_.infer(policy_observation, policy_result)) {
        return handle_policy_step_failure(policy_runtime_.last_error());
    }

    std::int64_t observation_time_ns = motor_state.timestamp_ns;
    if (has_imu_receive_timestamp) {
        observation_time_ns = std::min(observation_time_ns,
                                       imu_receive_timestamp_ns);
    }
    if (observation_time_ns <= 0) {
        observation_time_ns = timing_now_ns;
    }

    // 计算策略结果是否符合时效性，超过该时刻视为过期。
    const std::int64_t policy_valid_until_ns = observation_time_ns +
        robot_base::seconds_to_ns(
            config_.safety.policy_target_timeout_ms / 1000.0);
    if (robot_base::monotonic_now_ns() >= policy_valid_until_ns) {
        return handle_policy_step_failure(
            "policy inference result exceeded policy target deadline");
    }

    const std::uint64_t policy_seq = next_policy_seq_++;
    observation_builder_->commit_policy_action(policy_result.raw_action);

    // 构建控制命令帧
    PolicyTargetFrame policy_target;
    policy_target.policy_seq = policy_seq;
    policy_target.observation_time_ns = observation_time_ns;
    policy_target.valid_until_ns = policy_valid_until_ns;
    for (std::size_t model_index = 0; model_index < PolicyRuntime::kDof; ++model_index) {

        // 对模型原始输出截断[-1, 1]
        const double clipped_raw_action =
            std::max(-config_.action.raw_action_clip,
                     std::min(config_.action.raw_action_clip,
                              static_cast<double>(policy_result.raw_action[model_index])));
        // 进行缩放
        const double scaled_action = clipped_raw_action * config_.action.action_scale[model_index];
        // 进行截断
        const auto& action_clip = config_.action.action_clip[model_index];
        const double clipped_action_offset =
            std::max(action_clip[0], std::min(action_clip[1], scaled_action));

        // 叠加模型顺序的站立姿态，得到模型顺序的目标关节角
        policy_target.target_q_model_rad[model_index] =
            config_.action.default_joint_pos_rad[model_index] +
            clipped_action_offset;
        record.target_q_model_rad[model_index] =
            policy_target.target_q_model_rad[model_index];
    }

    // 发布到 SPSC 通道
    policy_target_channel_.publish(policy_target);


    // 记录日志的策略推理时间和原始动作输出
    record.inference_start_ns = policy_result.inference_start_ns;
    record.inference_end_ns   = policy_result.inference_end_ns;
    record.policy_seq = policy_seq;
    record.policy_observation_time_ns = observation_time_ns;
    record.policy_valid_until_ns      = policy_valid_until_ns;
    record.policy_observation = policy_observation;
    record.raw_action         = policy_result.raw_action;

    // 获取最新的策略命令日志状态，保存到日志记录中
    const PolicyCommandLogState command_log_state = latest_policy_command_log_state();
    for (std::size_t i = 0; i < PolicyRuntime::kDof; ++i) {
        record.target_pos_rad[i] = command_log_state.target_pos_rad[i];
        record.target_effort_permille[i] = command_log_state.target_effort_permille[i];
    }
    if (command_log_state.policy_seq != 0) {
        record.policy_seq = command_log_state.policy_seq;
        record.policy_observation_time_ns = command_log_state.observation_time_ns;
        record.policy_valid_until_ns = command_log_state.policy_valid_until_ns;
    }
    record.command_produced_at_ns = command_log_state.command_produced_at_ns;
    record.command_valid_until_ns = command_log_state.command_valid_until_ns;

    // 构建日志的观测信息
    record.command_applied =
         policy_command_worker_running_.load() &&
        !policy_command_worker_failed_.load();

    record.command_timestamp_ns =
        command_log_state.timestamp_ns != 0 ? command_log_state.timestamp_ns : steady_now_ns();

    record_inference(record);

    observation_builder_->advance_frame();

    observation_builder_->advance_episode();

    return true;
}

/* 处理策略执行失败：打印错误并停止生产者；保护动作由 RT 基类执行。 */
bool RobotInterface::handle_policy_step_failure(const std::string& message) {
    std::cerr << "[RobotInterface] policy_step failed: " << message << "\n";
    initialized_.store(false);
    stop_policy_command_worker();
    return false;
}

}  // namespace inference
