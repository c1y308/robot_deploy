#include "robot/robot_interface.hpp"
#include "kinematics/ankle_motor_fk.hpp"
#include "tool/tool.hpp"
#include "robot/action_processor.hpp"
#include "robot/joint_mapping.hpp"
#include "robot/observation_builder.hpp"
#include "policy/policy_execution_setup.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <sys/mman.h>


namespace inference {

/* 保存外部传入的接口配置，后续由初始化函数按模块使用。 */
RobotInterface::RobotInterface(RobotInterfaceConfig config)
    : config_(std::move(config)),
      motor_session_(config_),
      imu_session_(config_.imu, config_.runtime)
{
}

/* 析构仅尝试关闭一次；未获准释放时立即终止进程。 */
RobotInterface::~RobotInterface() noexcept {
    if (shutdown() == ShutdownResult::RetryRequired) {
        std::fputs("[RobotInterface] Fatal: destructor shutdown failed; "
                   "STOP not confirmed. Terminating process.\n", stderr);
        std::fflush(stderr);
        std::terminate();
    }
}

/* 编排机器人完整运行态：电机、策略、IMU 和站立姿态复位。 */
bool RobotInterface::initialize() {

    if (!initialize_processors()) {
        shutdown();
        return false;
    }
    // 模型 load/warmup、ORT worker 校验和内存锁定发生在电机、IMU 或
    // recorder 线程启动之前，保证新增 TID 可归因且失败时不会触碰电机。
    {
        PolicyExecutionSetup execution;
        std::string error;
        if (!execution.prepare(config_.runtime, error)) {
            std::cerr << "[RobotInterface] prepare_policy_execution failed: " << error << "\n";
            shutdown();
            return false;
        }
        if (!policy_runtime_.load(config_.policy, execution.session_options())) {
            std::cerr << "[RobotInterface] load_policy failed: "
                      << policy_runtime_.last_error() << "\n";
            shutdown();
            return false;
        }
        if (!execution.verify_workers(error)) {
            std::cerr << "[RobotInterface] verify_policy_workers failed: " << error << "\n";
            shutdown();
            return false;
        }
    }

    // 进程/主线程运行环境：模型就绪后、其他线程和硬件启动前完成。
    if (config_.runtime.require_process_memory_lock) {
        // 解决 MCL_FUTURE 在创建线程时一次性填充并锁定整个默认栈，
        // 拉长 EtherCAT 启动收发交接空档并导致失同步的问题。
        // 按需锁页，时序敏感线程仍在进入业务循环前预触碰所需栈页。
        if (::mlockall(MCL_CURRENT | MCL_FUTURE | MCL_ONFAULT) != 0) {
            const int saved_errno = errno;
            std::cerr << "[RobotInterface] mlockall(MCL_CURRENT | MCL_FUTURE | MCL_ONFAULT) "
                      << "failed: errno=" << saved_errno << " ("
                      << std::strerror(saved_errno)
                      << "). Check RLIMIT_MEMLOCK and CAP_IPC_LOCK.\n";
            shutdown();
            return false;
        }

        const int prefault_error =
            robot_base::prefault_current_thread_stack(
                config_.runtime.policy_main.stack_prefault_bytes);
        if (prefault_error != 0) {
            std::cerr << "[RobotInterface] policy_main stack prefault after "
                      << "mlockall failed: errno=" << prefault_error << " ("
                      << std::strerror(prefault_error)
                      << "); maximum 1 MiB and valid page size required\n";
            shutdown();
            return false;
        }
    }

    // 日志生命周期：按已校验的观测维度启动独立记录器。
    if (config_.recorder.enabled) {
        const auto observation_size =
            policy_observation::observation_size(config_.policy.gait.enabled);
        if (!inference_recorder_.start(config_.recorder, observation_size,
                                       config_.runtime.background)) {
            std::cerr << "[RobotInterface] failed to open inference log: "
                      << inference_recorder_.last_error() << "\n";
            inference_recorder_failed_ = true;
            shutdown();
            return false;
        }
        std::cout << "[RobotInterface] inference log: "
                  << inference_recorder_.log_path() << "\n";
    }

    // 初始化电机
    if (!motor_session_.initialize()) {
        shutdown();
        return false;
    }
    if (!motor_session_.restart()) {
        shutdown();
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 初始化 IMU
    if (!imu_session_.initialize()) {
        shutdown();
        return false;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 恢复到初始位置
    std::array<double, motor_base::kMotorNum> startup_hold_target_motor_rad{};
    if (!reset_joints(startup_hold_target_motor_rad)) {
        shutdown();
        return false;
    }

    // 处理器和复位姿态就绪后，绑定 worker 的固定依赖。
    try {
        policy_command_worker_ = std::make_unique<PolicyCommandWorker>(
            config_, motor_session_, *action_processor_, first_policy_inference_started_ns_);
    } catch (const std::exception& error) {
        std::cerr << "[RobotInterface] failed to construct policy command worker: "
                  << error.what() << "\n";
        shutdown();
        return false;
    }

    // 开启策略命令线程
    if (!policy_command_worker_->start(std::move(startup_hold_target_motor_rad))) {
        std::cerr << "[RobotInterface] failed to start policy command worker\n";
        shutdown();
        return false;
    }

    initialized_ = true;
    return true;
}


/* 先请求并确认停止；仅在持续通信故障时允许未确认释放。 */
ShutdownResult RobotInterface::shutdown() {
    const auto request = motor_session_.request_stop();
    initialized_ = false;
    // 会话释放控制器前，先结束仍可能访问控制器的命令线程。
    if (policy_command_worker_) {
        policy_command_worker_->stop();
    }
    const auto result = motor_session_.shutdown(request);
    if (result == ShutdownResult::RetryRequired) {
        return result;
    }
    if (policy_command_worker_) {
        record_completed_policy_frame(*policy_command_worker_);
        policy_command_worker_.reset();
    }
    policy_runtime_.shutdown();
    inference_recorder_.stop();
    inference_recorder_failed_ = false;
    policy_step_phase_ = PolicyStepPhase::Idle;
    policy_step_phase_started_ns_ = 0;
    last_policy_target_published_ns_ = 0;
    first_policy_inference_started_ns_.store(0, std::memory_order_release);
    imu_session_.deinitialize();
    return result;
}


bool RobotInterface::initialize_processors() {
    std::unique_ptr<const JointMapping> mapping;
    try {
        mapping = std::make_unique<const JointMapping>(
            static_cast<int>(kDof), config_.joint_mapping);
    } catch (const std::invalid_argument& error) {
        std::cerr << "[RobotInterface] initialize_processors failed: "
                  << error.what() << "\n";
        return false;
    }

    auto action_processor = std::make_unique<ActionProcessor>(config_, *mapping);
    auto observation_builder = std::make_unique<ObservationBuilder>(config_, *mapping);

    // 先替换旧处理器，再释放它们引用的旧映射。
    action_processor_    = std::move(action_processor);
    observation_builder_ = std::move(observation_builder);
    joint_mapping_       = std::move(mapping);

    return true;
}

/* 按 YAML 中配置的范围限制并保存目标速度，顺序为 [vx, vy, yaw_rate]。 */
void RobotInterface::set_target_velocity(double vx, double vy, double yaw_rate) {
    if (config_.command_ranges.lin_vel_x) {
        const auto& range = *config_.command_ranges.lin_vel_x;
        vx = std::clamp(vx, range[0], range[1]);
    }
    if (config_.command_ranges.lin_vel_y) {
        const auto& range = *config_.command_ranges.lin_vel_y;
        vy = std::clamp(vy, range[0], range[1]);
    }
    if (config_.command_ranges.ang_vel_z) {
        const auto& range = *config_.command_ranges.ang_vel_z;
        yaw_rate = std::clamp(yaw_rate, range[0], range[1]);
    }
    target_velocity_ = {vx, vy, yaw_rate};
}

bool RobotInterface::get_start_model_pose(
    const std::array<double, motor_base::kMotorNum>& current_motor_q, // 当前电机真实角度(rad)
    std::array<double, kDof>& start_model_q,   // 输出: 当前的模型关节角(rad)
    std::string& error) const
{
    const int count = joint_mapping_->dof_count();
    start_model_q.fill(0.0);

    // 按照模型顺序遍历，将电机角度转换为模型关节角度
    for (int model_index = 0; model_index < count; ++model_index) {
        // 跳过脚踝关节的模型 DOF，它们的角度由 FK 求解器计算得出
        if (joint_mapping_->is_model_dof(model_index)) {
            continue;
        }

        // 得到电机索引
        const int motor_index = joint_mapping_->get_motor_id(model_index);
        // 从电机角度转换为模型关节角度，考虑电机方向
        start_model_q[static_cast<std::size_t>(model_index)] =
            joint_mapping_->get_motor_direction(motor_index) * current_motor_q[static_cast<std::size_t>(motor_index)];
    }

    auto fill_ankle_start_model_q = [&](const char* ankle_name,
                                       const AnkleMap& ankle_map) {
        const double upper_motor =
            joint_mapping_->get_motor_direction(ankle_map.upper_motor_index) *
            current_motor_q[static_cast<std::size_t>(ankle_map.upper_motor_index)];
        const double lower_motor =
            joint_mapping_->get_motor_direction(ankle_map.lower_motor_index) *
            current_motor_q[static_cast<std::size_t>(ankle_map.lower_motor_index)];

        const auto pitch_index = static_cast<std::size_t>(ankle_map.model_pitch_dof);
        const auto roll_index  = static_cast<std::size_t>(ankle_map.model_roll_dof);
        const double initial_pitch = config_.default_joint_pos_rad[pitch_index];
        const double initial_roll  = config_.default_joint_pos_rad[roll_index];
        const ankle_motor_fk::FootAngles foot =
            ankle_motor_fk::solve(upper_motor, lower_motor, initial_roll, initial_pitch);

        if (!foot.reachable ||
            !std::isfinite(foot.pitch) || !std::isfinite(foot.roll)) {
            // 电机角已转换到求解器方向；失败时不能用目标姿态替代真实起点。
            error = std::string(ankle_name) +
                    " ankle FK failed while building reset start pose: upper_motor=" +
                    std::to_string(upper_motor) + " rad, lower_motor=" +
                    std::to_string(lower_motor) + " rad, reachable=" +
                    (foot.reachable ? "true" : "false") +
                    ", pitch=" + std::to_string(foot.pitch) +
                    " rad, roll=" + std::to_string(foot.roll) + " rad";
            return false;
        }
        start_model_q[pitch_index] = foot.pitch;
        start_model_q[roll_index] = foot.roll;
        return true;
    };

    if (!fill_ankle_start_model_q("left", joint_mapping_->left_ankle()) ||
        !fill_ankle_start_model_q("right", joint_mapping_->right_ankle())) {
        return false;
    }
    error.clear();
    return true;
}

/* 按平滑插值将关节恢复到 default_joint_pos_rad 初始姿态。 */
bool RobotInterface::reset_joints(
    std::array<double, motor_base::kMotorNum>& final_target_motor_rad) {
    const auto& target_model_q = config_.default_joint_pos_rad;
    std::string error;
    if (!motor_session_.refresh_feedback_for_policy(error)) {
        std::cerr << "[RobotInterface] reset_joints rejected: " << error << "\n";
        return false;
    }
    const MotorStateSnapshot motor_state = motor_session_.get_feedback_for_policy();
    if (motor_state.timestamp_ns <= 0 || !robot_base::finite_array(motor_state.position_rad)) {
        std::cerr << "[RobotInterface] reset_joints rejected: motor feedback is invalid.\n";
        return false;
    }
    std::array<double, kDof> start_model_q{};
    if (!get_start_model_pose(motor_state.position_rad, start_model_q, error)) {
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
    std::array<double, motor_base::kMotorNum> target_rad{};
    std::array<double, kDof> q_cmd_model{};

    for (int k = 1; k <= ramp_steps; ++k) {
        const double alpha = static_cast<double>(k) / static_cast<double>(ramp_steps);
        for (std::size_t i = 0; i < q_cmd_model.size(); ++i) {
            q_cmd_model[i] = start_model_q[i] * (1.0 - alpha) + target_model_q[i] * alpha;
        }
        if (!action_processor_->build_motor_position_targets(q_cmd_model, target_rad, error)) {
            std::cerr << "[RobotInterface] reset_joints failed: "
                      << error << "\n";
            return false;
        }
        motor_base::CommandTiming timing;
        timing.produced_at_ns = robot_base::monotonic_now_ns();
        timing.valid_until_ns = timing.produced_at_ns + robot_base::seconds_to_ns(
            config_.safety.control_command_timeout_ms / 1000.0);
        bool submitted = false;
        if (config_.motor.control_mode == motor_base::MotorControlMode::POSITION) {
            submitted = motor_session_.submit_position_targets_rad(target_rad, timing);
        } else if (config_.motor.control_mode == motor_base::MotorControlMode::IMPEDANCE) {
            const auto setpoints = ActionProcessor::make_position_impedance_setpoints(
                target_rad, config_.motor);
            submitted = motor_session_.submit_impedance_setpoints(setpoints, timing);
        }
        if (!submitted) {
            return false;
        }
        if (k < ramp_steps) {
            std::this_thread::sleep_for(dt);
        }
    }

    final_target_motor_rad = std::move(target_rad);
    return true;
}


// 独立 latest-value 数据源只检查各自的时间戳和年龄。
bool RobotInterface::validate_policy_sensor_timing(
    std::int64_t motor_timestamp_ns,
    std::int64_t imu_timestamp_ns,
    std::int64_t now_ns,
    std::string& error) const
{
    error.clear();
    if (motor_timestamp_ns <= 0) {
        error = "motor timestamp missing: host_timestamp_ns invariant violated";
        return false;
    }
    if (imu_timestamp_ns <= 0) {
        error = "IMU timestamp missing: host_timestamp_ns invariant violated";
        return false;
    }

    const std::int64_t imu_age_ns   = now_ns - imu_timestamp_ns;
    const std::int64_t motor_age_ns = now_ns - motor_timestamp_ns;

    if (imu_age_ns   > robot_base::seconds_to_ns(config_.sensor_guard.max_imu_sample_age_s) ||
        motor_age_ns > robot_base::seconds_to_ns(config_.sensor_guard.max_motor_sample_age_s)) {
        error = "sensor timing guard failed: imu_age_us=" +
            std::to_string(robot_base::ns_to_us(imu_age_ns)) +
            ", motor_age_us=" + std::to_string(robot_base::ns_to_us(motor_age_ns));
        return false;
    }
    return true;
}


RobotInterface::PolicyResultAdmission RobotInterface::admit_policy_result(
    std::int64_t observation_time_ns,
    std::int64_t decision_now_ns) const noexcept
{
    const std::int64_t obs_age_ns      = decision_now_ns - observation_time_ns;
    const std::int64_t hold_started_ns = last_policy_target_published_ns_ != 0
        ? last_policy_target_published_ns_
        : first_policy_inference_started_ns_.load(std::memory_order_acquire);

    PolicyResultAdmission admission;

    admission.obs_to_action_age_us = robot_base::ns_to_us(obs_age_ns);
    admission.target_hold_age_us   = robot_base::ns_to_us(decision_now_ns - hold_started_ns);
    admission.dropped = obs_age_ns > kMaxObsToActionAgeNs;
    return admission;
}


void RobotInterface::record_completed_policy_frame(PolicyCommandWorker& worker)
{
    if (!config_.recorder.enabled) {
        return;
    }
    InferenceRecord record;
    if (worker.try_consume_completed_record(record)) {
        record_policy_frame(record);
    }
}


void RobotInterface::record_policy_frame(const InferenceRecord& record) {
    if (!config_.recorder.enabled  || inference_recorder_failed_) {
        return;
    }

    if (!inference_recorder_.try_record(record)) {
        std::cerr << "[RobotInterface] failed to queue inference log: "
                  << inference_recorder_.last_error() << "\n";
        inference_recorder_failed_ = true;
    }
}


void RobotInterface::set_policy_step_phase(PolicyStepPhase phase) noexcept
{
    policy_step_phase_started_ns_ = robot_base::monotonic_now_ns();
    policy_step_phase_ = phase;
}


/* 执行一次策略闭环：使用保存的目标速度构建观测、模型推理并下发目标关节角。 */
bool RobotInterface::policy_step() {

    set_policy_step_phase(PolicyStepPhase::Precheck);

    if (!policy_command_worker_) {
        return handle_policy_step_failure("policy command worker is not initialized");
    }
    auto& worker = *policy_command_worker_;

    const std::array<double, 3> target_velocity = target_velocity_;

    std::string worker_error;

    if (!worker.check_health(worker_error)) {
        return handle_policy_step_failure(worker_error);
    }
    if (!policy_runtime_.is_loaded()) {
        return handle_policy_step_failure("policy is not loaded");
    }
    if (!motor_session_.is_initialized()) {
        return handle_policy_step_failure("motors are not initialized");
    }

    set_policy_step_phase(PolicyStepPhase::SensorSnapshot);
    if (!motor_session_.refresh_feedback_for_policy(worker_error)) {
        return handle_policy_step_failure(worker_error);
    }
    const MotorStateSnapshot motor_state = motor_session_.get_feedback_for_policy();

    AhrsStateSnapshot ahrs_state;
    if (!imu_session_.get_ahrs_snapshot(ahrs_state)) {
        return handle_policy_step_failure("AHRS data is not ready");
    }

    const std::int64_t timing_now_ns = robot_base::monotonic_now_ns();
    const std::int64_t imu_receive_timestamp_ns = ahrs_state.receive_timestamp_ns;

    if (!validate_policy_sensor_timing(motor_state.timestamp_ns,
                                         imu_receive_timestamp_ns,
                                       timing_now_ns, worker_error)) {
        return handle_policy_step_failure(worker_error);
    }
    const std::int64_t observation_time_ns =
        std::min(motor_state.timestamp_ns, imu_receive_timestamp_ns);

    // 单次策略闭环：同一份状态快照 -> 帧观测 -> 模型推理 -> 目标关节角 -> 电机下发。
    set_policy_step_phase(PolicyStepPhase::Observation);
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

    set_policy_step_phase(PolicyStepPhase::Inference);
    const std::int64_t inference_started_ns = robot_base::monotonic_now_ns();
    if (first_policy_inference_started_ns_.load(std::memory_order_relaxed) == 0) {
        first_policy_inference_started_ns_.store(inference_started_ns, std::memory_order_release);
    }
    const std::uint64_t policy_seq = observation_builder_->frame_index() + 1;
    PolicyRuntimeStepResult policy_result;
    if (!policy_runtime_.infer(policy_observation, policy_result)) {
        return handle_policy_step_failure(policy_runtime_.last_error());
    }

    set_policy_step_phase(PolicyStepPhase::PostInference);
    if (!worker.check_health(worker_error)) {
        return handle_policy_step_failure(worker_error);
    }

    // admission 之前只完成控制目标计算，不组装日志记录。
    const auto target_model_rad =
        action_processor_->make_model_target(policy_result.raw_action);

    set_policy_step_phase(PolicyStepPhase::Publish);
    const std::int64_t decision_now_ns = robot_base::monotonic_now_ns();
    const PolicyResultAdmission admission =
        admit_policy_result(observation_time_ns, decision_now_ns);

    // 两项年龄固定在 decision_now；仅启用 recorder 时填写非控制日志。
    const auto frame_index = observation_builder_->frame_index();
    const auto fill_log_record = [&](InferenceRecord& record) {
        record.frame_index = frame_index;
        record.motor_sample_timestamp_ns = motor_state.timestamp_ns;
        record.rx_pos_rad     = motor_state.position_rad;
        record.rx_vel_rad_s   = motor_state.velocity_rad_s;
        record.torque_percent = motor_state.torque_percent;
        record.comm_ok        = motor_state.comm_ok;
        record.enabled        = motor_state.enabled;
        record.imu_sample_timestamp_ns = ahrs_state.sample_timestamp_ns;
        record.imu_receive_timestamp_ns = imu_receive_timestamp_ns;
        record.motor_age_us = robot_base::ns_to_us(timing_now_ns - motor_state.timestamp_ns);
        record.inference_start_ns = policy_result.inference_start_ns;
        record.inference_end_ns   = policy_result.inference_end_ns;
        record.policy_seq = policy_seq;
        record.policy_observation_time_ns = observation_time_ns;
        record.policy_observation = policy_observation;
        record.raw_action = policy_result.raw_action;
        record.target_model_rad = target_model_rad;
        record.obs_to_action_age_us = admission.obs_to_action_age_us;
        record.target_hold_age_us = admission.target_hold_age_us;
        record.policy_result_dropped = admission.dropped;
    };

    // 丢弃的结果不发布，不刷新发布时间。
    if (!admission.dropped) {
        PolicyTargetFrame target;
        target.policy_seq = policy_seq;
        target.target_model_rad = target_model_rad;
        // worker 的目标超时诊断也使用该年龄。
        target.obs_to_action_age_us = admission.obs_to_action_age_us;
        if (config_.recorder.enabled) {
            InferenceRecord record;
            fill_log_record(record);
            last_policy_target_published_ns_ = worker.publish_target(target, &record);
        } else {
            last_policy_target_published_ns_ = worker.publish_target(target);
        }
        observation_builder_->set_last_action(policy_result.raw_action);
    }

    // 推进观测帧计数；丢弃结果也会推进。
    observation_builder_->advance_frame();

    // 尝试收取 worker 完成的记录。
    record_completed_policy_frame(worker);

    if (admission.dropped && config_.recorder.enabled) {
        InferenceRecord record;
        fill_log_record(record);
        record_policy_frame(record);
    }
    set_policy_step_phase(PolicyStepPhase::Idle);
    return true;
}

/* 故障入口先请求停止，再进行可能阻塞的诊断及 join。 */
bool RobotInterface::handle_policy_step_failure(const std::string& message) {

    motor_session_.request_stop();

    initialized_ = false;

    if (policy_command_worker_) {
        policy_command_worker_->request_stop();
    }

    // 这里记录 Interface 处理故障时的阶段，worker 的停止请求已经发出。
    const std::int64_t diagnostic_now_ns = robot_base::monotonic_now_ns();
    constexpr const char* phase_names[] = {
        "idle", "precheck", "sensor_snapshot", "observation",
        "inference", "post_inference", "publish"};
    std::cerr << "[RobotInterface] policy_step failed: " << message
              << ", policy_step_phase=" << phase_names[static_cast<std::size_t>(policy_step_phase_)]
              << ", phase_elapsed_us="
              << robot_base::ns_to_us(diagnostic_now_ns - policy_step_phase_started_ns_) << "\n";
    
    if (policy_command_worker_) {
        policy_command_worker_->stop();
        record_completed_policy_frame(*policy_command_worker_);
    }

    set_policy_step_phase(PolicyStepPhase::Idle);
    return false;
}

}  // namespace inference
