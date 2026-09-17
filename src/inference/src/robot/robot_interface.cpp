#include "robot/robot_interface.hpp"
#include "tool/tool.hpp"
#include "robot/action_processor.hpp"
#include "robot/joint_mapping.hpp"
#include "robot/observation_builder.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <sys/mman.h>


namespace inference {

/* 保存外部传入的接口配置，后续由初始化函数按模块使用。 */
RobotInterface::RobotInterface(RobotInterfaceConfig config)
    : config_(std::move(config)),
      motor_session_(config_.motor, config_.safety, config_.runtime),
      imu_session_(config_.imu, config_.runtime),
      worker_(config_, motor_session_, first_policy_inference_started_ns_)
{
}

/* 析构时释放策略、IMU 和电机资源，保证后台线程退出。 */
RobotInterface::~RobotInterface() {
    while (shutdown() == ShutdownResult::RetryRequired) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

/* 编排机器人完整运行态：电机、策略、IMU 和站立姿态复位。 */
bool RobotInterface::initialize() {

    // 如果已经初始化，直接返回成功。
    if (is_initialized()) {
        return true;
    }

    // 如果上次初始化失败，先尝试释放资源。
    if (shutdown() != ShutdownResult::Confirmed) return false;

    if (!initialize_model_processors()) {
        shutdown();
        return false;
    }
    // 模型 load/dry-run、Torch worker 校验和内存锁定必须发生在任何硬件或
    // recorder 线程启动之前，保证新增 TID 可归因且失败时不会触碰硬件。
    if (!load_policy()) {
        shutdown();
        return false;
    }

    // 初始化电机
    if (!motor_session_.initialize(/*defer_communication_protection=*/true)) {
        shutdown();
        return false;
    }
    if (!motor_session_.restart(-1)) {
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
    std::vector<double> startup_hold_target_motor_rad;
    if (!reset_joints(startup_hold_target_motor_rad)) {
        shutdown();
        return false;
    }

    // 初始化完成
    initialized_.store(true);

    // 开启策略命令线程
    if (!worker_.start(*action_processor_, std::move(startup_hold_target_motor_rad))) {
        initialized_.store(false);
        shutdown();
        return false;
    }

    return true;
}


/* 先请求并确认停止；仅在持续通信故障时允许未确认释放。 */
ShutdownResult RobotInterface::shutdown() {
    const auto request = motor_session_.request_stop();
    initialized_.store(false);
    worker_.request_stop();

    bool stop_confirmed = true;
    if (motor_session_.rt_started_) {
        stop_confirmed = motor_session_.wait_for_stop(request);
        if (!stop_confirmed) {
            const bool persistent_communication_loss =
                motor_session_.communication_fault_latched() &&
                motor_session_.communication_state() !=
                    MotorCommunicationState::Healthy;
            if (!persistent_communication_loss) {
                return ShutdownResult::RetryRequired;
            }
        }
    }

    worker_.stop();
    record_completed_policy_frame();
    unload_policy();
    imu_session_.deinitialize();
    motor_session_.release_controller();
    return stop_confirmed ? ShutdownResult::Confirmed
                          : ShutdownResult::ReleasedAfterCommLoss;
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

    // 配置运行时线程数和 CPU 亲和性
    const int intra_op_threads = config_.runtime.enabled
                                     ? config_.runtime.torch_intra_op_threads
                                     : 0;

    const int inter_op_threads = config_.runtime.enabled
                                     ? config_.runtime.torch_inter_op_threads
                                     : 0;

    const int openblas_threads = config_.runtime.enabled
                                     ? config_.runtime.openblas_threads
                                     : 0;

    // 加载模型
    if (!policy_runtime_.load(config_.policy,
                              intra_op_threads,
                              inter_op_threads,
                              openblas_threads,
                              config_.runtime.enabled
                                  ? config_.runtime.policy_main.cpu_ids
                                  : std::vector<int>{})) {
        std::cerr << "[RobotInterface] load_policy failed: "
                  << policy_runtime_.last_error() << "\n";
        return false;
    }

    // 进行锁页
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
            policy_runtime_.shutdown();
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
            policy_runtime_.shutdown();
            return false;
        }
    }

    // 是否开启日志记录
    if (config_.recorder.enabled) {

        config_.recorder.policy_observation_size =
            policy_observation::observation_size(config_.policy.gait.enabled);

        const robot_base::ThreadRuntimeOptions recorder_thread =
            config_.runtime.enabled ? config_.runtime.background
                                    : robot_base::ThreadRuntimeOptions{};

        if (!inference_recorder_.start(config_.recorder, recorder_thread)) {
            std::cerr << "[RobotInterface] failed to open inference log: "
                      << inference_recorder_.last_error() << "\n";
            inference_recorder_failed_ = true;
            if (config_.runtime.enabled) {
                return false;
            }
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


/* 按平滑插值将关节恢复到 default_joint_pos_rad 初始姿态。 */
bool RobotInterface::reset_joints(std::vector<double>& final_target_motor_rad) {
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

    final_target_motor_rad = std::move(target_rad);
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
    reset_policy_command_state();
    if (action_processor_) {
        action_processor_->reset_runtime_state();
    }
    if (observation_builder_) {
        observation_builder_->reset_runtime_state();
    }
}

void RobotInterface::reset_policy_command_state() noexcept
{
    policy_step_phase_ = PolicyStepPhase::Idle;
    policy_step_phase_started_ns_ = 0;
    next_policy_seq_ = 1;
    last_policy_target_published_ns_ = 0;
    first_policy_inference_started_ns_.store(0, std::memory_order_release);
    worker_.reset_channels();
}

// 检测传感器的时间戳是否过期或不一致
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
    const std::int64_t skew_ns = imu_timestamp_ns - motor_timestamp_ns;

    if (imu_age_ns   > robot_base::seconds_to_ns(config_.sensor_guard.max_imu_sample_age_s) ||
        motor_age_ns > robot_base::seconds_to_ns(config_.sensor_guard.max_motor_sample_age_s) ||
        robot_base::abs_ns(skew_ns) > robot_base::seconds_to_ns(config_.sensor_guard.max_sensor_state_skew_s)) {
        error = "sensor timing guard failed: imu_age_us=" +
            std::to_string(robot_base::ns_to_us(imu_age_ns)) +
            ", motor_age_us=" + std::to_string(robot_base::ns_to_us(motor_age_ns)) +
            ", imu_motor_skew_us=" + std::to_string(robot_base::ns_to_us(skew_ns));
        return false;
    }
    return true;
}


std::uint64_t RobotInterface::begin_policy_inference(std::int64_t now_ns) noexcept
{
    if (first_policy_inference_started_ns_.load(std::memory_order_relaxed) == 0) {
        first_policy_inference_started_ns_.store(now_ns, std::memory_order_release);
    }
    return next_policy_seq_++;
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


void RobotInterface::record_completed_policy_frame()
{
    InferenceRecord record;
    if (worker_.try_consume_completed_record(record)) {
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

    const std::array<double, 3> target_velocity = get_target_velocity();

    std::string worker_error;

    if (!worker_.healthy(worker_error)) {
        return handle_policy_step_failure(worker_error);
    }
    if (!policy_runtime_.is_loaded()) {
        return handle_policy_step_failure("policy is not loaded");
    }
    if (!motor_session_.is_initialized()) {
        return handle_policy_step_failure("motors are not initialized");
    }

    set_policy_step_phase(PolicyStepPhase::SensorSnapshot);
    const MotorStateSnapshot motor_state = motor_session_.get_motor_snapshot();
    const std::size_t motor_count = static_cast<std::size_t>(config_.motor.num_motors);

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

    // 启动及关节复位期间暂缓 OFFLINE/WKC 终端锁存，
    // 从正式策略推理开始恢复通信保护。
    motor_session_.enable_communication_protection();
    set_policy_step_phase(PolicyStepPhase::Inference);
    const std::uint64_t policy_seq =
        begin_policy_inference(robot_base::monotonic_now_ns());
    PolicyRuntimeStepResult policy_result;
    if (!policy_runtime_.infer(policy_observation, policy_result)) {
        return handle_policy_step_failure(policy_runtime_.last_error());
    }

    set_policy_step_phase(PolicyStepPhase::PostInference);
    if (!worker_.healthy(worker_error)) {
        return handle_policy_step_failure(worker_error);
    }

    // admission 之前只完成控制目标计算，不组装日志记录。
    std::array<double, PolicyRuntime::kDof> target_q_model_rad;
    for (std::size_t model_index = 0; model_index < PolicyRuntime::kDof; ++model_index) {

        const double raw_action =
            static_cast<double>(policy_result.raw_action[model_index]);
        const double clipped_raw_action = config_.action.raw_action_clip
            ? std::max(-*config_.action.raw_action_clip,
                       std::min(*config_.action.raw_action_clip, raw_action))
            : raw_action;
        // 进行缩放
        const double scaled_action = clipped_raw_action * config_.action.action_scale[model_index];
        // 进行截断
        const auto& action_clip = config_.action.action_clip[model_index];
        const double clipped_action_offset =
            std::max(action_clip[0], std::min(action_clip[1], scaled_action));

        // 叠加模型顺序的站立姿态，得到模型顺序的目标关节角
        target_q_model_rad[model_index] =
            config_.action.default_joint_pos_rad[model_index] +
            clipped_action_offset;
    }

    set_policy_step_phase(PolicyStepPhase::Publish);
    const std::int64_t decision_now_ns = robot_base::monotonic_now_ns();
    const PolicyResultAdmission admission =
        admit_policy_result(observation_time_ns, decision_now_ns);

    // 两项年龄固定在 decision_now；后续日志处理不追溯改变准入结论。
    InferenceRecord record;
    record.frame_index = observation_builder_->frame_index();
    record.motor_sample_timestamp_ns = motor_state.timestamp_ns;
    for (std::size_t i = 0; i < motor_count; ++i) {
        record.rx_pos_rad[i]     = motor_state.position_rad[i];
        record.rx_vel_rad_s[i]   = motor_state.velocity_rad_s[i];
        record.torque_percent[i] = motor_state.torque_percent[i];
        record.comm_ok[i]        = motor_state.comm_ok[i];
        record.enabled[i]        = motor_state.enabled[i];
    }
    record.imu_sample_timestamp_ns = ahrs_state.sample_timestamp_ns;
    record.imu_receive_timestamp_ns = imu_receive_timestamp_ns;
    record.motor_age_us = robot_base::ns_to_us(timing_now_ns - motor_state.timestamp_ns);
    record.inference_start_ns = policy_result.inference_start_ns;
    record.inference_end_ns   = policy_result.inference_end_ns;
    record.policy_seq = policy_seq;
    record.policy_observation_time_ns = observation_time_ns;
    record.policy_observation = policy_observation;
    record.raw_action         = policy_result.raw_action;
    record.target_q_model_rad = target_q_model_rad;
    record.obs_to_action_age_us  = admission.obs_to_action_age_us;
    record.target_hold_age_us    = admission.target_hold_age_us;
    record.policy_result_dropped = admission.dropped;

    // 丢弃的结果不发布，不刷新发布时间。
    if (!admission.dropped) {
        robot_detail::PolicyTargetFrame target;
        target.policy_seq       = policy_seq;
        target.inference_record = record;
        last_policy_target_published_ns_ = worker_.publish_target(target);
        observation_builder_->commit_policy_action(record.raw_action);
    }

    // 推进观测帧计数；丢弃结果也会推进。
    observation_builder_->advance_frame();

    // 尝试收取 worker 完成的记录。
    record_completed_policy_frame();

    // 如果本次结果被丢弃，直接提交本次记录(worker 线程无法获取到丢弃的记录，因为没有在SPSC通道发布)
    if (record.policy_result_dropped) {
        record_policy_frame(record);
    }
    set_policy_step_phase(PolicyStepPhase::Idle);
    return true;
}

/* 故障入口先请求停止，再进行可能阻塞的诊断及 join。 */
bool RobotInterface::handle_policy_step_failure(const std::string& message) {

    motor_session_.request_stop();

    initialized_.store(false);

    worker_.request_stop();

    // 这里记录 Interface 处理故障时的阶段，worker 的停止请求已经发出。
    const std::int64_t diagnostic_now_ns = robot_base::monotonic_now_ns();
    constexpr const char* phase_names[] = {
        "idle", "precheck", "sensor_snapshot", "observation",
        "inference", "post_inference", "publish"};
    std::cerr << "[RobotInterface] policy_step failed: " << message
              << ", policy_step_phase=" << phase_names[static_cast<std::size_t>(policy_step_phase_)]
              << ", phase_elapsed_us="
              << robot_base::ns_to_us(diagnostic_now_ns - policy_step_phase_started_ns_) << "\n";
    
    worker_.stop();

    record_completed_policy_frame();

    set_policy_step_phase(PolicyStepPhase::Idle);
    return false;
}

}  // namespace inference
