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
#include <exception>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <sched.h>
#include <sys/mman.h>


namespace inference {

namespace {

bool verify_policy_main_thread(
    const robot_base::ThreadRuntimeOptions& options,
    std::string& error)
{
    cpu_set_t expected;
    CPU_ZERO(&expected);
    if (options.cpu_ids.empty()) {
        error = "policy_main CPU set is empty";
        return false;
    }
    for (const int cpu : options.cpu_ids) {
        if (cpu < 0 || cpu >= CPU_SETSIZE) {
            error = "policy_main CPU is outside cpu_set_t";
            return false;
        }
        CPU_SET(cpu, &expected);
    }

    cpu_set_t actual;
    CPU_ZERO(&actual);
    if (::sched_getaffinity(0, sizeof(actual), &actual) != 0) {
        error = std::string("policy_main sched_getaffinity failed: ") +
                std::strerror(errno);
        return false;
    }
    if (!CPU_EQUAL(&actual, &expected)) {
        std::ostringstream oss;
        oss << "policy_main affinity mismatch: actual={";
        bool first = true;
        for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
            if (CPU_ISSET(cpu, &actual)) {
                oss << (first ? "" : ",") << cpu;
                first = false;
            }
        }
        oss << "} expected={";
        for (std::size_t index = 0; index < options.cpu_ids.size(); ++index) {
            oss << (index == 0U ? "" : ",") << options.cpu_ids[index];
        }
        oss << "}";
        error = oss.str();
        return false;
    }

    const int policy = ::sched_getscheduler(0);
    sched_param parameters{};
    if (policy < 0 || ::sched_getparam(0, &parameters) != 0) {
        error = std::string("policy_main scheduler readback failed: ") +
                std::strerror(errno);
        return false;
    }
    if (policy != SCHED_OTHER || parameters.sched_priority != 0) {
        error = "policy_main must be SCHED_OTHER/0 before model initialization";
        return false;
    }
    return true;
}

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
      motor_session_(config_.motor, config_.safety, config_.runtime),
      imu_session_(config_.imu, config_.runtime)
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
    if (initialized_.load()) {
        return true;
    }

    if (shutdown() != ShutdownResult::Confirmed) return false;

    if (config_.runtime.enabled) {
        std::string error;
        if (!verify_policy_main_thread(config_.runtime.policy_main, error)) {
            std::cerr << "[RobotInterface] runtime preflight failed: "
                      << error << "\n";
            return false;
        }
    }

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
    if (!motor_session_.initialize(/*defer_communication_protection=*/true)) {
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

    // std::this_thread::sleep_for(std::chrono::seconds(10));

    initialized_.store(true);
    if (!start_policy_command_worker()) {
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
    policy_command_worker_running_.store(false);

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

    stop_policy_command_worker();
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

    const int intra_op_threads = config_.runtime.enabled
                                     ? config_.runtime.torch_intra_op_threads
                                     : 0;
    const int inter_op_threads = config_.runtime.enabled
                                     ? config_.runtime.torch_inter_op_threads
                                     : 0;
    const int openblas_threads = config_.runtime.enabled
                                     ? config_.runtime.openblas_threads
                                     : 0;
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
    next_policy_seq_ = 1;
    next_target_seq_ = 1;
    stale_policy_drop_count_ = 0;
    last_policy_target_published_ns_ = 0;
    first_policy_inference_started_ns_.store(0, std::memory_order_release);
    policy_target_channel_.reset_empty();
    completed_policy_record_channel_.reset_empty();
}

bool RobotInterface::validate_policy_sensor_timing(
    std::int64_t motor_timestamp_ns, std::int64_t imu_timestamp_ns,
    std::int64_t now_ns, std::string& error) const
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

    const std::int64_t imu_age_ns = now_ns - imu_timestamp_ns;
    const std::int64_t motor_age_ns = now_ns - motor_timestamp_ns;
    const std::int64_t skew_ns = imu_timestamp_ns - motor_timestamp_ns;
    if (imu_age_ns < 0 || motor_age_ns < 0) {
        error = "negative sensor age: imu_age_us=" +
            std::to_string(robot_base::ns_to_us(imu_age_ns)) +
            ", motor_age_us=" + std::to_string(robot_base::ns_to_us(motor_age_ns));
        return false;
    }
    if (imu_age_ns > robot_base::seconds_to_ns(config_.sensor_guard.max_imu_sample_age_s) ||
        motor_age_ns > robot_base::seconds_to_ns(config_.sensor_guard.max_motor_sample_age_s) ||
        robot_base::abs_ns(skew_ns) >
            robot_base::seconds_to_ns(config_.sensor_guard.max_sensor_state_skew_s)) {
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
    std::int64_t observation_time_ns, std::int64_t decision_now_ns) const noexcept
{
    const std::int64_t obs_age_ns = decision_now_ns - observation_time_ns;
    const std::int64_t hold_started_ns = last_policy_target_published_ns_ != 0
        ? last_policy_target_published_ns_
        : first_policy_inference_started_ns_.load(std::memory_order_acquire);
    PolicyResultAdmission admission;
    admission.obs_to_action_age_us = robot_base::ns_to_us(obs_age_ns);
    admission.target_hold_age_us = robot_base::ns_to_us(decision_now_ns - hold_started_ns);
    admission.dropped = obs_age_ns > kMaxObsToActionAgeNs;
    return admission;
}

void RobotInterface::publish_policy_target(
    PolicyTargetFrame& staged_target, std::int64_t published_at_ns) noexcept
{
    staged_target.target_seq = next_target_seq_;
    staged_target.published_at_ns = published_at_ns;
    staged_target.valid_until_ns = published_at_ns +
        robot_base::seconds_to_ns(config_.safety.policy_target_timeout_ms / 1000.0);
    staged_target.inference_record.target_seq = staged_target.target_seq;
    staged_target.inference_record.policy_valid_until_ns = staged_target.valid_until_ns;
    // publish_written() 返回是否覆盖旧帧，不是发布成功与否。
    policy_target_channel_.publish_written();
    ++next_target_seq_;
    last_policy_target_published_ns_ = published_at_ns;
}

void RobotInterface::complete_policy_result(const InferenceRecord& record)
{
    if (record.policy_result_dropped) {
        ++stale_policy_drop_count_;
    } else {
        observation_builder_->commit_policy_action(record.raw_action);
    }
    observation_builder_->advance_frame();
    observation_builder_->advance_episode();

    // recorder 加锁、队列复制及错误字符串处理均在控制发布/丢弃之后。
    record_latest_completed_policy_frame();
    if (record.policy_result_dropped) {
        record_inference(record);
    }
    set_policy_step_phase(PolicyStepPhase::Idle);
}

std::int64_t RobotInterface::startup_policy_deadline_ns() const noexcept
{
    const std::int64_t started_ns =
        first_policy_inference_started_ns_.load(std::memory_order_acquire);
    return started_ns == 0 ? 0 : started_ns +
        robot_base::seconds_to_ns(config_.safety.policy_target_timeout_ms / 1000.0);
}

bool RobotInterface::startup_policy_target_expired(std::int64_t now_ns) const noexcept
{
    const std::int64_t deadline_ns = startup_policy_deadline_ns();
    return deadline_ns != 0 && policy_deadline_expired(deadline_ns, now_ns);
}

bool RobotInterface::policy_deadline_expired(
    std::int64_t deadline_ns, std::int64_t now_ns) noexcept
{
    return now_ns >= deadline_ns;
}

motor_base::CommandTiming RobotInterface::policy_command_timing(
    const PolicyTargetFrame& target, std::int64_t produced_at_ns) const noexcept
{
    motor_base::CommandTiming timing;
    timing.source_policy_seq = target.policy_seq;
    timing.produced_at_ns = produced_at_ns;
    timing.valid_until_ns = std::min(target.valid_until_ns, produced_at_ns +
        robot_base::seconds_to_ns(config_.safety.control_command_timeout_ms / 1000.0));
    return timing;
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

void RobotInterface::record_latest_completed_policy_frame()
{
    InferenceRecord record;
    if (completed_policy_record_channel_.try_consume_latest(record)) {
        record_inference(record);
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

    reset_policy_command_state();
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

    const robot_base::ThreadRuntimeOptions worker_options =
        config_.runtime.enabled ? config_.runtime.policy_command
                                : robot_base::ThreadRuntimeOptions{};
    std::string thread_error;
    if (!robot_base::start_configured_thread(
            policy_command_worker_thread_, "policy_cmd", worker_options,
            [this] { policy_command_worker_loop(); }, thread_error)) {
        policy_command_worker_running_.store(false);
        std::cerr << "[RobotInterface] failed to start policy command worker: "
                  << thread_error << "\n";
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
    record_latest_completed_policy_frame();
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
    std::uint64_t last_logged_policy_seq = 0;

    try {
        while (policy_command_worker_running_.load()) {
            next_wake += period;

            PolicyTargetFrame latest_target;
            if (policy_target_channel_.try_consume_latest(latest_target)) {
                current_target = latest_target;
                has_current_target = true;
            }

            if (!has_current_target) {
                const std::int64_t deadline_ns = startup_policy_deadline_ns();
                const std::int64_t now_ns = robot_base::monotonic_now_ns();
                if (startup_policy_target_expired(now_ns)) {
                    PolicyTargetFrame startup_target;
                    startup_target.valid_until_ns = startup_policy_deadline_ns();
                    startup_target.published_at_ns = startup_target.valid_until_ns -
                        robot_base::seconds_to_ns(config_.safety.policy_target_timeout_ms / 1000.0);
                    fail_policy_command_worker(policy_deadline_error(startup_target, now_ns));
                    break;
                }
                // 首帧前复位命令也封顶首次正式推理的截止期。
                if (!motor_session_.apply_targets_rad(
                        startup_hold_target_motor_rad_, deadline_ns)) {
                    const std::int64_t failed_now_ns = robot_base::monotonic_now_ns();
                    if (startup_policy_target_expired(failed_now_ns)) {
                        PolicyTargetFrame startup_target;
                        startup_target.valid_until_ns = startup_policy_deadline_ns();
                        startup_target.published_at_ns = startup_target.valid_until_ns -
                            robot_base::seconds_to_ns(config_.safety.policy_target_timeout_ms / 1000.0);
                        fail_policy_command_worker(policy_deadline_error(startup_target, failed_now_ns));
                    } else {
                        fail_policy_command_worker("failed to refresh startup reset pose");
                    }
                    break;
                }
                std::this_thread::sleep_until(next_wake);
                const auto now = std::chrono::steady_clock::now();
                if (now > next_wake + period) {
                    next_wake = now;
                }
                continue;
            }

            if (current_target.policy_seq == 0 || current_target.target_seq == 0) {
                fail_policy_command_worker("policy target sequence is missing");
                break;
            }

            const std::int64_t now_ns = robot_base::monotonic_now_ns();
            if (policy_deadline_expired(current_target.valid_until_ns, now_ns)) {
                fail_policy_command_worker(
                    policy_deadline_error(current_target, now_ns));
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
            if (policy_deadline_expired(current_target.valid_until_ns, produced_at_ns)) {
                fail_policy_command_worker(
                    policy_deadline_error(current_target, produced_at_ns));
                break;
            }
            const motor_base::CommandTiming timing =
                policy_command_timing(current_target, produced_at_ns);

            const bool applied =
                motor_session_.apply_impedance_setpoints_realtime(
                    command.setpoints,
                    command.setpoint_count,
                    timing);

            if (!applied) {
                fail_policy_command_worker("failed to apply policy impedance command");
                break;
            }

            if (current_target.policy_seq != last_logged_policy_seq) {
                InferenceRecord completed_record = current_target.inference_record;
                completed_record.command_timestamp_ns = produced_at_ns;
                completed_record.command_produced_at_ns = produced_at_ns;
                completed_record.command_valid_until_ns = timing.valid_until_ns;
                completed_record.command_applied = true;
                for (std::size_t i = 0; i < PolicyRuntime::kDof; ++i) {
                    completed_record.target_pos_rad[i] =
                        command.setpoints[i].position_rad;
                    completed_record.target_effort_permille[i] =
                        command.target_effort_permille[i];
                }
                completed_policy_record_channel_.publish(completed_record);
                last_logged_policy_seq = current_target.policy_seq;
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

void RobotInterface::fail_policy_command_worker(std::string message)
{
    motor_session_.request_stop();
    policy_command_worker_running_.store(false);
    initialized_.store(false);
    const std::string printable_message = message;
    const bool already_failed = policy_command_worker_failed_.exchange(true);
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

void RobotInterface::set_policy_step_phase(PolicyStepPhase phase) noexcept
{
    policy_step_phase_started_ns_.store(robot_base::monotonic_now_ns(),
                                        std::memory_order_relaxed);
    policy_step_phase_.store(phase, std::memory_order_release);
}

std::string RobotInterface::policy_deadline_error(
    const PolicyTargetFrame& target,
    std::int64_t now_ns) const
{
    const PolicyStepPhase phase =
        policy_step_phase_.load(std::memory_order_acquire);
    const std::int64_t phase_started_ns =
        policy_step_phase_started_ns_.load(std::memory_order_relaxed);

    const char* phase_name = "unknown";
    switch (phase) {
        case PolicyStepPhase::Idle:           phase_name = "idle"; break;
        case PolicyStepPhase::Precheck:       phase_name = "precheck"; break;
        case PolicyStepPhase::SensorSnapshot: phase_name = "sensor_snapshot"; break;
        case PolicyStepPhase::Observation:    phase_name = "observation"; break;
        case PolicyStepPhase::Inference:      phase_name = "inference"; break;
        case PolicyStepPhase::PostInference:  phase_name = "post_inference"; break;
        case PolicyStepPhase::Publish:        phase_name = "publish"; break;
    }

    std::ostringstream stream;
    stream << "policy target deadline expired: policy_seq="
           << target.policy_seq
           << ", target_seq=" << target.target_seq
           << ", target_hold_age_us="
           << robot_base::ns_to_us(now_ns - target.published_at_ns)
           << ", obs_to_action_age_us=" << target.inference_record.obs_to_action_age_us
           << ", overdue_us="
           << robot_base::ns_to_us(now_ns - target.valid_until_ns)
           << ", policy_step_phase=" << phase_name
           << ", phase_elapsed_us=";
    if (phase_started_ns > 0 && now_ns >= phase_started_ns) {
        stream << robot_base::ns_to_us(now_ns - phase_started_ns);
    } else {
        stream << "unknown";
    }
    return stream.str();
}

/* 执行一次策略闭环：使用保存的目标速度构建观测、模型推理并下发目标关节角。 */
bool RobotInterface::policy_step() {

    set_policy_step_phase(PolicyStepPhase::Precheck);

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

    set_policy_step_phase(PolicyStepPhase::SensorSnapshot);
    const MotorStateSnapshot motor_state = motor_session_.get_motor_snapshot();
    const std::size_t motor_count = static_cast<std::size_t>(config_.motor.num_motors);

    AhrsStateSnapshot ahrs_state;
    if (!imu_session_.get_ahrs_snapshot(ahrs_state)) {
        return handle_policy_step_failure("AHRS data is not ready");
    }

    const std::int64_t timing_now_ns = robot_base::monotonic_now_ns();
    const std::int64_t imu_receive_timestamp_ns = ahrs_state.receive_timestamp_ns;

    if (!ahrs_state.ahrs_ready) {
        return handle_policy_step_failure("AHRS data is not ready");
    }
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
    if (!policy_command_worker_healthy(worker_error)) {
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
    fill_record_motor_state(motor_state, motor_count, record);
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
    record.obs_to_action_age_us = admission.obs_to_action_age_us;
    record.target_hold_age_us = admission.target_hold_age_us;
    record.policy_result_dropped = admission.dropped;

    if (admission.dropped) {
        // 无 target 可发布：不分配 target_seq，也不刷新发布时间。
        complete_policy_result(record);
        return true;
    }

    PolicyTargetFrame* staged_target = policy_target_channel_.acquire_write_slot();
    staged_target->target_q_model_rad = target_q_model_rad;
    staged_target->policy_seq = policy_seq;
    staged_target->observation_time_ns = observation_time_ns;
    staged_target->inference_record = record;
    // 大块固定日志复制已完成；采样后只写时间/序号标量并发布写槽。
    publish_policy_target(*staged_target, robot_base::monotonic_now_ns());
    complete_policy_result(staged_target->inference_record);
    return true;
}

/* 故障入口先请求停止，再进行可能阻塞的诊断及 join。 */
bool RobotInterface::handle_policy_step_failure(const std::string& message) {
    motor_session_.request_stop();
    initialized_.store(false);
    policy_command_worker_running_.store(false);
    std::cerr << "[RobotInterface] policy_step failed: " << message << "\n";
    stop_policy_command_worker();
    set_policy_step_phase(PolicyStepPhase::Idle);
    return false;
}

}  // namespace inference
