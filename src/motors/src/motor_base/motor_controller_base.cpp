#include "motor_base/motor_controller_base.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <pthread.h>
#include <sched.h>
#include <stdexcept>
#include <time.h>
#include <utility>

namespace motor_base {

namespace {
constexpr long kNsecPerSec = 1000000000L;
constexpr clockid_t kClockToUse = CLOCK_MONOTONIC;

void add_period_ns(timespec& time, long period_ns)
{
    time.tv_sec += period_ns / kNsecPerSec;
    time.tv_nsec += period_ns % kNsecPerSec;
    while (time.tv_nsec >= kNsecPerSec) {
        time.tv_nsec -= kNsecPerSec;
        ++time.tv_sec;
    }
}

const char* scheduling_policy_name(int policy)
{
    switch (policy) {
        case SCHED_FIFO: return "SCHED_FIFO";
        case SCHED_RR: return "SCHED_RR";
        case SCHED_OTHER: return "SCHED_OTHER";
#ifdef SCHED_BATCH
        case SCHED_BATCH: return "SCHED_BATCH";
#endif
#ifdef SCHED_IDLE
        case SCHED_IDLE: return "SCHED_IDLE";
#endif
    }
    return "UNKNOWN";
}


bool read_thread_scheduling(std::thread& thread,
                            int& policy,
                            sched_param& param)
{
    const int result =
        pthread_getschedparam(thread.native_handle(), &policy, &param);
    if (result != 0) {
        std::cerr << "[MotorControllerBase] failed to read realtime "
                  << "scheduling: " << std::strerror(result) << std::endl;
        return false;
    }
    return true;
}


void print_realtime_scheduling_state(int requested_priority,
                                     bool actual_available,
                                     int actual_policy,
                                     int actual_priority)
{
    std::cerr << "[MotorControllerBase] Requested realtime scheduling:\n"
              << "  policy   = SCHED_FIFO\n"
              << "  priority = " << requested_priority << "\n";
    if (actual_available) {
        std::cerr << "[MotorControllerBase] Actual scheduling:\n"
                  << "  policy   = " << scheduling_policy_name(actual_policy) << "\n"
                  << "  priority = " << actual_priority << std::endl;
    } else {
        std::cerr << "[MotorControllerBase] Actual scheduling: unavailable"
                  << std::endl;
    }
}


bool verify_realtime_priority(std::thread& thread, int expected_priority)
{
    int policy = 0;
    sched_param actual{};
    if (!read_thread_scheduling(thread, policy, actual)) {
        print_realtime_scheduling_state(expected_priority, false, 0, 0);
        return false;
    }

    if (policy == SCHED_FIFO && actual.sched_priority == expected_priority) {
        return true;
    }

    std::cerr << "[MotorControllerBase] realtime scheduling verification failed"
              << std::endl;
    print_realtime_scheduling_state(
        expected_priority,
        true,
        policy,
        actual.sched_priority);
    return false;
}


bool set_realtime_priority(std::thread& thread, int priority)
{
    if (priority <= 0) {
        return true;
    }

    sched_param param{};
    param.sched_priority = priority;
    const int result =
        pthread_setschedparam(thread.native_handle(), SCHED_FIFO, &param);
    if (result != 0) {
        int actual_policy = 0;
        sched_param actual{};
        const bool actual_available =
            read_thread_scheduling(thread, actual_policy, actual);
        std::cerr << "[MotorControllerBase] failed to set realtime scheduling: "
                  << std::strerror(result) << std::endl;
        print_realtime_scheduling_state(
            priority,
            actual_available,
            actual_policy,
            actual.sched_priority);
        return false;
    }

    return verify_realtime_priority(thread, priority);
}

const char* setpoint_source_name(SetpointSource source)
{
    switch (source) {
        case SetpointSource::POLICY: return "POLICY";
        case SetpointSource::DEBUG: return "DEBUG";
    }
    return "UNKNOWN";
}
} // namespace


MotorControllerBase::MotorControllerBase(std::size_t motor_count)
    : MotorControllerBase(motor_count, RealtimeOptions())
{
}

MotorControllerBase::MotorControllerBase(
    std::size_t motor_count,
    RealtimeOptions options)
    : rt_options_(options),
      motor_count_(motor_count),
      cmd_queue_(rt_options_.command_queue_capacity),
      status_channel_(),
      rt_event_dispatcher_(rt_options_.rt_event_queue_capacity)
{
    if (rt_options_.max_commands_per_cycle == 0) {
        throw std::invalid_argument(
            "RealtimeOptions max_commands_per_cycle must be positive");
    }
    if (rt_options_.rt_period_ns <= 0) {
        throw std::invalid_argument(
            "RealtimeOptions rt_period_ns must be positive");
    }
    if (rt_options_.status_publish_period_ms <= 0) {
        throw std::invalid_argument(
            "RealtimeOptions status_publish_period_ms must be positive");
    }
    status_channel_.configure(motor_count_, rt_options_.status_publish_period_ms);
    setpoint_channel_debug_.reset_empty();
    setpoint_channel_policy_.reset_empty();
    command_feedback_channel_.reset_empty();
    policy_feedback_channel_.reset_empty();

    discrete_cmd_queues_.reserve(motor_count_);
    for (std::size_t i = 0; i < motor_count_; ++i) {
        discrete_cmd_queues_.emplace_back(
            rt_options_.discrete_queue_capacity_per_motor);
    }
}

MotorControllerBase::~MotorControllerBase()
{
    rt_scheduling_ready_.store(false, std::memory_order_release);
    running_.store(false, std::memory_order_release);
    if (rt_thread_.joinable()) {
        rt_thread_.join();
    }
    rt_event_dispatcher_.stop();
    status_channel_.stop();
}


bool MotorControllerBase::connect(const char* interface_name)
{
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (running_.load(std::memory_order_acquire)) {
        return false;
    }

    return connect_impl(interface_name);
}


bool MotorControllerBase::start()
{
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (running_.load(std::memory_order_acquire)) {
        return rt_scheduling_ready_.load(std::memory_order_acquire);
    }

    rt_scheduling_ready_.store(false, std::memory_order_release);
    setpoint_channel_debug_.reset_empty();
    setpoint_channel_policy_.reset_empty();
    command_feedback_channel_.reset_empty();
    policy_feedback_channel_.reset_empty();
    status_channel_.start();
    rt_event_dispatcher_.start();

    if (!realtime_start_callback()) {
        rt_event_dispatcher_.stop();
        status_channel_.stop();
        return false;
    }

    running_.store(true, std::memory_order_release);
    try {
        rt_thread_ = std::thread(&MotorControllerBase::thread_func, this);
    } catch (...) {
        running_.store(false, std::memory_order_release);
        rt_scheduling_ready_.store(false, std::memory_order_release);
        realtime_stop_callback();
        rt_event_dispatcher_.stop();
        status_channel_.stop();
        throw;
    }

    const bool scheduling_ready =
        set_realtime_priority(rt_thread_, rt_options_.rt_priority);
    rt_scheduling_ready_.store(scheduling_ready, std::memory_order_release);
    if (!scheduling_ready) {
        running_.store(false, std::memory_order_release);
        if (rt_thread_.joinable()) {
            rt_thread_.join();
        }
        realtime_stop_callback();
        rt_event_dispatcher_.stop();
        status_channel_.stop();
        return false;
    }

    return true;
}


bool MotorControllerBase::realtime_start_callback()
{
    return true;
}


void MotorControllerBase::shutdown()
{
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    const bool was_running = running_.exchange(false, std::memory_order_acq_rel);
    rt_scheduling_ready_.store(false, std::memory_order_release);

    if (rt_thread_.joinable()) {
        rt_thread_.join();
    }

    if (was_running) {
        realtime_stop_callback();
        rt_event_dispatcher_.stop();
        status_channel_.stop();
    }
}


void MotorControllerBase::realtime_stop_callback() noexcept
{
}


void MotorControllerBase::thread_func()
{
    timespec next_period{};
    clock_gettime(kClockToUse, &next_period);

    while (running_.load(std::memory_order_acquire)) {
        // 进行离散命令分发：离散命令入各个电机的离散命令队列。
        process_queued_commands();
        // 处理各个电机的离散命令队列（状态机）
        service_discrete_commands();
        process_latest_setpoint_commands();

        realtime_cycle_callback();

        add_period_ns(next_period, rt_options_.rt_period_ns);
        clock_nanosleep(kClockToUse, TIMER_ABSTIME, &next_period, nullptr);
    }
}


/// @brief 异步发送离散控制命令。
/// @brief 进行通用性检查并调用 validate_command() 进行驱动特定检查。

CommandSubmitResult MotorControllerBase::send_discrete_command(
    const ControlCommand& cmd)
{
    if (cmd.kind != ControlCommandKind::DISCRETE) {
        return {CommandSubmitStatus::INVALID_COMMAND, std::nullopt};
    }

    // 通用性检查，如电机索引、数据长度
    if (!cmd.payload_valid) {
        return {CommandSubmitStatus::INVALID_PAYLOAD, std::nullopt};
    }

    if (cmd.motor_index < ControlCommand::kAllMotors ||
        cmd.motor_index >= static_cast<int>(motor_count_)) {
        return {CommandSubmitStatus::INVALID_COMMAND, std::nullopt};
    }

    // 如果需要实时检查实时调度是否可用
    const bool realtime_required = rt_options_.rt_priority > 0;
    const bool realtime_ready    = rt_scheduling_ready_.load(std::memory_order_acquire);
    if (realtime_required && !realtime_ready) {
        if (cmd.discrete_type == DiscreteCommandType::RESTART) {
            std::cerr << "[MotorControllerBase] Motion enable rejected: "
                      << "realtime scheduling is not active." << std::endl;
            return {CommandSubmitStatus::INVALID_COMMAND, std::nullopt};
        }
    }

    const CommandSubmitStatus driver_validation = validate_command(cmd);
    if (driver_validation != CommandSubmitStatus::ACCEPTED) {
        return {driver_validation, std::nullopt};
    }

    // 对于离散命令，分配 command_id 并入命令队列
    std::lock_guard<std::mutex> lock(discrete_command_submission_mutex_);
    const CommandId command_id =
        next_discrete_command_id_.fetch_add(1, std::memory_order_relaxed);
    discrete_command_results_.initialize(
        command_id,
        discrete_command_target_mask(cmd, motor_count_));

    // 入命令队列失败时，清除命令结果缓存
    if (!cmd_queue_.try_push({cmd, command_id})) {
        discrete_command_results_.clear(command_id);
        return {CommandSubmitStatus::QUEUE_FULL, std::nullopt};
    }

    return {CommandSubmitStatus::ACCEPTED, command_id};
}

CommandSubmitResult MotorControllerBase::send_policy_setpoint(
    const ControlCommand& cmd)
{
    if (!cmd.payload_valid) {
        return {CommandSubmitStatus::INVALID_PAYLOAD, std::nullopt};
    }

    if (cmd.kind != ControlCommandKind::SETPOINT ||
        cmd.motor_index != ControlCommand::kAllMotors) {
        return {CommandSubmitStatus::INVALID_COMMAND, std::nullopt};
    }

    switch (cmd.setpoint_type) {
        case SetpointCommandType::POSITION_TARGETS:
        case SetpointCommandType::VELOCITY_TARGETS:
        case SetpointCommandType::TORQUE_TARGETS:
        case SetpointCommandType::IMPEDANCE_TARGETS:
            break;
        default:
            return {CommandSubmitStatus::INVALID_COMMAND, std::nullopt};
    }

    if (cmd.payload_size != motor_count_) {
        return {CommandSubmitStatus::INVALID_PAYLOAD, std::nullopt};
    }

    if (active_setpoint_source_ != SetpointSource::POLICY) {
        std::cerr << "[MotorControllerBase] Policy setpoint rejected: active "
                  << "setpoint source is "
                  << setpoint_source_name(active_setpoint_source_) << "."
                  << std::endl;
        return {CommandSubmitStatus::SOURCE_INACTIVE, std::nullopt};
    }

    const bool realtime_required = rt_options_.rt_priority > 0;
    const bool realtime_ready =
        rt_scheduling_ready_.load(std::memory_order_acquire);
    if (realtime_required && !realtime_ready) {
        std::cerr << "[MotorControllerBase] Motion command rejected: "
                  << "realtime scheduling is not active." << std::endl;
        return {CommandSubmitStatus::INVALID_COMMAND, std::nullopt};
    }

    const CommandSubmitStatus driver_validation = validate_command(cmd);
    if (driver_validation != CommandSubmitStatus::ACCEPTED) {
        return {driver_validation, std::nullopt};
    }

    setpoint_channel_policy_.publish(cmd);
    return {CommandSubmitStatus::ACCEPTED, std::nullopt};
}

CommandSubmitResult MotorControllerBase::send_debug_setpoint(
    const ControlCommand& cmd)
{
    if (!cmd.payload_valid) {
        return {CommandSubmitStatus::INVALID_PAYLOAD, std::nullopt};
    }

    if (cmd.kind != ControlCommandKind::SETPOINT ||
        cmd.motor_index != ControlCommand::kAllMotors) {
        return {CommandSubmitStatus::INVALID_COMMAND, std::nullopt};
    }

    switch (cmd.setpoint_type) {
        case SetpointCommandType::POSITION_TARGETS:
        case SetpointCommandType::VELOCITY_TARGETS:
        case SetpointCommandType::TORQUE_TARGETS:
        case SetpointCommandType::IMPEDANCE_TARGETS:
            break;
        default:
            return {CommandSubmitStatus::INVALID_COMMAND, std::nullopt};
    }

    if (cmd.payload_size != motor_count_) {
        return {CommandSubmitStatus::INVALID_PAYLOAD, std::nullopt};
    }

    if (active_setpoint_source_ != SetpointSource::DEBUG) {
        std::cerr << "[MotorControllerBase] Debug setpoint rejected: active "
                  << "setpoint source is "
                  << setpoint_source_name(active_setpoint_source_) << "."
                  << std::endl;
        return {CommandSubmitStatus::SOURCE_INACTIVE, std::nullopt};
    }

    const bool realtime_required = rt_options_.rt_priority > 0;
    const bool realtime_ready =
        rt_scheduling_ready_.load(std::memory_order_acquire);
    if (realtime_required && !realtime_ready) {
        std::cerr << "[MotorControllerBase] Motion command rejected: "
                  << "realtime scheduling is not active." << std::endl;
        return {CommandSubmitStatus::INVALID_COMMAND, std::nullopt};
    }

    const CommandSubmitStatus driver_validation = validate_command(cmd);
    if (driver_validation != CommandSubmitStatus::ACCEPTED) {
        return {driver_validation, std::nullopt};
    }

    setpoint_channel_debug_.publish(cmd);
    return {CommandSubmitStatus::ACCEPTED, std::nullopt};
}

void MotorControllerBase::set_active_setpoint_source(SetpointSource source)
{
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (running_.load(std::memory_order_acquire)) {
        std::cerr << "[MotorControllerBase] set_active_setpoint_source("
                  << setpoint_source_name(source)
                  << ") ignored: controller is running." << std::endl;
        return;
    }
    active_setpoint_source_ = source;
}


CommandSubmitStatus MotorControllerBase::validate_command(const ControlCommand&) const
{
    return CommandSubmitStatus::ACCEPTED;
}


std::vector<MotorStatusSnapshot> MotorControllerBase::get_status()
{
    return status_channel_.get_status();
}

DiscreteCommandResult MotorControllerBase::get_discrete_command_result(
    CommandId id) const
{
    return discrete_command_results_.get(id);
}

bool MotorControllerBase::try_consume_latest_status_command(
    std::array<MotorStatusSnapshot, kMaxMotorCommandSetpoints>& feedback)
{
    return command_feedback_channel_.try_consume_latest(feedback);
}

bool MotorControllerBase::try_consume_latest_status_policy(
    std::array<MotorStatusSnapshot, kMaxMotorCommandSetpoints>& feedback)
{
    return policy_feedback_channel_.try_consume_latest(feedback);
}


void MotorControllerBase::set_status_callback(StatusCallback cb)
{
    status_channel_.set_callback(std::move(cb));
}


void MotorControllerBase::set_event_callback(RtEventCallback cb)
{
    rt_event_dispatcher_.set_callback(std::move(cb));
}


bool MotorControllerBase::write_status(StatusWriteToken& token)
{
    return status_channel_.write(token);
}


void MotorControllerBase::publish_status(const StatusWriteToken& token)
{
    status_channel_.publish(token);
}

void MotorControllerBase::publish_feedback(
    const std::array<MotorStatusSnapshot, kMaxMotorCommandSetpoints>& feedback)
{
    // 同一帧 fan-out 到两条独立 SPSC 通道，各自保持 1 producer + 1 consumer 契约
    command_feedback_channel_.publish(feedback);
    policy_feedback_channel_.publish(feedback);
}


void MotorControllerBase::push_event(const RtEvent& event)
{
    rt_event_dispatcher_.push(event);
}


void MotorControllerBase::set_event_fallback_printer(
    RtEventDispatcher::EventPrinter printer)
{
    rt_event_dispatcher_.set_fallback_printer(std::move(printer));
}

void MotorControllerBase::process_queued_commands()
{
    std::size_t processed = 0;
    while (processed < rt_options_.max_commands_per_cycle) {
        const DiscreteCommandSubmissionQueue::Entry* entry = cmd_queue_.front();
        if (!entry) {
            break;
        }
        const ControlCommand& cmd = entry->command;
        ++processed;
        if (cmd.kind == ControlCommandKind::DISCRETE) {
            enqueue_discrete_command(cmd, *entry->command_id);
        }
        cmd_queue_.pop_front();
    }
}

void MotorControllerBase::process_latest_setpoint_commands()
{
    ControlCommand cmd;
    switch (active_setpoint_source_) {
        case SetpointSource::POLICY:
            if (setpoint_channel_policy_.try_consume_latest(cmd)) {
                apply_setpoint_command_impl(cmd);
            }
            break;

        case SetpointSource::DEBUG:
            if (setpoint_channel_debug_.try_consume_latest(cmd)) {
                apply_setpoint_command_impl(cmd);
            }
            break;
    }
}

void MotorControllerBase::enqueue_discrete_command(
    const ControlCommand& cmd,
    CommandId command_id)
{
    auto enqueue_one = [this, &cmd, command_id](int idx) {
        if (idx < 0 || idx >= static_cast<int>(motor_count_)) {
            return;
        }

        DiscreteCommand pending(cmd.discrete_type, cmd.mode, command_id);
        pending.phase = DiscretePhase::QUEUED;
        pending.from_all_motors =
            (cmd.motor_index == ControlCommand::kAllMotors);

        pending.enqueue_tick    = discrete_cmd_tick_;
        pending.next_retry_tick = discrete_cmd_tick_;
        pending.next_verify_tick = discrete_cmd_tick_;
        pending.deadline_tick = discrete_cmd_tick_ + kDiscreteTimeoutTicks;

        pending.cur_retry = 0;
        pending.max_retries = kDiscreteMaxRetries;

        pending.stable_success_cycles = 0;
        pending.fail_reason = DiscreteFailReason::NONE;

        if (!discrete_cmd_queues_[static_cast<std::size_t>(idx)].push_back(pending)) {
            discrete_command_results_.mark_failed(command_id, idx);
            discrete_queue_full_callback(idx, cmd);
        }
    };

    if (cmd.motor_index == ControlCommand::kAllMotors) {
        for (std::size_t i = 0; i < motor_count_; ++i) {
            enqueue_one(static_cast<int>(i));
        }
    } else {
        enqueue_one(cmd.motor_index);
    }
}

void MotorControllerBase::discrete_queue_full_callback(
    int,
    const ControlCommand&)
{
    printf("[MotorControllerBase] Warning: discrete command queue full\n");
}

// thread_func()中调用，处理各个电机的离散命令队列（状态机）
void MotorControllerBase::service_discrete_commands()
{
    ++discrete_cmd_tick_;

    /* 遍历每个电机的离散命令队列 */
    for (std::size_t i = 0; i < discrete_cmd_queues_.size(); ++i) {
        auto& queue = discrete_cmd_queues_[i];

        /* 空队列则跳过 */
        if (queue.empty()) {
            continue;
        }

        /* 取出命令 */
        auto& cmd = queue.front();
        const int motor_index = static_cast<int>(i);

        /* 检查命令状态机 */
        if (cmd.phase == DiscretePhase::DONE) {
            discrete_command_results_.mark_done(cmd.command_id, motor_index);
            queue.pop_front();
            continue;
        }

        if (cmd.phase == DiscretePhase::FAILED) {
            discrete_command_results_.mark_failed(cmd.command_id, motor_index);
            discrete_command_failed_callback(motor_index, cmd, cmd.fail_reason);
            queue.pop_front();
            continue;
        }

        // 超时
        if (discrete_cmd_tick_ > cmd.deadline_tick) {
            cmd.phase = DiscretePhase::FAILED;
            cmd.fail_reason = DiscreteFailReason::TIMEOUT;
            continue;
        }

        if (cmd.phase == DiscretePhase::QUEUED) {
            cmd.phase = DiscretePhase::APPLY_PENDING;
        }

        if (cmd.phase == DiscretePhase::APPLY_PENDING) {
            // 超过最大重试次数
            if (cmd.cur_retry >= cmd.max_retries) {
                cmd.phase = DiscretePhase::FAILED;
                cmd.fail_reason = DiscreteFailReason::MAX_RETRY;
                continue;
            }

            // 到达再次重试时间
            if (discrete_cmd_tick_ >= cmd.next_retry_tick) {
                apply_discrete_command_impl(motor_index, cmd);
                cmd.cur_retry += 1;
                cmd.next_retry_tick  = discrete_cmd_tick_ + kDiscreteRetryTicks;

                cmd.next_verify_tick = discrete_cmd_tick_ + kDiscreteVerifyIntervalTicks;
                cmd.stable_success_cycles = 0;
                cmd.phase = DiscretePhase::VERIFYING;
            }
            continue;
        }

        if (cmd.phase == DiscretePhase::VERIFYING) {
            // 还未到达下次验证时间点
            if (discrete_cmd_tick_ < cmd.next_verify_tick) {
                continue;
            }

            const DiscreteCommandEvaluation evaluation = evaluate_discrete_command_impl(motor_index, cmd);
            switch (evaluation) {
                case DiscreteCommandEvaluation::FAILED:
                    cmd.phase = DiscretePhase::FAILED;
                    cmd.fail_reason = DiscreteFailReason::FAULT;
                    continue;

                case DiscreteCommandEvaluation::PENDING:
                    cmd.stable_success_cycles = 0;
                    if (discrete_cmd_tick_ >= cmd.next_retry_tick) {
                        cmd.phase = DiscretePhase::APPLY_PENDING;
                    } else {
                        cmd.next_verify_tick = discrete_cmd_tick_ + kDiscreteVerifyIntervalTicks;
                    }
                    continue;

                case DiscreteCommandEvaluation::SATISFIED:
                    cmd.stable_success_cycles += 1;
                    if (cmd.stable_success_cycles >= kDiscreteSuccessStableTicks) {
                        cmd.phase = DiscretePhase::DONE;
                        discrete_command_results_.mark_done(
                            cmd.command_id,
                            motor_index);
                        queue.pop_front();
                    } else {
                        cmd.next_verify_tick = discrete_cmd_tick_ + kDiscreteVerifyIntervalTicks;
                    }
                    continue;
            }
        }
    }
}

void MotorControllerBase::discrete_command_failed_callback(
    int,
    const DiscreteCommand&,
    DiscreteFailReason)
{
    printf("[MotorControllerBase] Warning: discrete command failed\n");
}

bool MotorControllerBase::is_running() const
{
    return running_.load(std::memory_order_acquire);
}


std::vector<double> MotorControllerBase::get_positions_rad()
{
    const auto status = get_status();
    std::vector<double> q(status.size(), 0.0);
    for (std::size_t i = 0; i < status.size(); ++i) {
        q[i] = status[i].position_rad;
    }
    return q;
}

} // namespace motor_base
