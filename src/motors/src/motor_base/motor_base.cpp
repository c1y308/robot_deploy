#include "motor_base/motor_base.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <pthread.h>
#include <sched.h>
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

void set_realtime_priority(std::thread& thread, int priority)
{
    if (priority <= 0) {
        return;
    }

    sched_param param{};
    param.sched_priority = priority;
    const int result =
        pthread_setschedparam(thread.native_handle(), SCHED_FIFO, &param);
    if (result != 0) {
        std::cerr << "[MotorControllerBase] Warning: failed to set realtime "
                  << "scheduling (SCHED_FIFO priority " << priority
                  << "): " << std::strerror(result) << std::endl;
    }
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
    rt_options_.max_commands_per_cycle =
        std::max<std::size_t>(1, rt_options_.max_commands_per_cycle);
    rt_options_.rt_period_ns = std::max<long>(1, rt_options_.rt_period_ns);
    rt_options_.status_publish_period_ms =
        std::max(1, rt_options_.status_publish_period_ms);
    status_channel_.configure(motor_count_, rt_options_.status_publish_period_ms);

    discrete_cmd_queues_.reserve(motor_count_);
    for (std::size_t i = 0; i < motor_count_; ++i) {
        discrete_cmd_queues_.emplace_back(
            rt_options_.discrete_queue_capacity_per_motor);
    }
}

MotorControllerBase::~MotorControllerBase()
{
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

void MotorControllerBase::start()
{
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (running_.load(std::memory_order_acquire)) {
        return;
    }

    status_channel_.start();
    rt_event_dispatcher_.start();

    if (!realtime_start_callback()) {
        rt_event_dispatcher_.stop();
        status_channel_.stop();
        return;
    }

    running_.store(true, std::memory_order_release);
    try {
        rt_thread_ = std::thread(&MotorControllerBase::thread_func, this);
    } catch (...) {
        running_.store(false, std::memory_order_release);
        realtime_stop_callback();
        rt_event_dispatcher_.stop();
        status_channel_.stop();
        throw;
    }

    set_realtime_priority(rt_thread_, rt_options_.rt_priority);
}

bool MotorControllerBase::realtime_start_callback()
{
    return true;
}


void MotorControllerBase::shutdown()
{
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    const bool was_running = running_.exchange(false, std::memory_order_acq_rel);

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
        process_queued_commands();
        service_discrete_commands();
        realtime_cycle_callback();
        add_period_ns(next_period, rt_options_.rt_period_ns);
        clock_nanosleep(kClockToUse, TIMER_ABSTIME, &next_period, nullptr);
    }
}


/// @brief 异步发送控制命令（stop / restart / set_mode / setpoints）
/// @brief 进行通用性检查：如电机索引、SETPOINT类型命令的数据长度
/// @brief 调用 validate_command() 进行驱动特定检查，入命令队列

CommandSubmitResult MotorControllerBase::send_command(const ControlCommand& cmd)
{
    if (!cmd.payload_valid) {
        return CommandSubmitResult::INVALID_PAYLOAD;
    }

    if (cmd.motor_index < ControlCommand::kAllMotors ||
        cmd.motor_index >= static_cast<int>(motor_count_)) {
        return CommandSubmitResult::INVALID_COMMAND;
    }

    switch (cmd.kind) {
        case ControlCommandKind::DISCRETE:
            break;

        case ControlCommandKind::SETPOINT: {
            if (cmd.motor_index != ControlCommand::kAllMotors) {
                return CommandSubmitResult::INVALID_COMMAND;
            }

            std::size_t payload_size = 0;
            switch (cmd.setpoint_type) {
                case SetpointCommandType::POSITION_TARGETS:
                case SetpointCommandType::VELOCITY_TARGETS:
                case SetpointCommandType::TORQUE_TARGETS:
                    payload_size = cmd.setpoints.size();
                    break;
                case SetpointCommandType::IMPEDANCE_TARGETS:
                    payload_size = cmd.impedance_setpoints.size();
                    break;
                default:
                    return CommandSubmitResult::INVALID_COMMAND;
            }

            if (payload_size != motor_count_) {
                return CommandSubmitResult::INVALID_PAYLOAD;
            }
            break;
        }

        default:
            return CommandSubmitResult::INVALID_COMMAND;
    }

    const CommandSubmitResult driver_validation = validate_command(cmd);
    if (driver_validation != CommandSubmitResult::ACCEPTED) {
        return driver_validation;
    }

    if (!cmd_queue_.try_push(cmd)) {
        return CommandSubmitResult::QUEUE_FULL;
    }
    return CommandSubmitResult::ACCEPTED;
}


CommandSubmitResult MotorControllerBase::validate_command(const ControlCommand&) const
{
    return CommandSubmitResult::ACCEPTED;
}


std::vector<MotorStatusSnapshot> MotorControllerBase::get_status()
{
    return status_channel_.get_status();
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
        const ControlCommand* cmd = cmd_queue_.front();
        if (!cmd) {
            break;
        }
        ++processed;
        if (cmd->kind == ControlCommandKind::DISCRETE) {
            enqueue_discrete_command(*cmd);
        } else if (cmd->kind == ControlCommandKind::SETPOINT) {
            apply_setpoint_command_callback(*cmd);
        }
        cmd_queue_.pop_front();
    }
}

/* 通过控制命令中的 离散命令类型 和 模式 构建离散命令；并将其入对应电机的离散命令队列 */
void MotorControllerBase::enqueue_discrete_command(const ControlCommand& cmd)
{
    auto enqueue_one = [this, &cmd](int idx) {
        if (idx < 0 || idx >= static_cast<int>(motor_count_)) {
            return;
        }

        DiscreteCommand pending(cmd.discrete_type, cmd.mode);
        pending.phase = DiscretePhase::QUEUED;

        pending.enqueue_tick    = discrete_cmd_tick_;
        pending.next_retry_tick = discrete_cmd_tick_;
        pending.next_verify_tick = discrete_cmd_tick_;
        pending.deadline_tick = discrete_cmd_tick_ + kDiscreteTimeoutTicks;

        pending.cur_retry = 0;
        pending.max_retries = kDiscreteMaxRetries;

        pending.stable_success_cycles = 0;
        pending.fail_reason = DiscreteFailReason::NONE;

        if (!discrete_cmd_queues_[static_cast<std::size_t>(idx)].push_back(pending)) {
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
            queue.pop_front();
            continue;
        }

        if (cmd.phase == DiscretePhase::FAILED) {
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
                apply_discrete_command_callback(motor_index, cmd);
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

            const DiscreteCommandEvaluation evaluation = evaluate_discrete_command_callback(motor_index, cmd);
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

double MotorControllerBase::rad_to_deg(double rad)
{
    constexpr double kPi = 3.14159265358979323846;
    return rad * (180.0 / kPi);
}

bool MotorControllerBase::is_running() const
{
    return running_.load(std::memory_order_acquire);
}


std::vector<double> MotorControllerBase::get_joint_q_rad()
{
    const auto status = get_status();
    std::vector<double> q(status.size(), 0.0);
    for (std::size_t i = 0; i < status.size(); ++i) {
        q[i] = status[i].position_rad;
    }
    return q;
}


std::vector<double> MotorControllerBase::get_joint_vel_rad_s()
{
    const auto status = get_status();
    std::vector<double> dq(status.size(), 0.0);
    for (std::size_t i = 0; i < status.size(); ++i) {
        dq[i] = status[i].velocity_rad_s;
    }
    return dq;
}


std::vector<double> MotorControllerBase::get_joint_torque_percent()
{
    const auto status = get_status();
    std::vector<double> torque(status.size(), 0.0);
    for (std::size_t i = 0; i < status.size(); ++i) {
        torque[i] = status[i].torque_percent;
    }
    return torque;
}

} // namespace motor_base
