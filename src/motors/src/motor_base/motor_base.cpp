#include "motor_base/MotorControllerBase.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <pthread.h>
#include <sched.h>
#include <time.h>

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
      cmd_queue_(rt_options_.command_queue_capacity)
{
    rt_options_.max_commands_per_cycle =
        std::max<std::size_t>(1, rt_options_.max_commands_per_cycle);
    rt_options_.rt_period_ns = std::max<long>(1, rt_options_.rt_period_ns);

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
}

void MotorControllerBase::start()
{
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (running_.load(std::memory_order_acquire)) {
        return;
    }

    if (!on_realtime_start()) {
        return;
    }

    running_.store(true, std::memory_order_release);
    try {
        rt_thread_ = std::thread(&MotorControllerBase::rt_thread_func, this);
    } catch (...) {
        running_.store(false, std::memory_order_release);
        on_realtime_stop();
        throw;
    }

    set_realtime_priority(rt_thread_, rt_options_.rt_priority);
}

void MotorControllerBase::shutdown()
{
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    const bool was_running = running_.exchange(false, std::memory_order_acq_rel);

    if (rt_thread_.joinable()) {
        rt_thread_.join();
    }

    if (was_running) {
        on_realtime_stop();
    }
}

bool MotorControllerBase::is_running() const
{
    return running_.load(std::memory_order_acquire);
}

CommandSubmitResult MotorControllerBase::send_command(const ControlCommand& cmd)
{
    const CommandSubmitResult validation = validate_command(cmd);
    if (validation != CommandSubmitResult::ACCEPTED) {
        return validation;
    }

    if (!cmd_queue_.try_push(cmd)) {
        return CommandSubmitResult::QUEUE_FULL;
    }
    return CommandSubmitResult::ACCEPTED;
}

void MotorControllerBase::process_queued_commands_rt()
{
    ControlCommand cmd;
    std::size_t processed = 0;
    while (processed < rt_options_.max_commands_per_cycle &&
           cmd_queue_.try_pop_rt(cmd)) {
        ++processed;
        if (cmd.kind == ControlCommandKind::DISCRETE) {
            enqueue_discrete_command_rt(cmd);
        } else if (cmd.kind == ControlCommandKind::SETPOINT) {
            apply_setpoint_command_rt(cmd);
        }
    }
}

void MotorControllerBase::advance_discrete_command_tick_rt()
{
    ++discrete_cmd_tick_;
}

void MotorControllerBase::service_discrete_commands_rt()
{
    for (std::size_t i = 0; i < discrete_cmd_queues_.size(); ++i) {
        auto& queue = discrete_cmd_queues_[i];
        if (queue.empty()) {
            continue;
        }

        auto& cmd = queue.front_rt();
        const int motor_index = static_cast<int>(i);

        if (cmd.phase == DiscretePhase::DONE) {
            queue.pop_front_rt();
            continue;
        }
        if (cmd.phase == DiscretePhase::FAILED) {
            on_discrete_command_failed_rt(motor_index, cmd, cmd.fail_reason);
            queue.pop_front_rt();
            continue;
        }

        if (discrete_cmd_tick_ > cmd.deadline_tick) {
            cmd.phase = DiscretePhase::FAILED;
            cmd.fail_reason = kDiscreteFailTimeout;
            continue;
        }

        if (!is_motor_comm_ok_rt(motor_index)) {
            cmd.stable_success_cycles = 0;
            continue;
        }

        if (is_motor_faulted_rt(motor_index)) {
            cmd.phase = DiscretePhase::FAILED;
            cmd.fail_reason = kDiscreteFailFault;
            continue;
        }

        if (cmd.phase == DiscretePhase::QUEUED) {
            cmd.phase = DiscretePhase::APPLY_PENDING;
        }

        if (cmd.phase == DiscretePhase::APPLY_PENDING) {
            if (cmd.cur_retry >= cmd.max_retries) {
                cmd.phase = DiscretePhase::FAILED;
                cmd.fail_reason = kDiscreteFailMaxRetry;
                continue;
            }
            if (discrete_cmd_tick_ >= cmd.next_retry_tick) {
                apply_discrete_command_rt(motor_index, cmd);
                cmd.cur_retry += 1;
                cmd.next_retry_tick = discrete_cmd_tick_ + kDiscreteRetryTicks;
                cmd.next_verify_tick =
                    discrete_cmd_tick_ + kDiscreteVerifyIntervalTicks;
                cmd.stable_success_cycles = 0;
                cmd.phase = DiscretePhase::VERIFYING;
            }
            continue;
        }

        if (cmd.phase == DiscretePhase::VERIFYING) {
            if (discrete_cmd_tick_ < cmd.next_verify_tick) {
                continue;
            }

            if (is_discrete_command_satisfied_rt(motor_index, cmd)) {
                cmd.stable_success_cycles += 1;
                if (cmd.stable_success_cycles >= kDiscreteSuccessStableTicks) {
                    cmd.phase = DiscretePhase::DONE;
                    queue.pop_front_rt();
                } else {
                    cmd.next_verify_tick =
                        discrete_cmd_tick_ + kDiscreteVerifyIntervalTicks;
                }
            } else {
                cmd.stable_success_cycles = 0;
                if (discrete_cmd_tick_ >= cmd.next_retry_tick) {
                    cmd.phase = DiscretePhase::APPLY_PENDING;
                }
            }
        }
    }
}

void MotorControllerBase::on_discrete_command_failed_rt(
    int,
    const DiscreteCommand&,
    int)
{
}

void MotorControllerBase::on_discrete_queue_full_rt(
    int,
    const ControlCommand&)
{
}

void MotorControllerBase::rt_thread_func()
{
    timespec next_period{};
    clock_gettime(kClockToUse, &next_period);

    while (running_.load(std::memory_order_acquire)) {
        rt_cycle();
        add_period_ns(next_period, rt_options_.rt_period_ns);
        clock_nanosleep(kClockToUse, TIMER_ABSTIME, &next_period, nullptr);
    }
}

void MotorControllerBase::enqueue_discrete_command_rt(const ControlCommand& cmd)
{
    auto enqueue_one = [this, &cmd](int idx) {
        if (idx < 0 || idx >= static_cast<int>(motor_count_)) {
            return;
        }

        DiscreteCommand pending(cmd.discrete_type, cmd.mode);
        pending.phase = DiscretePhase::QUEUED;
        pending.enqueue_tick = discrete_cmd_tick_;
        pending.next_retry_tick = discrete_cmd_tick_;
        pending.next_verify_tick = discrete_cmd_tick_;
        pending.deadline_tick = discrete_cmd_tick_ + kDiscreteTimeoutTicks;
        pending.cur_retry = 0;
        pending.max_retries = kDiscreteMaxRetries;
        pending.stable_success_cycles = 0;
        pending.fail_reason = kDiscreteFailNone;

        if (!discrete_cmd_queues_[static_cast<std::size_t>(idx)].push_back_rt(
                pending)) {
            on_discrete_queue_full_rt(idx, cmd);
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

double MotorControllerBase::rad_to_deg(double rad)
{
    constexpr double kPi = 3.14159265358979323846;
    return rad * (180.0 / kPi);
}

} // namespace motor_base
