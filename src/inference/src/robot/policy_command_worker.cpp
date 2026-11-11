#include "robot/policy_command_worker.hpp"

#include "robot/action_processor.hpp"
#include "robot/robot_motor_session.hpp"
#include "thread/thread_runtime.hpp"
#include "tool/tool.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

namespace inference {

PolicyCommandWorker::PolicyCommandWorker(
    const RobotInterfaceConfig&      config,
    RobotMotorSession&               motor_session,
    ActionProcessor&                 action_processor,
    const std::atomic<std::int64_t>& first_policy_inference_started_ns)
    : config_(config),
      motor_session_(motor_session),
      action_processor_(action_processor),

      /* 设置保护配置 */
      target_timeout_ns_(robot_base::seconds_to_ns(
          config.safety.policy_target_timeout_ms / 1000.0)),
      command_timeout_ns_(robot_base::seconds_to_ns(
          config.safety.control_command_timeout_ms / 1000.0)),

      first_policy_inference_started_ns_(first_policy_inference_started_ns)
{
}

PolicyCommandWorker::~PolicyCommandWorker() {
    stop();
}

bool PolicyCommandWorker::start(
    std::array<double, motor_base::kMotorNum> startup_hold_target_motor_rad)
{
    // 线程启动之前电机需要保持的角度
    startup_hold_target_motor_rad_ = std::move(startup_hold_target_motor_rad);
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        error_.clear();
        failed_.store(false);
    }

    // 在线程接管前刷新一次复位命令，为首次 worker 调度保留完整有效期。
    if (!motor_session_.apply_targets_rad(startup_hold_target_motor_rad_)) {
        std::cerr << "[PolicyCommandWorker] start rejected: "
                     "failed to refresh startup reset pose\n";
        return false;
    }

    running_.store(true);

    std::string thread_error;
    if (!robot_base::start_configured_thread(
            worker_thread_, "policy_cmd", config_.runtime.policy_command,
            [this] { run(); }, thread_error)) {
        running_.store(false);
        std::cerr << "[PolicyCommandWorker] failed to start thread: "
                  << thread_error << "\n";
        return false;
    }

    return true;
}


void PolicyCommandWorker::stop()
{
    request_stop();
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

void PolicyCommandWorker::request_stop() noexcept
{
    running_.store(false);
}


bool PolicyCommandWorker::check_health(std::string& error) const
{
    if (failed_.load()) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        error = error_;
        return false;
    }
    if (!running_.load()) {
        error = "policy command worker is not running";
        return false;
    }

    error.clear();
    return true;
}

// 接收策略目标及关联日志，设置有效期，再交给 policy_cmd 线程消费。
std::int64_t PolicyCommandWorker::publish_target(
    const PolicyTargetFrame& target,
    const InferenceRecord*   record) noexcept
{
    // // 从 channel 中获取一个写槽
    auto& frame = *target_channel_.acquire_write_slot();
    frame.target = target;
    if (config_.recorder.enabled) {
        frame.record = *record;
    }
    // 设置发布时间和截止期；worker 的 run() 循环会检查截止期，过期就触发故障停止。
    const std::int64_t published_at_ns = robot_base::monotonic_now_ns();
    frame.target.published_at_ns       = published_at_ns;
    frame.target.policy_valid_until_ns = published_at_ns + target_timeout_ns_;

    // publish_written() 返回是否覆盖旧帧，不影响发布。
    if (config_.recorder.enabled) {
        frame.record.policy_valid_until_ns = frame.target.policy_valid_until_ns;
    }
    target_channel_.publish_written();
    return published_at_ns;
}

bool PolicyCommandWorker::try_consume_target(PolicyTargetWithRecordFrame& frame) noexcept
{
    return target_channel_.try_consume_latest(frame);
}


bool PolicyCommandWorker::try_consume_completed_record(InferenceRecord& record) noexcept
{
    return record_channel_.try_consume_latest(record);
}

motor_base::CommandTiming PolicyCommandWorker::build_command_timing(
    const PolicyTargetFrame& target,
    std::int64_t             produced_at_ns,
    std::int64_t feedback_valid_until_ns) const noexcept
{
    motor_base::CommandTiming timing;
    timing.source_policy_seq = target.policy_seq;
    timing.produced_at_ns    = produced_at_ns;
    timing.valid_until_ns    = std::min({
        target.policy_valid_until_ns,
        produced_at_ns + command_timeout_ns_,
        feedback_valid_until_ns});
    return timing;
}


bool PolicyCommandWorker::refresh_startup_hold()
{
    // 获取首次正式推理的开始时间
    const std::int64_t started_ns  = first_policy_inference_started_ns_.load(std::memory_order_acquire);
    const std::int64_t deadline_ns = started_ns == 0 ? 0 : started_ns + target_timeout_ns_;
    const std::int64_t now_ns      = robot_base::monotonic_now_ns();
    // 首帧前复位命令也封顶首次正式推理的截止期。
    const bool refreshed = (deadline_ns == 0 || now_ns < deadline_ns) &&
        motor_session_.apply_targets_rad(
            startup_hold_target_motor_rad_, deadline_ns);
    if (!refreshed) {
        const std::int64_t failed_now_ns = robot_base::monotonic_now_ns();
        if (deadline_ns != 0 && failed_now_ns >= deadline_ns) {
            fail("first policy target deadline expired: deadline_ns=" +
                std::to_string(deadline_ns) + ", overdue_us=" +
                std::to_string(robot_base::ns_to_us(failed_now_ns - deadline_ns)));
        } else {
            fail("failed to refresh startup reset pose");
        }
        return false;
    }
    return true;
}

void PolicyCommandWorker::complete_and_publish_record(
    const InferenceRecord&    record,
    const PolicyMotorCommand& command,
    const motor_base::CommandTiming& timing)
{
    auto& completed_record = *record_channel_.acquire_write_slot();
    completed_record = record;
    completed_record.command_timestamp_ns   = timing.produced_at_ns;
    completed_record.command_valid_until_ns = timing.valid_until_ns;
    completed_record.command_applied = true;
    for (std::size_t i = 0; i < motor_base::kMotorNum; ++i) {
        completed_record.target_pos_rad[i] =
            command.setpoints[i].position_rad;
        completed_record.target_effort_permille[i] =
            command.setpoints[i].effort_ff;
    }
    // 发布给 recorder，供后续日志写入线程处理。
    record_channel_.publish_written();
}

void PolicyCommandWorker::run()
{
    // 控制周期至少 1 ns
    const auto period = std::chrono::nanoseconds(
        std::max<std::int64_t>(
            1,
            robot_base::seconds_to_ns(config_.ankle.filter_dt_s)));

    auto next_wake = std::chrono::steady_clock::now();

    // 保存当前使用的策略目标及关联记录
    PolicyTargetWithRecordFrame current_frame;
    const auto& current_target = current_frame.target;
    bool has_current_target = false;

    PolicyMotorCommand command;
    std::string error;
    std::uint64_t last_recorded_policy_seq = 0;

    try {
        while (running_.load()) {
            next_wake += period;

            if (try_consume_target(current_frame)) {
                has_current_target = true;
            }

            if (!has_current_target) {
                if (!refresh_startup_hold()) {
                    break;
                }
            }
            else {
                const std::int64_t now_ns = robot_base::monotonic_now_ns();
                if (now_ns >= current_target.policy_valid_until_ns) {
                    fail(format_target_timeout_error(current_target, now_ns));
                    break;
                }

                if (!motor_session_.refresh_command_feedback(error)) {
                    fail(error);
                    break;
                }

                std::int64_t feedback_valid_until_ns = 0;
                if (!motor_session_.validate_motor_feedback_timing(
                        robot_base::monotonic_now_ns(), feedback_valid_until_ns, error)) {
                    fail(error);
                    break;
                }
                const auto& motor_feedback = motor_session_.command_feedback();

                error.clear();
                if (!action_processor_.build_policy_motor_command(
                        current_target.target_model_rad,    // 模型关节顺序的目标角度
                        motor_feedback,                                       // 物理电机反馈 motor_feedback
                        command,
                        error)) {
                    fail("failed to build policy impedance command: " + error);
                    break;
                }
                // 计算需要时间，所以先前有效的目标和反馈，到这里可能已经过期。
                // 两次检查分别保证：计算前：不使用已经过期的数据。计算后：不提交计算期间已经失效的结果。
                const std::int64_t produced_at_ns = robot_base::monotonic_now_ns();
                if (produced_at_ns >= current_target.policy_valid_until_ns) {
                    fail(format_target_timeout_error(current_target, produced_at_ns));
                    break;
                }
                if (!motor_session_.validate_motor_feedback_timing(
                        produced_at_ns, feedback_valid_until_ns, error)) {
                    fail(error);
                    break;
                }
                // 构造命令时间信息
                const motor_base::CommandTiming timing =
                    build_command_timing(current_target, produced_at_ns,
                                         feedback_valid_until_ns);

                const bool applied =
                    motor_session_.apply_impedance_setpoints_realtime(
                        command.setpoints,
                        command.setpoint_count,
                        timing);

                if (!applied) {
                    fail("failed to apply policy impedance command");
                    break;
                }
                // 继续构建日志记录
                if (config_.recorder.enabled &&
                    current_target.policy_seq != last_recorded_policy_seq) {
                    complete_and_publish_record(current_frame.record, command, timing);
                    last_recorded_policy_seq = current_target.policy_seq;
                }
            }

            std::this_thread::sleep_until(next_wake);
            const auto now = std::chrono::steady_clock::now();
            if (now > next_wake + period) {
                next_wake = now;
            }
        }
    } catch (const std::exception& error) {
        fail(std::string("policy command worker exception: ") + error.what());
    }
}


void PolicyCommandWorker::fail(std::string message)
{
    motor_session_.request_stop();
    request_stop();
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        error_ = message;
        failed_.store(true);
    }

    std::cerr << "[PolicyCommandWorker] failed: " << message << "\n";
}


std::string PolicyCommandWorker::format_target_timeout_error(
    const PolicyTargetFrame& target,
    std::int64_t now_ns) const
{
    std::ostringstream stream;
    stream << "policy target deadline expired: policy_seq="
           << target.policy_seq
           << ", target_hold_age_us="
           << robot_base::ns_to_us(now_ns - target.published_at_ns)
           << ", obs_to_action_age_us=" << target.obs_to_action_age_us
           << ", overdue_us="
           << robot_base::ns_to_us(now_ns - target.policy_valid_until_ns);
    return stream.str();
}


}  // namespace inference
