#include "robot/policy_command_worker.hpp"

#include "robot/action_processor.hpp"
#include "robot/robot_motor_session.hpp"
#include "tool/thread_runtime.hpp"
#include "tool/tool.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

namespace inference::robot_detail {

PolicyCommandWorker::PolicyCommandWorker(
    const RobotInterfaceConfig& config,
    RobotMotorSession& motor_session,
    const std::atomic<std::int64_t>& first_policy_inference_started_ns)
    : config_(config),
      motor_session_(motor_session),
      first_policy_inference_started_ns_(first_policy_inference_started_ns)
{
}

PolicyCommandWorker::~PolicyCommandWorker() {
    stop();
}

bool PolicyCommandWorker::start(
    ActionProcessor&    action_processor,
    std::array<double, motor_base::kMaxMotors> startup_hold_target_motor_rad)
{
    action_processor_ = &action_processor;
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
            [this] { loop(); }, thread_error)) {
        running_.store(false);
        std::cerr << "[PolicyCommandWorker] failed to start thread: "
                  << thread_error << "\n";
        return false;
    }

    return true;
}

void PolicyCommandWorker::request_stop() noexcept
{
    running_.store(false);
}

void PolicyCommandWorker::stop()
{
    request_stop();
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

bool PolicyCommandWorker::healthy(std::string& error) const
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

void PolicyCommandWorker::reset_channels() noexcept
{
    target_channel_.reset_empty();
    completed_record_channel_.reset_empty();
}

PolicyTargetFrame& PolicyCommandWorker::acquire_target_write_slot() noexcept
{
    return *target_channel_.acquire_write_slot();
}

std::int64_t PolicyCommandWorker::publish_target() noexcept
{
    auto& staged_target = acquire_target_write_slot();

    const std::int64_t published_at_ns = robot_base::monotonic_now_ns();
    staged_target.published_at_ns = published_at_ns;
    staged_target.inference_record.policy_valid_until_ns  = published_at_ns +
        robot_base::seconds_to_ns(config_.safety.policy_target_timeout_ms / 1000.0);

    // publish_written() 返回是否覆盖旧帧，不影响发布。
    target_channel_.publish_written();
    return published_at_ns;
}

bool PolicyCommandWorker::try_consume_completed_record(InferenceRecord& record) noexcept
{
    return completed_record_channel_.try_consume_latest(record);
}

std::int64_t PolicyCommandWorker::startup_policy_deadline_ns() const noexcept
{
    const std::int64_t started_ns =
        first_policy_inference_started_ns_.load(std::memory_order_acquire);
    return started_ns == 0 ? 0 : started_ns +
        robot_base::seconds_to_ns(config_.safety.policy_target_timeout_ms / 1000.0);
}


bool PolicyCommandWorker::policy_deadline_expired(
    std::int64_t deadline_ns, std::int64_t now_ns) noexcept
{
    return now_ns >= deadline_ns;
}


motor_base::CommandTiming PolicyCommandWorker::policy_command_timing(
    const PolicyTargetFrame& target,
    std::int64_t produced_at_ns,
    std::int64_t oldest_feedback_timestamp_ns) const noexcept
{
    motor_base::CommandTiming timing;
    timing.source_policy_seq = target.inference_record.policy_seq;
    timing.produced_at_ns    = produced_at_ns;
    timing.valid_until_ns    = std::min({
        target.inference_record.policy_valid_until_ns,
        produced_at_ns + robot_base::seconds_to_ns(
            config_.safety.control_command_timeout_ms / 1000.0),
        oldest_feedback_timestamp_ns + robot_base::seconds_to_ns(
            config_.sensor_guard.max_motor_sample_age_s)});
    return timing;
}


void PolicyCommandWorker::loop()
{
    const auto period = std::chrono::nanoseconds(
        std::max<std::int64_t>(
            1,
            static_cast<std::int64_t>(std::llround(config_.ankle_torque.filter_dt_s * 1'000'000'000.0))
         ));

    auto next_wake = std::chrono::steady_clock::now();

    PolicyTargetFrame current_target;
    bool has_current_target = false;

    std::array<motor_base::MotorStatusSnapshot,
               motor_base::kMaxMotorCommandSetpoints> motor_feedback;
    bool has_motor_feedback = false;
    ActionProcessor::FixedPolicyMotorCommand command;
    std::string error;
    std::uint64_t last_logged_policy_seq = 0;

    try {
        while (running_.load()) {
            next_wake += period;

            if (target_channel_.try_consume_latest(current_target)) {
                has_current_target = true;
            }

            if (!has_current_target) {
                const std::int64_t deadline_ns = startup_policy_deadline_ns();
                const std::int64_t now_ns = robot_base::monotonic_now_ns();
                // 首帧前复位命令也封顶首次正式推理的截止期。
                const bool refreshed = (deadline_ns == 0 || !policy_deadline_expired(deadline_ns, now_ns)) &&
                    motor_session_.apply_targets_rad(
                        startup_hold_target_motor_rad_, deadline_ns);
                if (!refreshed) {
                    const std::int64_t failed_now_ns = robot_base::monotonic_now_ns();
                    if (deadline_ns != 0 && policy_deadline_expired(deadline_ns, failed_now_ns)) {
                        fail("first policy target deadline expired: deadline_ns=" +
                            std::to_string(deadline_ns) + ", overdue_us=" +
                            std::to_string(robot_base::ns_to_us(failed_now_ns - deadline_ns)));
                    } else {
                        fail("failed to refresh startup reset pose");
                    }
                    break;
                }
            } else {
                const std::int64_t now_ns = robot_base::monotonic_now_ns();
                if (policy_deadline_expired(current_target.inference_record.policy_valid_until_ns, now_ns)) {
                    fail(policy_deadline_error(current_target, now_ns));
                    break;
                }

                if (motor_session_.try_consume_latest_status_command(motor_feedback)) {
                    has_motor_feedback = true;
                }
                if (!has_motor_feedback) {
                    fail("policy motor feedback is not initialized");
                    break;
                }

                const auto oldest_feedback = std::min_element(
                    motor_feedback.begin(),
                    motor_feedback.begin() + config_.motor.num_motors,
                    [](const auto& lhs, const auto& rhs) {
                        return lhs.host_timestamp_ns < rhs.host_timestamp_ns;
                    });
                const std::int64_t max_feedback_age_ns = robot_base::seconds_to_ns(
                    config_.sensor_guard.max_motor_sample_age_s);
                const auto fail_if_feedback_expired =
                    [this, oldest_feedback, max_feedback_age_ns](std::int64_t checked_at_ns) {
                        if (oldest_feedback->host_timestamp_ns > 0 &&
                            checked_at_ns < oldest_feedback->host_timestamp_ns + max_feedback_age_ns) {
                            return false;
                        }
                        fail(std::string(oldest_feedback->host_timestamp_ns <= 0
                                ? "motor feedback timestamp missing: motor_index="
                                : "motor feedback expired: motor_index=") +
                            std::to_string(oldest_feedback->motor_index) +
                            ", feedback_age_us=" + std::to_string(robot_base::ns_to_us(
                                checked_at_ns - oldest_feedback->host_timestamp_ns)) +
                            ", max_motor_age_us=" + std::to_string(
                                robot_base::ns_to_us(max_feedback_age_ns)));
                        return true;
                    };
                if (fail_if_feedback_expired(robot_base::monotonic_now_ns())) {
                    break;
                }

                error.clear();
                if (!action_processor_->build_policy_impedance_command(
                        current_target.inference_record.target_q_model_rad,
                        motor_feedback,
                        command,
                        error)) {
                    fail("failed to build policy impedance command: " + error);
                    break;
                }

                const std::int64_t produced_at_ns = robot_base::monotonic_now_ns();
                if (policy_deadline_expired(current_target.inference_record.policy_valid_until_ns, produced_at_ns)) {
                    fail(policy_deadline_error(current_target, produced_at_ns));
                    break;
                }
                if (fail_if_feedback_expired(produced_at_ns)) {
                    break;
                }
                const motor_base::CommandTiming timing =
                    policy_command_timing(current_target, produced_at_ns,
                                          oldest_feedback->host_timestamp_ns);

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
                    current_target.inference_record.policy_seq != last_logged_policy_seq) {
                    auto& completed_record = *completed_record_channel_.acquire_write_slot();
                    completed_record = current_target.inference_record;
                    completed_record.command_timestamp_ns   = produced_at_ns;
                    completed_record.command_valid_until_ns = timing.valid_until_ns;
                    completed_record.command_applied = true;
                    for (std::size_t i = 0; i < policy_observation::kDof; ++i) {
                        completed_record.target_pos_rad[i] =
                            command.setpoints[i].position_rad;
                        completed_record.target_effort_permille[i] =
                            command.setpoints[i].effort_ff;
                    }
                    // 发布给 recorder，供后续日志写入线程处理。
                    completed_record_channel_.publish_written();
                    last_logged_policy_seq = current_target.inference_record.policy_seq;
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


std::string PolicyCommandWorker::policy_deadline_error(
    const PolicyTargetFrame& target,
    std::int64_t now_ns) const
{
    std::ostringstream stream;
    stream << "policy target deadline expired: policy_seq="
           << target.inference_record.policy_seq
           << ", target_hold_age_us="
           << robot_base::ns_to_us(now_ns - target.published_at_ns)
           << ", obs_to_action_age_us=" << target.inference_record.obs_to_action_age_us
           << ", overdue_us="
           << robot_base::ns_to_us(now_ns - target.inference_record.policy_valid_until_ns);
    return stream.str();
}


}  // namespace inference::robot_detail
