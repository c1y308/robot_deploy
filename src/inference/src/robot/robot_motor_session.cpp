#include "robot/robot_motor_session.hpp"

#include "tool/tool.hpp"
#include "motor_base/motor_controller_base.hpp"
#include "driver/myact/myact_motor_controller.hpp"
#include "protocol/ethercat/ethercat_adapter_igh.hpp"

#include <array>
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <iostream>
#include <thread>
#include <utility>

namespace inference {

namespace {

const char* command_submit_status_name(motor_base::CommandSubmitStatus status)
{
    switch (status) {
        case motor_base::CommandSubmitStatus::ACCEPTED: return "ACCEPTED";
        case motor_base::CommandSubmitStatus::QUEUE_FULL: return "QUEUE_FULL";
        case motor_base::CommandSubmitStatus::INVALID_COMMAND: return "INVALID_COMMAND";
        case motor_base::CommandSubmitStatus::INVALID_PAYLOAD: return "INVALID_PAYLOAD";
    }
    return "UNKNOWN";
}

}  // namespace

RobotMotorSession::RobotMotorSession(const RobotInterfaceConfig& config)
    : config_(config.motor),
      motor_rt_options_(config.runtime.motor_rt),
      background_thread_options_(config.runtime.background),
      max_feedback_age_ns_(robot_base::seconds_to_ns(config.sensor_guard.max_motor_sample_age_s)) {}

RobotMotorSession::~RobotMotorSession() noexcept
{
    // The controller reuses an outstanding all-axis STOP request.
    if (shutdown(request_stop()) == ShutdownResult::RetryRequired) {
        std::fputs("[RobotMotorSession] Fatal: destructor shutdown failed; "
                   "STOP not confirmed. Terminating process.\n", stderr);
        std::fflush(stderr);
        std::terminate();
    }
}

std::unique_ptr<motor_base::MotorControllerBase>
RobotMotorSession::create_controller() const
{
    // 创建通信适配器
    auto adapter = std::make_shared<myactua::EthercatAdapterIGH>();
    // 准备驱动配置，再覆盖两类线程选项
    myactua::MyActMotorController::Options controller_options;
    controller_options.rt_thread_options = motor_rt_options_;
    controller_options.background_thread_options = background_thread_options_;
    // 创建并返回具体控制器
    return std::make_unique<myactua::MyActMotorController>(
        std::move(adapter), config_.num_motors, controller_options);
}


bool RobotMotorSession::initialize()
{
    // 首先检查是否允许初始化；消费者启动前清空两侧反馈缓存。
    if (initialized_.load()) {
        return true;
    }
    // A failed stop/initialization may still own an active RT controller.
    if (controller_) return false;

    feedback_for_policy_ = {};
    feedback_for_command_ = {};
    // 使用工厂函数创建电机控制器
    controller_ = create_controller();

    std::cout << "[RobotMotorSession] connecting...\n";

    if (!controller_->connect()) {
        std::cerr << "[RobotMotorSession] connect failed.\n";
        release_controller();
        return false;
    }
    // 等待全部电机及通信就绪
    if (!controller_->wait_all_motors_ready(config_.wait_all_motors_timeout_ms,
                                            config_.wait_all_motors_poll_ms)) {
        std::cerr << "[RobotMotorSession] Not all slaves became ready in timeout.\n";
        release_controller();
        return false;
    }

    if (config_.print_motors_info) {
        controller_->set_print_info(config_.print_motor_ids);
    } else {
        controller_->set_print_info({});
    }

    if (!controller_->start()) {
        std::cerr << "[RobotMotorSession] RT prerequisite failed: "
                  << "realtime scheduling is not active.\n";
        release_controller();
        return false;
    }

    rt_started_ = true;
    // RT 启动后失败时保留控制器，由调用方统一进入 shutdown()。
    if (!stop()) return false;

    // STOP invalidates earlier mode commands, so configure only after its ACK.
    for (int i = 0; i < config_.num_motors; ++i) {
        if (!submit_discrete_command(
                motor_base::ControlCommand::set_mode(config_.control_mode, i),
                "set_mode")) return false;
    }
    initialized_.store(true);

    return true;
}


ShutdownResult RobotMotorSession::shutdown(const motor_base::CommandSubmitResult& request)
{
    const bool stop_confirmed = !rt_started_ || wait_for_discrete_command(request, "stop");
    if (!stop_confirmed &&
        !(controller_ && controller_->communication_fault_latched() &&
          !controller_->communication_healthy())) {
        return ShutdownResult::RetryRequired;
    }
    release_controller();
    return stop_confirmed ? ShutdownResult::Confirmed
                          : ShutdownResult::ReleasedWithCommLoss;
}

void RobotMotorSession::release_controller()
{
    // 停止控制器线程
    if (controller_) controller_->shutdown();
    // 销毁控制器对象
    controller_.reset();
    // 重置生命周期标志
    rt_started_ = false;
    initialized_.store(false);
    // 消费者已停止，清空两侧精简反馈缓存。
    feedback_for_policy_ = {};
    feedback_for_command_ = {};
}

bool RobotMotorSession::check_health(std::string& error) const
{
    if (!initialized_.load() || !controller_) {
        error = "motors are not initialized";
        return false;
    }
    if (controller_->terminal_fault_latched()) {
        error = "motor controller terminal fault is latched";
        return false;
    }
    error.clear();
    return true;
}

motor_base::CommandSubmitResult RobotMotorSession::request_stop()
{
    if (!controller_) {
        return {motor_base::CommandSubmitStatus::INVALID_COMMAND, std::nullopt};
    }
    return controller_->send_discrete_command(
        motor_base::ControlCommand::stop());
}

bool RobotMotorSession::stop()
{
    return wait_for_discrete_command(request_stop(), "stop");
}

// 发送 restart 命令
bool RobotMotorSession::restart()
{
    if (!initialized_.load() || !controller_) {
        return false;
    }
    if (!controller_->is_realtime_scheduling_ready()) {
        std::cerr << "[RobotMotorSession] restart rejected: "
                  << "realtime scheduling is not active.\n";
        return false;
    }
    return wait_for_discrete_command(
        controller_->send_discrete_command(motor_base::ControlCommand::restart()),
        "restart");
}

bool RobotMotorSession::wait_for_discrete_command(
    const motor_base::CommandSubmitResult& request, const char* context)
{
    // 检查命令是否提交成功
    if (request.status != motor_base::CommandSubmitStatus::ACCEPTED) {
        std::cerr << "[RobotMotorSession] " << context << " command rejected: "
                  << command_submit_status_name(request.status) << "\n";
        return false;
    }
    // 输出缺少命令 ID
    if (!request.command_id) {
        std::cerr << "[RobotMotorSession] " << context << " command missing command_id\n";
        return false;
    }
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(config_.discrete_command_completion_timeout_ms);
    for (;;) {
        const auto result = controller_->get_discrete_command_result(*request.command_id);
        switch (result) {
            case motor_base::DiscreteCommandResult::SUCCEEDED:
                return true;
            case motor_base::DiscreteCommandResult::FAILED:
                std::cerr << "[RobotMotorSession] " << context << " command failed\n";
                return false;
            case motor_base::DiscreteCommandResult::UNKNOWN:
                std::cerr << "[RobotMotorSession] " << context << " command result unknown\n";
                return false;
            case motor_base::DiscreteCommandResult::PENDING:
                break;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            std::cerr << "[RobotMotorSession] " << context
                      << " timed out waiting for command completion\n";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool RobotMotorSession::submit_discrete_command(
    const motor_base::ControlCommand& command, const char* context)
{
    const motor_base::CommandSubmitResult result =
        controller_->send_discrete_command(command);
    if (result.status == motor_base::CommandSubmitStatus::ACCEPTED) {
        return true;
    }

    std::cerr << "[RobotMotorSession] " << context
              << " command rejected: "
              << command_submit_status_name(result.status) << "\n";
    return false;
}

// 仅构造 POSITION 模式的位置目标。
bool RobotMotorSession::submit_position_targets_rad(
    const std::array<double, motor_base::kMotorNum>& target_motor_rad,
    const motor_base::CommandTiming& timing)
{
    if (!initialized_.load() || !controller_) {
        return false;
    }

    if (config_.control_mode != motor_base::MotorControlMode::POSITION) {
        std::cerr << "[RobotMotorSession] submit_position_targets_rad requires position mode\n";
        return false;
    }
    // 调用 ControlCommand 的工厂函数构造并写入时间信息
    auto command = motor_base::ControlCommand::make_position_targets_rad(
        target_motor_rad.data(), target_motor_rad.size());
    command.timing = timing;

    const motor_base::CommandSubmitResult result =
        controller_->send_continuous_command(command);
    if (result.status != motor_base::CommandSubmitStatus::ACCEPTED) {
        std::cerr << "[RobotMotorSession] submit_position_targets_rad command rejected: "
                  << command_submit_status_name(result.status) << "\n";
        return false;
    }
    return true;
}

bool RobotMotorSession::submit_impedance_setpoints(
    const std::array<motor_base::ImpedanceSetpoint, motor_base::kMotorNum>& setpoints,
    const motor_base::CommandTiming& timing)
{
    if (!initialized_.load() || !controller_) {
        return false;
    }
    if (config_.control_mode != motor_base::MotorControlMode::IMPEDANCE) {
        std::cerr << "[RobotMotorSession] submit_impedance_setpoints requires impedance mode\n";
        return false;
    }
    // 通过工厂函数构造并写入时间信息
    motor_base::ControlCommand command =
        motor_base::ControlCommand::make_impedance_targets(
            setpoints.data(),
            static_cast<std::size_t>(config_.num_motors));
    command.timing = timing;

    // 发送命令
    const motor_base::CommandSubmitResult result =
        controller_->send_continuous_command(command);
    if (result.status == motor_base::CommandSubmitStatus::ACCEPTED) {
        return true;
    }

    std::cerr << "[RobotMotorSession] submit_impedance_setpoints command rejected: "
              << command_submit_status_name(result.status) << "\n";
    return false;
}



bool RobotMotorSession::refresh_feedback_for_command(std::string& error)
{
    return refresh_feedback(feedback_for_command_, FeedbackChannel::Command, error);
}

bool RobotMotorSession::refresh_feedback_for_policy(std::string& error)
{
    return refresh_feedback(feedback_for_policy_, FeedbackChannel::Policy, error);
}

bool RobotMotorSession::refresh_feedback(
    FeedbackCache& cache, FeedbackChannel channel, std::string& error)
{
    if (!initialized_.load() || !controller_) {
        error = "policy motor feedback is not initialized";
        return false;
    }

    // 获取原始逐轴反馈信息
    std::array<motor_base::MotorStatusSnapshot, motor_base::kMotorNum> feedback{};
    const bool received = channel == FeedbackChannel::Policy
        ? controller_->try_consume_latest_feedback_for_policy(feedback)
        : controller_->try_consume_latest_feedback_for_command(feedback);


    // 处理为 MotorStateSnapshot，推理层需要的信息
    if (received) {
        MotorStateSnapshot snapshot;
        snapshot.timestamp_ns    = feedback[0].host_timestamp_ns;
        cache.oldest_motor_index = feedback[0].motor_index;
        for (int i = 0; i < config_.num_motors; ++i) {
            const auto& motor = feedback[static_cast<std::size_t>(i)];
            if (motor.host_timestamp_ns < snapshot.timestamp_ns) {
                snapshot.timestamp_ns    = motor.host_timestamp_ns;
                cache.oldest_motor_index = motor.motor_index;
            }
            snapshot.position_rad[i]   = motor.position_rad;
            snapshot.velocity_rad_s[i] = motor.velocity_rad_s;
            snapshot.torque_percent[i] = motor.torque_percent;
            snapshot.comm_ok[i] = motor.comm_ok;
            snapshot.enabled[i] = motor.enabled;
        }
        cache.snapshot = snapshot;
        cache.has_feedback = true;
    }
    if (!cache.has_feedback) {
        error = "policy motor feedback is not initialized";
        return false;
    }
    error.clear();
    return true;
}

const MotorStateSnapshot& RobotMotorSession::get_feedback_for_policy() const noexcept
{
    return feedback_for_policy_.snapshot;
}


const MotorStateSnapshot& RobotMotorSession::get_feedback_for_command() const noexcept
{
    return feedback_for_command_.snapshot;
}

bool RobotMotorSession::validate_feedback_timing_for_command(
    std::int64_t  now_ns,
    std::int64_t& feedback_valid_until_ns,
    std::string& error) const
{
    if (!feedback_for_command_.has_feedback) {
        error = "policy motor feedback is not initialized";
        return false;
    }

    const auto timestamp_ns = feedback_for_command_.snapshot.timestamp_ns;
    const std::int64_t deadline_ns =
        timestamp_ns + max_feedback_age_ns_;
    if (timestamp_ns > 0 && now_ns < deadline_ns) {
        feedback_valid_until_ns = deadline_ns;
        error.clear();
        return true;
    }

    error = std::string(timestamp_ns <= 0
                ? "motor feedback timestamp missing: motor_index="
                : "motor feedback expired: motor_index=") +
        std::to_string(feedback_for_command_.oldest_motor_index) +
        ", feedback_age_us=" + std::to_string(robot_base::ns_to_us(
            now_ns - timestamp_ns)) +
        ", max_motor_age_us=" + std::to_string(robot_base::ns_to_us(max_feedback_age_ns_));
    return false;
}


}  // namespace inference
