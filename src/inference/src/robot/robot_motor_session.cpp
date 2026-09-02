#include "robot/robot_motor_session.hpp"

#include "tool/tool.hpp"
#include "driver/myact/myact_motor_controller.hpp"
#include "protocol/ethercat/ethercat_adapter_igh.hpp"
#include "motor_base/motor_controller_base.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <thread>
#include <utility>

namespace inference {

namespace {

bool finite_impedance_setpoints(
    const std::vector<motor_base::ImpedanceSetpoint>& setpoints)
{
    return std::all_of(setpoints.begin(), setpoints.end(), [](const auto& value) {
        return std::isfinite(value.position_rad) &&
               std::isfinite(value.velocity_rad_s) &&
               std::isfinite(value.effort_ff) &&
               std::isfinite(value.kp) &&
               std::isfinite(value.kd);
    });
}

bool is_mit_mode(motor_base::MotorControlMode mode)
{
    return mode == motor_base::MotorControlMode::IMPEDANCE;
}

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

/* 把一段 MotorStatusSnapshot 填入 inference 层的电机状态快照 */
template <typename Iterator>
void fill_motor_snapshot_from_range(MotorStateSnapshot& snapshot,
                                     Iterator begin,
                                     Iterator end)
{
    const std::size_t count = static_cast<std::size_t>(end - begin);
    snapshot.timestamp_ns = count > 0 ? begin->host_timestamp_ns : 0;
    snapshot.position_rad.reserve(count);
    snapshot.velocity_rad_s.reserve(count);
    snapshot.torque_percent.reserve(count);
    snapshot.comm_ok.reserve(count);
    snapshot.enabled.reserve(count);
    snapshot.faulted.reserve(count);

    for (auto it = begin; it != end; ++it) {
        const auto& motor = *it;
        snapshot.position_rad.push_back(motor.position_rad);
        snapshot.velocity_rad_s.push_back(motor.velocity_rad_s);
        snapshot.torque_percent.push_back(motor.torque_percent);
        snapshot.comm_ok.push_back(motor.comm_ok ? 1U : 0U);
        snapshot.enabled.push_back(motor.enabled ? 1U : 0U);
        snapshot.faulted.push_back(motor.faulted ? 1U : 0U);
    }
}

}  // namespace

RobotMotorSession::RobotMotorSession(MotorConfig config)
    : config_(std::move(config)) {}

RobotMotorSession::~RobotMotorSession()
{
    deinitialize();
}


bool RobotMotorSession::initialize()
{
    if (initialized_.load()) {
        return true;
    }

    adapter_    = std::make_shared<myactua::EthercatAdapterIGH>();
    controller_ = std::make_unique<myactua::MyActMotorController>(adapter_, config_.num_motors);

    std::cout << "[RobotMotorSession] Connecting EtherCAT on "
              << config_.ethercat_ifname << "...\n";

    if (!controller_->connect(config_.ethercat_ifname.c_str())) {
        std::cerr << "[RobotMotorSession] EtherCAT connect failed.\n";
        controller_.reset();
        adapter_.reset();
        return false;
    }

    if (!controller_->wait_all_motors_ready(config_.wait_all_motors_timeout_ms,
                                            config_.wait_all_motors_poll_ms)) {
        std::cerr << "[RobotMotorSession] Not all slaves became ready in timeout.\n";
        controller_.reset();
        adapter_.reset();
        return false;
    }

    for (int i = 0; i < config_.num_motors; ++i) {
        if (!submit_command(
                motor_base::ControlCommand::set_mode(config_.control_mode, i),
                "set_mode")) {
            controller_.reset();
            adapter_.reset();
            return false;
        }
    }

    if (config_.print_motors_info) {
        controller_->set_print_info(config_.print_motor_ids);
    } else {
        controller_->set_print_info({});
    }

    if (!controller_->start()) {
        std::cerr << "[RobotMotorSession] RT prerequisite failed: "
                  << "realtime scheduling is not active.\n";
        controller_->shutdown();
        controller_.reset();
        adapter_.reset();
        return false;
    }

    if (!submit_command(motor_base::ControlCommand::stop(), "initial_stop")) {
        controller_.reset();
        adapter_.reset();
        return false;
    }
    initialized_.store(true);
    motion_enabled_.store(false);

    return true;
}


void RobotMotorSession::deinitialize()
{
    if (initialized_.load() && controller_) {
        stop(-1);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        controller_->shutdown();
    }

    controller_.reset();
    adapter_.reset();
    initialized_.store(false);
    motion_enabled_.store(false);
}


bool RobotMotorSession::stop(int motor_index)
{
    if (!initialized_.load() || !controller_) {
        return false;
    }
    if (motor_index >= config_.num_motors) {
        std::cerr << "[RobotMotorSession] stop invalid motor_index="
                  << motor_index << "\n";
        return false;
    }

    if (!submit_command(motor_base::ControlCommand::stop(motor_index), "stop")) {
        return false;
    }
    if (motor_index < 0) {
        motion_enabled_.store(false);
    }
    return true;
}


bool RobotMotorSession::restart(int motor_index)
{
    if (!initialized_.load() || !controller_) {
        return false;
    }
    if (motor_index >= config_.num_motors) {
        std::cerr << "[RobotMotorSession] restart invalid motor_index="
                  << motor_index << "\n";
        return false;
    }
    if (!controller_->is_realtime_scheduling_ready()) {
        std::cerr << "[RobotMotorSession] restart rejected: "
                  << "realtime scheduling is not active.\n";
        return false;
    }

    const motor_base::CommandSubmitResult submit_result =
        controller_->send_command(motor_base::ControlCommand::restart(motor_index));
    if (submit_result.status != motor_base::CommandSubmitStatus::ACCEPTED) {
        std::cerr << "[RobotMotorSession] restart command rejected: "
                  << command_submit_status_name(submit_result.status) << "\n";
        motion_enabled_.store(false);
        return false;
    }

    if (!submit_result.command_id.has_value()) {
        std::cerr << "[RobotMotorSession] restart command missing command_id\n";
        motion_enabled_.store(false);
        return false;
    }

    const motor_base::CommandId command_id = *submit_result.command_id;
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(config_.control_ready_timeout_ms);

    while (true) {
        const motor_base::DiscreteCommandResult command_result =
            controller_->get_discrete_command_result(command_id);
        switch (command_result) {
            case motor_base::DiscreteCommandResult::SUCCEEDED:
                if (motor_index < 0 || config_.num_motors == 1) {
                    motion_enabled_.store(true);
                }
                return true;

            case motor_base::DiscreteCommandResult::FAILED:
                std::cerr << "[RobotMotorSession] restart command failed\n";
                motion_enabled_.store(false);
                return false;

            case motor_base::DiscreteCommandResult::UNKNOWN:
                std::cerr << "[RobotMotorSession] restart command result unknown\n";
                motion_enabled_.store(false);
                return false;

            case motor_base::DiscreteCommandResult::PENDING:
                break;
        }

        if (std::chrono::steady_clock::now() >= deadline) {
            std::cerr << "[RobotMotorSession] restart timed out waiting for control_ready\n";
            motion_enabled_.store(false);
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}


bool RobotMotorSession::apply_targets_rad(const std::vector<double>& target_motor_rad)
{
    if (!initialized_.load() || !controller_) {
        return false;
    }
    if (!motion_enabled_.load()) {
        std::cerr << "[RobotMotorSession] apply_targets_rad rejected: motors are stopped. "
                  << "Call restart(-1) first.\n";
        return false;
    }
    if (static_cast<int>(target_motor_rad.size()) != config_.num_motors) {
        std::cerr << "[RobotMotorSession] apply_targets_rad rejected: target size mismatch\n";
        return false;
    }

    if (is_mit_mode(config_.control_mode)) {
        std::vector<motor_base::ImpedanceSetpoint> impedance_setpoints(config_.num_motors);
        for (int i = 0; i < config_.num_motors; ++i) {
            impedance_setpoints[i] = motor_base::ImpedanceSetpoint(target_motor_rad[i],
                                                                   0.0,
                                                                   0.0,
                                                                   config_.mit_kp[i],
                                                                   config_.mit_kd[i]);
        }
        return apply_impedance_setpoints(impedance_setpoints);
    }

    if (config_.control_mode != motor_base::MotorControlMode::POSITION) {
        std::cerr << "[RobotMotorSession] apply_targets_rad supports only impedance or position mode\n";
        return false;
    }

    if (!submit_command(
            motor_base::ControlCommand::set_position_targets_rad(target_motor_rad),
            "apply_targets_rad_position")) {
        return false;
    }
    return true;
}

bool RobotMotorSession::apply_impedance_setpoints(
    const std::vector<motor_base::ImpedanceSetpoint>& setpoints)
{
    if (!initialized_.load() || !controller_) {
        return false;
    }
    if (!motion_enabled_.load()) {
        std::cerr << "[RobotMotorSession] apply_impedance_setpoints rejected: motors are stopped. "
                  << "Call restart(-1) first.\n";
        return false;
    }
    if (!is_mit_mode(config_.control_mode)) {
        std::cerr << "[RobotMotorSession] apply_impedance_setpoints supports only impedance mode\n";
        return false;
    }
    if (static_cast<int>(setpoints.size()) != config_.num_motors) {
        std::cerr << "[RobotMotorSession] apply_impedance_setpoints rejected: target size mismatch\n";
        return false;
    }
    if (!finite_impedance_setpoints(setpoints)) {
        std::cerr << "[RobotMotorSession] apply_impedance_setpoints rejected: non-finite setpoint\n";
        return false;
    }

    if (!submit_command(
            motor_base::ControlCommand::set_impedance_targets(setpoints),
            "apply_impedance_setpoints")) {
        return false;
    }
    return true;
}

bool RobotMotorSession::apply_impedance_setpoints_realtime(
    const std::array<motor_base::ImpedanceSetpoint,
                     motor_base::kMaxMotorCommandSetpoints>& setpoints,
    std::size_t count)
{
    const motor_base::ControlCommand command =
        motor_base::ControlCommand::set_impedance_targets_fixed(
            setpoints.data(),
            count);
    const motor_base::CommandSubmitResult result =
        controller_->send_realtime_setpoint_command(command);
    if (result.status == motor_base::CommandSubmitStatus::ACCEPTED) {
        return true;
    }

    std::cerr << "[RobotMotorSession] apply_impedance_setpoints command rejected: "
              << command_submit_status_name(result.status) << "\n";
    return false;
}

bool RobotMotorSession::try_consume_command_feedback(
    std::array<motor_base::MotorStatusSnapshot,
               motor_base::kMaxMotorCommandSetpoints>& feedback)
{
    if (!initialized_.load() || !controller_) {
        return false;
    }
    return controller_->try_consume_command_feedback(feedback);
}


bool RobotMotorSession::submit_command(const motor_base::ControlCommand& command,
                                       const char* context)
{
    const motor_base::CommandSubmitResult result = controller_->send_command(command);
    if (result.status == motor_base::CommandSubmitStatus::ACCEPTED) {
        return true;
    }

    std::cerr << "[RobotMotorSession] " << context
              << " command rejected: "
              << command_submit_status_name(result.status) << "\n";
    return false;
}




std::vector<double> RobotMotorSession::get_joint_q() const
{
    return controller_->get_positions_rad();
}


MotorStateSnapshot RobotMotorSession::get_motor_snapshot() const
{
    MotorStateSnapshot snapshot;
    if (!controller_) {
        return snapshot;
    }

    // 优先读 policy 线程专属的 RT feedback 通道；无新帧时沿用缓存帧。
    // 启动窗口内尚未收到任何反馈帧时退回公共状态通道，保持旧行为。
    std::array<motor_base::MotorStatusSnapshot,
               motor_base::kMaxMotorCommandSetpoints> latest_feedback;
    if (controller_->try_consume_policy_feedback(latest_feedback)) {
        latest_feedback_ = latest_feedback;
        has_policy_feedback_ = true;
    }
    if (!has_policy_feedback_) {
        const std::vector<motor_base::MotorStatusSnapshot> status = controller_->get_status();
        fill_motor_snapshot_from_range(snapshot, status.begin(), status.end());
        return snapshot;
    }

    const std::size_t motor_count = static_cast<std::size_t>(config_.num_motors);
    fill_motor_snapshot_from_range(snapshot, latest_feedback_.data(),
                                    latest_feedback_.data() + motor_count);

    return snapshot;
}

}  // namespace inference
