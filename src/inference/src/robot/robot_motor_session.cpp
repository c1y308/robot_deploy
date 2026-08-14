#include "robot/robot_motor_session.hpp"

#include "driver/myact/motor_control.hpp"
#include "ethercat_adapter_igh.hpp"
#include "motor_base/motor_base.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <thread>
#include <utility>

namespace inference {
namespace {

bool finite_vector(const std::vector<double>& values)
{
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    });
}

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

const char* command_submit_result_name(motor_base::CommandSubmitResult result)
{
    switch (result) {
        case motor_base::CommandSubmitResult::ACCEPTED: return "ACCEPTED";
        case motor_base::CommandSubmitResult::QUEUE_FULL: return "QUEUE_FULL";
        case motor_base::CommandSubmitResult::INVALID_COMMAND: return "INVALID_COMMAND";
        case motor_base::CommandSubmitResult::INVALID_PAYLOAD: return "INVALID_PAYLOAD";
    }
    return "UNKNOWN";
}

std::int64_t motor_steady_now_ns() noexcept
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace

RobotMotorSession::RobotMotorSession(MotorConfig config)
    : config_(std::move(config)) {}

RobotMotorSession::~RobotMotorSession()
{
    deinitialize();
}

bool RobotMotorSession::initialize_and_start()
{
    if (initialized_.load()) {
        return true;
    }

    if (!validate_config()) {
        return false;
    }

    adapter_    = std::make_shared<myactua::EthercatAdapterIGH>();
    controller_ = std::make_unique<myactua::MYACTUA>(adapter_, config_.num_motors);

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

    controller_->start();

    if (!submit_command(motor_base::ControlCommand::stop(), "initial_stop")) {
        controller_.reset();
        adapter_.reset();
        return false;
    }
    initialized_.store(true);
    motion_enabled_.store(false);

    return true;
}

bool RobotMotorSession::validate_config() const
{
    if (config_.num_motors <= 0) {
        std::cerr << "[RobotMotorSession] num_motors must be positive\n";
        return false;
    }

    if (is_mit_mode(config_.control_mode)) {
        if (static_cast<int>(config_.mit_kp.size()) != config_.num_motors) {
            std::cerr << "[RobotMotorSession] MIT mode requires mit_kp size="
                      << config_.num_motors << ", got=" << config_.mit_kp.size()
                      << "\n";
            return false;
        }
        if (static_cast<int>(config_.mit_kd.size()) != config_.num_motors) {
            std::cerr << "[RobotMotorSession] MIT mode requires mit_kd size="
                      << config_.num_motors << ", got=" << config_.mit_kd.size()
                      << "\n";
            return false;
        }
        if (!finite_vector(config_.mit_kp) || !finite_vector(config_.mit_kd)) {
            std::cerr << "[RobotMotorSession] MIT mode requires finite mit_kp/mit_kd values\n";
            return false;
        }
    }

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

bool RobotMotorSession::submit_command(const motor_base::ControlCommand& command,
                                       const char* context)
{
    if (!controller_) {
        return false;
    }
    const motor_base::CommandSubmitResult result = controller_->send_command(command);
    if (result == motor_base::CommandSubmitResult::ACCEPTED) {
        return true;
    }

    std::cerr << "[RobotMotorSession] " << context
              << " command rejected: "
              << command_submit_result_name(result) << "\n";
    return false;
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

    if (!submit_command(motor_base::ControlCommand::restart(motor_index), "restart")) {
        return false;
    }
    if (motor_index < 0 || config_.num_motors == 1) {
        motion_enabled_.store(true);
    }
    return true;
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

std::vector<double> RobotMotorSession::get_joint_q() const
{
    std::vector<double> q(config_.num_motors, 0.0);
    if (!controller_) {
        return q;
    }
    q = controller_->get_joint_q_rad();
    return q;
}

std::vector<double> RobotMotorSession::get_joint_vel() const
{
    std::vector<double> dq(config_.num_motors, 0.0);
    if (!controller_) {
        return dq;
    }
    dq = controller_->get_joint_vel_rad_s();
    return dq;
}

std::vector<double> RobotMotorSession::get_joint_torque_percent() const
{
    std::vector<double> torque(config_.num_motors, 0.0);
    if (!controller_) {
        return torque;
    }
    torque = controller_->get_joint_torque_percent();
    return torque;
}

MotorStateSnapshot RobotMotorSession::get_motor_state() const
{
    MotorStateSnapshot snapshot;
    snapshot.timestamp_ns = motor_steady_now_ns();
    if (!controller_) {
        return snapshot;
    }

    const std::vector<motor_base::MotorStatusSnapshot> status = controller_->get_status();
    snapshot.position_rad.reserve(status.size());
    snapshot.velocity_rad_s.reserve(status.size());
    snapshot.torque_percent.reserve(status.size());
    snapshot.comm_ok.reserve(status.size());
    snapshot.enabled.reserve(status.size());
    snapshot.faulted.reserve(status.size());

    for (const auto& motor : status) {
        snapshot.position_rad.push_back(motor.position_rad);
        snapshot.velocity_rad_s.push_back(motor.velocity_rad_s);
        snapshot.torque_percent.push_back(motor.torque_percent);
        snapshot.comm_ok.push_back(motor.comm_ok ? 1U : 0U);
        snapshot.enabled.push_back(motor.enabled ? 1U : 0U);
        snapshot.faulted.push_back(motor.faulted ? 1U : 0U);
    }

    return snapshot;
}

}  // namespace inference
