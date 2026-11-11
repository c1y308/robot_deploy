#include "driver/myact/myact_motor_controller.hpp"
#include "protocol/ethercat/ethercat_adapter_igh.hpp"
#include "robot/action_processor.hpp"
#include "robot/policy_command_worker.hpp"
#include "robot/robot_interface.hpp"
#include "robot/robot_motor_session.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

namespace {
// Publish in-memory feedback without connect(), start() or any hardware access.
class TestMotorController : public myactua::MyActMotorController {
public:
    using myactua::MyActMotorController::MyActMotorController;
    using motor_base::MotorControllerBase::publish_feedback;
};

void check(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}
}

namespace inference {

// Exercise timing and channel behavior without initializing hardware or a worker thread.
struct RobotInterfacePolicyTimingTestAccess {
    static void verify_uninitialized_interface(bool logging_enabled)
    {
        RobotInterfaceConfig config;
        config.recorder.enabled = logging_enabled;
        {
            RobotInterface robot(config);
            check(!robot.is_initialized() && !robot.policy_command_worker_,
                  "worker constructed before initialization");
            // Destruction must also work without initialize() or explicit shutdown().
        }

        RobotInterface robot(config);
        std::ostringstream diagnostic;
        auto* previous_buffer = std::cerr.rdbuf(diagnostic.rdbuf());
        const bool stepped = robot.policy_step();
        std::cerr.rdbuf(previous_buffer);
        check(!stepped && !robot.is_initialized() && !robot.policy_command_worker_,
              "policy step accepted without a worker");
        check(diagnostic.str().find("policy command worker is not initialized") !=
                  std::string::npos,
              "missing worker was not diagnosed");
        check(robot.shutdown() == ShutdownResult::Confirmed &&
              robot.shutdown() == ShutdownResult::Confirmed,
              "shutdown without a worker is not repeatable");
    }

    static void verify_failed_interface_initialization(bool logging_enabled)
    {
        RobotInterfaceConfig config;
        config.recorder.enabled = logging_enabled;
        // Fail before model setup or any hardware initialization.
        config.joint_mapping.model_to_motor_index.clear();
        RobotInterface robot(config);
        check(!robot.initialize() && !robot.is_initialized() &&
              !robot.policy_command_worker_,
              "failed processor initialization left a worker");
        check(robot.shutdown() == ShutdownResult::Confirmed &&
              robot.shutdown() == ShutdownResult::Confirmed,
              "shutdown after early initialization failure is not repeatable");
    }

    static void verify_failed_worker_start(bool logging_enabled)
    {
        RobotInterfaceConfig config;
        config.recorder.enabled = logging_enabled;
        RobotInterface robot(config);
        check(robot.initialize_processors(), "failed to create test processors");
        robot.policy_command_worker_ = std::make_unique<PolicyCommandWorker>(
            robot.config_, robot.motor_session_, *robot.action_processor_,
            robot.first_policy_inference_started_ns_);
        auto& worker = *robot.policy_command_worker_;
        // No motor controller exists, so startup refresh fails without starting a thread.
        check(!worker.start({}) && !worker.is_running() &&
              !worker.worker_thread_.joinable() && !robot.is_initialized(),
              "worker started without initialized motors");
        worker.stop();
        worker.stop();
        std::string error;
        check(!worker.check_health(error) && error == "policy command worker is not running",
              "failed worker start has incorrect health state");
        check(!robot.policy_step() && robot.policy_command_worker_.get() == &worker,
              "fault handling destroyed the stopped worker prematurely");
        check(robot.shutdown() == ShutdownResult::Confirmed &&
              !robot.policy_command_worker_ &&
              robot.shutdown() == ShutdownResult::Confirmed,
              "shutdown did not release the failed worker safely");
    }

    static void verify_deadlines(PolicyCommandWorker& worker)
    {
        PolicyTargetFrame target;
        target.policy_seq = 7;
        target.policy_valid_until_ns = 2'000'000'000;
        const auto command_limited = worker.build_command_timing(
            target, 1'000'000'000, 1'020'000'000);
        check(command_limited.source_policy_seq == 7 &&
              command_limited.produced_at_ns == 1'000'000'000 &&
              command_limited.valid_until_ns == 1'010'000'000,
              "command timeout or timing metadata changed");

        const auto feedback_limited = worker.build_command_timing(
            target, 1'000'000'000, 1'005'000'000);
        check(feedback_limited.valid_until_ns == 1'005'000'000,
              "command outlives feedback");

        target.policy_valid_until_ns = 1'003'000'000;
        const auto target_limited = worker.build_command_timing(
            target, 1'000'000'000, 1'020'000'000);
        const auto refreshed = worker.build_command_timing(
            target, 1'001'000'000, 1'021'000'000);
        check(target_limited.valid_until_ns == 1'003'000'000 &&
              refreshed.valid_until_ns == target_limited.valid_until_ns,
              "refresh extends policy target deadline");

        target.policy_valid_until_ns = 2'000'000'000;
        const auto expired = worker.build_command_timing(
            target, 1'005'000'000, 1'005'000'000);
        check(!expired.is_well_formed() && expired.valid_until_ns == 1'005'000'000,
              "command timing accepts feedback at its deadline");
    }

    static TestMotorController& install_feedback_controller(RobotMotorSession& motors)
    {
        motors.adapter_ = std::make_shared<myactua::EthercatAdapterIGH>();
        auto controller = std::make_unique<TestMotorController>(
            motors.adapter_, motors.config_.num_motors);
        auto& result = *controller;
        motors.controller_ = std::move(controller);
        motors.initialized_.store(true);
        return result;
    }

    static void verify_feedback()
    {
        for (const std::int64_t max_age_ns : {20'000'000LL, 35'000'000LL}) {
            RobotInterfaceConfig config;
            config.sensor_guard.max_motor_sample_age_s =
                static_cast<double>(max_age_ns) / 1'000'000'000.0;
            // Also verify unused slots do not affect the oldest active motor.
            config.motor.num_motors = max_age_ns == 20'000'000 ? 12 : 6;
            RobotInterface robot(config);
            auto& motors = robot.motor_session_;
            std::string error = "previous error";
            std::int64_t deadline_ns = 0;
            check(!motors.refresh_command_feedback(error) &&
                  error == "policy motor feedback is not initialized",
                  "feedback refresh accepted without a controller");
            check(!motors.validate_motor_feedback_timing(1'000'000'000, deadline_ns, error),
                  "feedback validation accepted an empty cache");

            auto& controller = install_feedback_controller(motors);
            check(!motors.refresh_command_feedback(error) &&
                  error == "policy motor feedback is not initialized",
                  "feedback refresh accepted before the first frame");

            std::array<motor_base::MotorStatusSnapshot, motor_base::kMotorNum> feedback{};
            for (int i = 0; i < config.motor.num_motors; ++i) {
                feedback[i].motor_index = i;
                feedback[i].host_timestamp_ns = 1'001'000'000;
                feedback[i].position_rad = 0.01 * i;
            }
            feedback[4].host_timestamp_ns = 1'000'000'000;
            controller.publish_feedback(feedback);
            check(motors.refresh_command_feedback(error) && error.empty() &&
                  motors.command_feedback()[4].host_timestamp_ns == 1'000'000'000 &&
                  motors.command_feedback()[4].position_rad == feedback[4].position_rad,
                  "first feedback frame was not cached");
            check(!motors.has_policy_feedback_,
                  "command refresh modified the policy cache");
            const auto policy_snapshot = motors.get_motor_snapshot();
            check(motors.has_policy_feedback_ &&
                  motors.latest_feedback_[4].host_timestamp_ns == 1'000'000'000 &&
                  policy_snapshot.position_rad[4] == feedback[4].position_rad,
                  "command consumer drained the policy feedback channel");

            check(motors.validate_motor_feedback_timing(1'000'000'000, deadline_ns, error) &&
                  error.empty() && deadline_ns == 1'000'000'000 + max_age_ns,
                  "feedback deadline ignores the oldest motor or sensor configuration");
            const auto original_deadline_ns = deadline_ns;
            check(motors.refresh_command_feedback(error) && error.empty() &&
                  motors.command_feedback()[4].host_timestamp_ns == 1'000'000'000 &&
                  motors.validate_motor_feedback_timing(deadline_ns - 1, deadline_ns, error) &&
                  deadline_ns == original_deadline_ns,
                  "refresh without a new frame changed or renewed cached feedback");

            // Simulate a new frame arriving while a command uses the cached frame.
            // Validation must still expire the frame used by that calculation.
            for (int i = 0; i < config.motor.num_motors; ++i) {
                feedback[i].host_timestamp_ns += 100'000'000;
                feedback[i].position_rad += 0.1;
            }
            controller.publish_feedback(feedback);
            check(!motors.validate_motor_feedback_timing(
                      original_deadline_ns, deadline_ns, error),
                  "feedback accepted at its deadline or silently refreshed during validation");
            const auto max_age_us = std::to_string(max_age_ns / 1000);
            check(error == "motor feedback expired: motor_index=4, feedback_age_us=" +
                      max_age_us + ", max_motor_age_us=" + max_age_us,
                  "feedback timeout diagnostic changed");
            check(!motors.validate_motor_feedback_timing(
                      original_deadline_ns + 1, deadline_ns, error),
                  "cached feedback accepted after expiry");

            check(motors.refresh_command_feedback(error) && error.empty() &&
                  motors.command_feedback()[4].host_timestamp_ns == 1'100'000'000 &&
                  motors.validate_motor_feedback_timing(1'100'000'000, deadline_ns, error) &&
                  deadline_ns == 1'100'000'000 + max_age_ns,
                  "new feedback did not replace the expired cache");
            check(motors.latest_feedback_[4].host_timestamp_ns == 1'000'000'000 &&
                  motors.latest_feedback_[4].position_rad == policy_snapshot.position_rad[4],
                  "command refresh changed the previously consumed policy snapshot");
            check(motors.get_motor_snapshot().position_rad[4] == feedback[4].position_rad &&
                  motors.latest_feedback_[4].host_timestamp_ns == 1'100'000'000 &&
                  motors.command_feedback()[4].host_timestamp_ns == 1'100'000'000,
                  "policy refresh changed or could not independently consume feedback");

            feedback[4].host_timestamp_ns = 0;
            controller.publish_feedback(feedback);
            check(motors.refresh_command_feedback(error) &&
                  !motors.validate_motor_feedback_timing(1'100'000'000, deadline_ns, error) &&
                  error.find("motor feedback timestamp missing: motor_index=4") == 0,
                  "missing feedback timestamp accepted");
            feedback[4].host_timestamp_ns = 1'100'000'000;
            controller.publish_feedback(feedback);
            check(motors.refresh_command_feedback(error) &&
                  motors.validate_motor_feedback_timing(1'100'000'000, deadline_ns, error) &&
                  error.empty() && motors.is_initialized(),
                  "feedback did not recover after a valid frame");

            check(motors.deinitialize() &&
                  !motors.has_command_feedback_ &&
                  motors.command_feedback()[4].host_timestamp_ns == 0 &&
                  !motors.refresh_command_feedback(error) &&
                  !motors.validate_motor_feedback_timing(1'100'000'000, deadline_ns, error),
                  "released session retained usable command feedback");
            auto& next_controller = install_feedback_controller(motors);
            check(!motors.refresh_command_feedback(error),
                  "new controller reused feedback from the previous controller");
            next_controller.publish_feedback(feedback);
            check(motors.refresh_command_feedback(error) &&
                  motors.validate_motor_feedback_timing(1'100'000'000, deadline_ns, error),
                  "new controller could not populate the cleared command cache");
        }
    }

    static void verify_target_channel(PolicyCommandWorker& worker, bool logging_enabled)
    {
        for (std::uint64_t seq : {1U, 2U}) {
            PolicyTargetFrame target;
            target.policy_seq = seq;
            target.target_model_rad[0] = static_cast<double>(seq);
            target.published_at_ns = 1;
            target.policy_valid_until_ns = 2;
            InferenceRecord record;
            if (logging_enabled) {
                record.policy_seq = seq;
                record.raw_action[0] = static_cast<float>(seq);
                record.policy_valid_until_ns = 3;
            }
            const auto published_at_ns =
                worker.publish_target(target, logging_enabled ? &record : nullptr);
            check(published_at_ns > 2, "invalid target publication time");
            check(target.published_at_ns == 1 && target.policy_valid_until_ns == 2 &&
                  (!logging_enabled || record.policy_valid_until_ns == 3),
                  "publication changed caller-owned timing metadata");
            target.target_model_rad[0] = -1.0;
            record.raw_action[0] = -1.0F;
        }

        PolicyCommandWorker::PolicyTargetWithRecordFrame frame;
        check(worker.try_consume_target(frame) && frame.target.policy_seq == 2 &&
              frame.target.target_model_rad[0] == 2.0,
              "target channel did not preserve latest target");
        check(frame.target.policy_valid_until_ns - frame.target.published_at_ns == 60'000'000,
              "published target timeout changed");
        if (logging_enabled) {
            check(frame.record.policy_seq == frame.target.policy_seq &&
                  frame.record.raw_action[0] == 2.0F &&
                  frame.record.policy_valid_until_ns == frame.target.policy_valid_until_ns,
                  "target and log record lost pairing");
        }
        check(!worker.try_consume_target(frame), "target consumed twice");
    }

    static void verify_command_record(PolicyCommandWorker& worker)
    {
        InferenceRecord record;
        record.policy_seq = 9;
        record.frame_index = 8;
        record.raw_action[0] = 0.5F;
        PolicyMotorCommand command;
        command.setpoint_count = motor_base::kMotorNum;
        for (std::size_t i = 0; i < command.setpoints.size(); ++i) {
            command.setpoints[i].position_rad = static_cast<double>(i);
            command.setpoints[i].effort_ff = -static_cast<double>(i);
        }
        motor_base::CommandTiming timing;
        timing.source_policy_seq = record.policy_seq;
        timing.produced_at_ns = 1'000'000'000;
        timing.valid_until_ns = 1'010'000'000;
        worker.complete_and_publish_record(record, command, timing);

        InferenceRecord completed;
        check(worker.try_consume_completed_record(completed) &&
              completed.policy_seq == 9 && completed.frame_index == 8 &&
              completed.raw_action[0] == 0.5F && completed.command_applied &&
              completed.command_timestamp_ns == timing.produced_at_ns &&
              completed.command_valid_until_ns == timing.valid_until_ns,
              "command record lost source or command metadata");
        for (std::size_t i = 0; i < command.setpoints.size(); ++i) {
            check(completed.target_pos_rad[i] == command.setpoints[i].position_rad &&
                  completed.target_effort_permille[i] == command.setpoints[i].effort_ff,
                  "command record lost motor setpoints");
        }
        check(!worker.try_consume_completed_record(completed), "command record consumed twice");
    }

    static void verify_startup_hold(PolicyCommandWorker& worker,
                                   std::atomic<std::int64_t>& first_inference_ns)
    {
        // Before inference there is no first-target deadline. The absent motor
        // controller rejects the refresh, rather than reporting a target timeout.
        first_inference_ns.store(0);
        check(!worker.refresh_startup_hold(), "startup hold accepted without motors");
        std::string error;
        check(!worker.check_health(error) && error == "failed to refresh startup reset pose",
              "first target deadline armed before inference");

        // This deadline is already in the past, so no position command is submitted.
        first_inference_ns.store(1);
        check(!worker.refresh_startup_hold(), "expired startup hold accepted");
        check(!worker.check_health(error) &&
              error.find("first policy target deadline expired: deadline_ns=60000001, overdue_us=") == 0 &&
              !worker.is_running(), "startup timeout did not preserve fault state");
    }
};

}  // namespace inference

int main()
{
    using Access = inference::RobotInterfacePolicyTimingTestAccess;
    for (bool logging_enabled : {false, true}) {
        Access::verify_uninitialized_interface(logging_enabled);
        Access::verify_failed_interface_initialization(logging_enabled);
        Access::verify_failed_worker_start(logging_enabled);

        inference::RobotInterfaceConfig config;
        config.safety.policy_target_timeout_ms = 60.0;
        config.safety.control_command_timeout_ms = 10.0;
        config.sensor_guard.max_motor_sample_age_s = 0.020;
        config.recorder.enabled = logging_enabled;
        const inference::JointMapping mapping(static_cast<int>(inference::kDof),
                                              config.joint_mapping);
        inference::ActionProcessor action_processor(config, mapping);
        inference::RobotMotorSession motors(
            config.motor, config.safety, config.runtime, config.sensor_guard);
        std::atomic<std::int64_t> first_inference_ns{0};
        inference::PolicyCommandWorker worker(config, motors, action_processor, first_inference_ns);
        Access::verify_deadlines(worker);
        Access::verify_target_channel(worker, logging_enabled);
        if (logging_enabled) {
            Access::verify_command_record(worker);
        }
        Access::verify_startup_hold(worker, first_inference_ns);
    }
    Access::verify_feedback();
}
