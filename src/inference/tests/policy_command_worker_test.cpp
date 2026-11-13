#include "driver/myact/myact_motor_controller.hpp"
#include "motor_base/motor_controller_base.hpp"
#include "protocol/ethercat/ethercat_adapter_igh.hpp"
#include "robot/action_processor.hpp"
#include "robot/policy_command_worker.hpp"
#include "robot/robot_interface.hpp"
#include "robot/robot_motor_session.hpp"
#include "tool/tool.hpp"

#include <array>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <sys/wait.h>
#include <unistd.h>

namespace {
// Publish in-memory feedback without connect(), start() or any hardware access.
class TestMotorController : public myactua::MyActMotorController {
public:
    using myactua::MyActMotorController::MyActMotorController;
    using motor_base::MotorControllerBase::publish_feedback;
    using motor_base::MotorControllerBase::latch_terminal_fault;
};

// A different driver implementation; no adapter, hardware or RT thread is used.
class MemoryMotorController final : public motor_base::MotorControllerBase {
public:
    using motor_base::MotorControllerBase::MotorControllerBase;

    bool fault_latched{false};
    bool healthy{true};

    bool communication_fault_latched() const noexcept override { return fault_latched; }
    bool communication_healthy() const noexcept override { return healthy; }
    bool wait_all_motors_ready(int, int) const override { return true; }
    void set_print_info(const std::vector<int>&) override {}

private:
    bool connect_impl() override { return true; }
    void realtime_cycle_callback() override {}
    void apply_setpoint_command_impl(const motor_base::ControlCommand&) override {}
    void apply_discrete_command_impl(int, const motor_base::DiscreteCommand&) override {}
    motor_base::DiscreteCommandEvaluation evaluate_discrete_command_impl(
        int, const motor_base::DiscreteCommand&) const override
    {
        return motor_base::DiscreteCommandEvaluation::SATISFIED;
    }
};

void check(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

template <typename Exercise>
void expect_destructor_termination(Exercise exercise, const char* expected_diagnostic)
{
    int diagnostic_pipe[2];
    check(pipe(diagnostic_pipe) == 0, "failed to create destructor diagnostic pipe");
    const pid_t child = fork();
    check(child >= 0, "failed to fork destructor test");
    if (child == 0) {
        close(diagnostic_pipe[0]);
        check(dup2(diagnostic_pipe[1], STDERR_FILENO) >= 0,
              "failed to redirect destructor diagnostic");
        close(diagnostic_pipe[1]);
        std::set_terminate([] { std::_Exit(86); });
        // An infinite destructor retry must fail this test instead of hanging it.
        alarm(5);
        exercise();
        std::_Exit(0);
    }

    close(diagnostic_pipe[1]);
    int status = 0;
    check(waitpid(child, &status, 0) == child, "failed to wait for destructor test");
    std::string diagnostic;
    char buffer[256];
    ssize_t count;
    while ((count = read(diagnostic_pipe[0], buffer, sizeof(buffer))) > 0) {
        diagnostic.append(buffer, static_cast<std::size_t>(count));
    }
    close(diagnostic_pipe[0]);
    check(count == 0, "failed to read destructor diagnostic");
    check(WIFEXITED(status) && WEXITSTATUS(status) == 86,
          "failed destructor did not call std::terminate");
    check(diagnostic.find(expected_diagnostic) != std::string::npos,
          "destructor did not flush its fatal diagnostic before termination");
}
}

namespace motor_base {
struct MotorControllerTimingTestAccess {
    static DiscreteCommandResultTracker& results(MotorControllerBase& controller)
    {
        return controller.discrete_command_results_;
    }

    static bool consume_setpoint(MotorControllerBase& controller, ControlCommand& command)
    {
        return controller.setpoint_channel_policy_.try_consume_latest(command);
    }

    static CommandId latest_stop(const MotorControllerBase& controller)
    {
        return controller.latest_stop_id(0);
    }

    static void confirm_stop(MotorControllerBase& controller)
    {
        // Drive the real STOP confirmation path synchronously, without start().
        for (std::uint64_t tick = 0;
             tick <= kDiscreteVerifyIntervalTicks * kDiscreteSuccessStableTicks;
             ++tick) {
            ++controller.discrete_cmd_tick_;
            controller.service_stop_requests(true);
        }
    }
};
}

namespace inference {

// Exercise timing and channel behavior without initializing hardware or a worker thread.
struct RobotInterfacePolicyTimingTestAccess {
    static void verify_shutdown_with_base_controller()
    {
        for (bool stop_confirmed : {false, true}) {
            for (bool fault_latched : {false, true}) {
                for (bool healthy : {false, true}) {
                    RobotInterfaceConfig config;
                    config.motor.discrete_command_completion_timeout_ms = 0;
                    RobotMotorSession motors(config);
                    auto controller = std::make_unique<MemoryMotorController>(
                        config.motor.num_motors);
                    auto& driver = *controller;
                    driver.fault_latched = fault_latched;
                    driver.healthy = healthy;
                    motors.controller_ = std::move(controller);
                    motors.initialized_.store(true);
                    // Model the session owning RT, but do not launch any threads.
                    motors.rt_started_ = true;

                    const auto request = motors.request_stop();
                    check(request.status == motor_base::CommandSubmitStatus::ACCEPTED &&
                          request.command_id.has_value(),
                          "base-only controller rejected STOP");
                    if (stop_confirmed) {
                        motor_base::MotorControllerTimingTestAccess::confirm_stop(driver);
                        check(driver.get_discrete_command_result(*request.command_id) ==
                                  motor_base::DiscreteCommandResult::SUCCEEDED,
                              "in-memory STOP was not confirmed");
                    }

                    const auto expected = stop_confirmed
                        ? ShutdownResult::Confirmed
                        : (fault_latched && !healthy
                               ? ShutdownResult::ReleasedWithCommLoss
                               : ShutdownResult::RetryRequired);
                    check(motors.shutdown(request) == expected,
                          "base-only controller changed the shutdown decision");
                    if (expected == ShutdownResult::RetryRequired) {
                        check(motors.controller_.get() == &driver &&
                              motors.is_initialized() && motors.rt_started_,
                              "retry discarded the controller or session state");
                        check(motors.request_stop().command_id == request.command_id,
                              "retry replaced the outstanding STOP");
                        motor_base::MotorControllerTimingTestAccess::confirm_stop(driver);
                        check(motors.shutdown(request) == ShutdownResult::Confirmed,
                              "confirmed STOP could not complete a shutdown retry");
                    }
                    check(!motors.controller_ && !motors.is_initialized() &&
                          !motors.rt_started_,
                          "completed shutdown retained the controller or session state");
                }
            }
        }
    }

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
        const auto command_limited = worker.build_policy_command_timing(
            target, 1'000'000'000, 1'020'000'000);
        check(command_limited.source_policy_seq == 7 &&
              command_limited.produced_at_ns == 1'000'000'000 &&
              command_limited.valid_until_ns == 1'010'000'000,
              "command timeout or timing metadata changed");

        const auto feedback_limited = worker.build_policy_command_timing(
            target, 1'000'000'000, 1'005'000'000);
        check(feedback_limited.valid_until_ns == 1'005'000'000,
              "command outlives feedback");

        target.policy_valid_until_ns = 1'003'000'000;
        const auto target_limited = worker.build_policy_command_timing(
            target, 1'000'000'000, 1'020'000'000);
        const auto refreshed = worker.build_policy_command_timing(
            target, 1'001'000'000, 1'021'000'000);
        check(target_limited.valid_until_ns == 1'003'000'000 &&
              refreshed.valid_until_ns == target_limited.valid_until_ns,
              "refresh extends policy target deadline");

        target.policy_valid_until_ns = 2'000'000'000;
        const auto expired = worker.build_policy_command_timing(
            target, 1'005'000'000, 1'005'000'000);
        check(!expired.is_well_formed() && expired.valid_until_ns == 1'005'000'000,
              "command timing accepts feedback at its deadline");
    }

    static TestMotorController& install_feedback_controller(
        RobotMotorSession& motors,
        myactua::MyActMotorController::Options options = {})
    {
        auto adapter = std::make_shared<myactua::EthercatAdapterIGH>();
        auto controller = std::make_unique<TestMotorController>(
            std::move(adapter), motors.config_.num_motors, std::move(options));
        auto& result = *controller;
        motors.controller_ = std::move(controller);
        motors.initialized_.store(true);
        return result;
    }

    static void verify_position_hold_command(
        const motor_base::ControlCommand& command,
        const std::array<double, motor_base::kMotorNum>& target,
        const RobotInterfaceConfig& config)
    {
        check(command.payload_valid && command.payload_size == target.size(),
              "position hold payload changed");
        for (std::size_t i = 0; i < target.size(); ++i) {
            if (config.motor.control_mode == motor_base::MotorControlMode::POSITION) {
                check(command.setpoint_type == motor_base::SetpointCommandType::POSITION_TARGETS &&
                      command.setpoints[i] == target[i], "POSITION target changed");
            } else {
                const auto& setpoint = command.impedance_setpoints[i];
                check(command.setpoint_type == motor_base::SetpointCommandType::IMPEDANCE_TARGETS &&
                      setpoint.position_rad == target[i] && setpoint.velocity_rad_s == 0.0 &&
                      setpoint.effort_ff == 0.0 && setpoint.kp == config.motor.mit_kp[i] &&
                      setpoint.kd == config.motor.mit_kd[i],
                      "impedance hold changed position, physical motor gains or feedforward");
            }
        }
        check(command.timing.source_policy_seq == 0,
              "position hold acquired a policy sequence");
    }

    static void verify_target_submission_after_stop()
    {
        for (const auto mode : {motor_base::MotorControlMode::POSITION,
                                motor_base::MotorControlMode::IMPEDANCE}) {
            RobotInterfaceConfig config;
            config.motor.control_mode = mode;
            config.safety.control_command_timeout_ms = 1000.0;
            std::array<double, motor_base::kMotorNum> target{};
            for (std::size_t i = 0; i < target.size(); ++i) {
                target[i] = 0.01 * static_cast<double>(i);
                config.motor.mit_kp[i] = 10.0 + static_cast<double>(i);
                config.motor.mit_kd[i] = 1.0 + 0.1 * static_cast<double>(i);
            }
            auto setpoints = ActionProcessor::make_position_impedance_setpoints(target, config.motor);
            RobotMotorSession motors(config);
            check(!motors.submit_position_targets_rad(target, {}) &&
                  !motors.submit_impedance_setpoints(setpoints, {}),
                  "uninitialized session accepted a continuous target");
            myactua::MyActMotorController::Options options;
            // Test publication without starting RT or accessing hardware.
            options.rt_thread_options.scheduling_policy =
                robot_base::ThreadSchedulingPolicy::OTHER;
            auto& controller = install_feedback_controller(motors, std::move(options));
            const auto request = motors.request_stop();
            check(request.status == motor_base::CommandSubmitStatus::ACCEPTED &&
                  request.command_id.has_value(), "test STOP request was rejected");
            const auto submit = [&](std::int64_t deadline = 0) {
                motor_base::CommandTiming timing;
                timing.produced_at_ns = robot_base::monotonic_now_ns();
                timing.valid_until_ns = timing.produced_at_ns + robot_base::seconds_to_ns(
                    config.safety.control_command_timeout_ms / 1000.0);
                if (deadline > 0) {
                    timing.valid_until_ns = std::min(timing.valid_until_ns, deadline);
                }
                if (mode == motor_base::MotorControlMode::POSITION) {
                    return motors.submit_position_targets_rad(target, timing);
                }
                return motors.submit_impedance_setpoints(setpoints, timing);
            };
            check(submit(),
                  "target publication was rejected after STOP submission");
            motor_base::ControlCommand command;
            check(motor_base::MotorControllerTimingTestAccess::consume_setpoint(controller, command),
                  "submitted target did not reach the command channel");
            verify_position_hold_command(command, target, config);
            check(command.timing.valid_until_ns - command.timing.produced_at_ns == 1'000'000'000,
                  "position hold lost its independent command timeout");
            const auto deadline = robot_base::monotonic_now_ns() + 500'000'000;
            check(submit(deadline) &&
                  motor_base::MotorControllerTimingTestAccess::consume_setpoint(controller, command) &&
                  command.timing.valid_until_ns == deadline,
                  "position hold did not preserve the external deadline cap");
            check(!submit(1) &&
                  !motor_base::MotorControllerTimingTestAccess::consume_setpoint(controller, command),
                  "expired position hold was published");

            motor_base::CommandTiming policy_timing;
            policy_timing.source_policy_seq = 19;
            policy_timing.produced_at_ns = robot_base::monotonic_now_ns();
            policy_timing.valid_until_ns = policy_timing.produced_at_ns + 100'000'000;
            if (mode == motor_base::MotorControlMode::POSITION) {
                check(!motors.submit_impedance_setpoints(setpoints, policy_timing),
                      "POSITION session accepted an impedance target");
                check(motors.submit_position_targets_rad(target, policy_timing) &&
                      motor_base::MotorControllerTimingTestAccess::consume_setpoint(controller, command),
                      "complete POSITION command was rejected");
                check(command.timing.source_policy_seq == policy_timing.source_policy_seq &&
                      command.timing.produced_at_ns == policy_timing.produced_at_ns &&
                      command.timing.valid_until_ns == policy_timing.valid_until_ns,
                      "POSITION command replaced caller-provided timing");
            } else {
                check(!motors.submit_position_targets_rad(target, policy_timing),
                      "IMPEDANCE session accepted a POSITION target");
                setpoints[0] = motor_base::ImpedanceSetpoint(0.1, 0.2, 123.0, 0.0, 0.0);
                check(motors.submit_impedance_setpoints(setpoints, policy_timing) &&
                      motor_base::MotorControllerTimingTestAccess::consume_setpoint(controller, command),
                      "complete policy impedance command was rejected");
                check(command.impedance_setpoints[0].position_rad == 0.1 &&
                      command.impedance_setpoints[0].velocity_rad_s == 0.2 &&
                      command.impedance_setpoints[0].effort_ff == 123.0 &&
                      command.impedance_setpoints[0].kp == 0.0 &&
                      command.impedance_setpoints[0].kd == 0.0 &&
                      command.timing.source_policy_seq == policy_timing.source_policy_seq &&
                      command.timing.produced_at_ns == policy_timing.produced_at_ns &&
                      command.timing.valid_until_ns == policy_timing.valid_until_ns,
                      "policy impedance fields or original timing were replaced");
            }
            check(!controller.is_running() &&
                  controller.get_discrete_command_result(*request.command_id) ==
                      motor_base::DiscreteCommandResult::PENDING,
                  "successful target publication was mistaken for command execution");
        }
    }

    static void verify_impedance_motor_count()
    {
        for (int motor_count : {6, 12}) {
            RobotInterfaceConfig config;
            config.motor.num_motors = motor_count;
            RobotMotorSession motors(config);
            myactua::MyActMotorController::Options options;
            options.rt_thread_options.scheduling_policy = robot_base::ThreadSchedulingPolicy::OTHER;
            auto& controller = install_feedback_controller(motors, std::move(options));
            std::array<motor_base::ImpedanceSetpoint, motor_base::kMotorNum> setpoints{};
            for (std::size_t i = 0; i < setpoints.size(); ++i) {
                // Unused entries deliberately exceed the drive's torque range.
                setpoints[i] = motor_base::ImpedanceSetpoint(
                    0.01 * static_cast<double>(i), 0.0,
                    i < static_cast<std::size_t>(motor_count) ? 1.0 : 40000.0, 10.0, 1.0);
            }
            motor_base::CommandTiming timing;
            timing.source_policy_seq = 42;
            timing.produced_at_ns = robot_base::monotonic_now_ns();
            timing.valid_until_ns = timing.produced_at_ns + 10'000'000;
            motor_base::ControlCommand command;
            check(motors.submit_impedance_setpoints(setpoints, timing) &&
                  motor_base::MotorControllerTimingTestAccess::consume_setpoint(controller, command),
                  "configured motor count was not used for impedance submission");
            check(command.payload_size == static_cast<std::size_t>(motor_count) &&
                  command.timing.source_policy_seq == timing.source_policy_seq &&
                  command.timing.produced_at_ns == timing.produced_at_ns &&
                  command.timing.valid_until_ns == timing.valid_until_ns,
                  "implicit motor count changed the payload size or timing");
            for (std::size_t i = 0; i < setpoints.size(); ++i) {
                check(command.impedance_setpoints[i].effort_ff ==
                          (i < command.payload_size ? 1.0 : 0.0),
                      "unused array entries were included in the impedance payload");
            }
        }
    }

    static void verify_startup_hold_modes()
    {
        for (const auto mode : {motor_base::MotorControlMode::POSITION,
                                motor_base::MotorControlMode::IMPEDANCE}) {
            RobotInterfaceConfig config;
            config.motor.control_mode = mode;
            config.safety.control_command_timeout_ms = 1000.0;
            config.motor.mit_kp.fill(15.0);
            config.motor.mit_kd.fill(1.5);
            RobotMotorSession motors(config);
            myactua::MyActMotorController::Options options;
            options.rt_thread_options.scheduling_policy = robot_base::ThreadSchedulingPolicy::OTHER;
            auto& controller = install_feedback_controller(motors, std::move(options));
            const JointMapping mapping(static_cast<int>(kDof), config.joint_mapping);
            ActionProcessor processor(config, mapping);
            std::atomic<std::int64_t> first_inference_ns{0};
            PolicyCommandWorker worker(config, motors, processor, first_inference_ns);
            worker.startup_hold_target_motor_rad_.fill(0.1);
            motor_base::ControlCommand command;
            check(worker.submit_startup_hold() &&
                  motor_base::MotorControllerTimingTestAccess::consume_setpoint(controller, command),
                  "initial position hold was not published");
            verify_position_hold_command(command, worker.startup_hold_target_motor_rad_, config);
            check(command.timing.valid_until_ns - command.timing.produced_at_ns == 1'000'000'000,
                  "hold before first inference acquired a policy deadline");
            const auto started_ns = robot_base::monotonic_now_ns();
            first_inference_ns.store(started_ns);
            check(worker.refresh_startup_hold() &&
                  motor_base::MotorControllerTimingTestAccess::consume_setpoint(controller, command),
                  "first-frame position hold was not published");
            verify_position_hold_command(command, worker.startup_hold_target_motor_rad_, config);
            check(command.timing.valid_until_ns == started_ns + 60'000'000,
                  "position hold extended the first policy target deadline");
            first_inference_ns.store(1);
            check(!worker.refresh_startup_hold() &&
                  !motor_base::MotorControllerTimingTestAccess::consume_setpoint(controller, command),
                  "expired first-frame hold was published");
        }
    }

    static void verify_discrete_command_wait()
    {
        RobotInterfaceConfig config;
        config.motor.discrete_command_completion_timeout_ms = 0;
        RobotMotorSession motors(config);
        myactua::MyActMotorController::Options options;
        options.rt_thread_options.scheduling_policy = robot_base::ThreadSchedulingPolicy::OTHER;
        auto& controller = install_feedback_controller(motors, std::move(options));
        auto& results = motor_base::MotorControllerTimingTestAccess::results(controller);

        const auto expect_failure = [&](const motor_base::CommandSubmitResult& request,
                                        const char* expected_diagnostic) {
            std::ostringstream diagnostic;
            auto* previous_buffer = std::cerr.rdbuf(diagnostic.rdbuf());
            const bool completed = motors.wait_for_discrete_command(request, "test");
            std::cerr.rdbuf(previous_buffer);
            check(!completed && diagnostic.str().find(expected_diagnostic) != std::string::npos,
                  "discrete command failure lost its diagnostic reason");
        };

        expect_failure({motor_base::CommandSubmitStatus::QUEUE_FULL, std::nullopt},
                       "test command rejected: QUEUE_FULL");
        const auto request = controller.send_discrete_command(
            motor_base::ControlCommand::set_mode(motor_base::MotorControlMode::POSITION));
        check(request.status == motor_base::CommandSubmitStatus::ACCEPTED && request.command_id,
              "test discrete command was rejected");
        expect_failure(request, "test timed out waiting for command completion");
        for (int i = 0; i < config.motor.num_motors; ++i) {
            results.mark_done(*request.command_id, i);
        }
        check(motors.wait_for_discrete_command(request, "test"),
              "completed discrete command was rejected");

        const auto failed_request = controller.send_discrete_command(
            motor_base::ControlCommand::set_mode(motor_base::MotorControlMode::POSITION));
        check(failed_request.status == motor_base::CommandSubmitStatus::ACCEPTED &&
              failed_request.command_id, "test failing command was rejected");
        results.mark_failed(*failed_request.command_id, 0);
        expect_failure(failed_request, "test command failed");
        results.clear(*request.command_id);
        expect_failure(request, "test command result unknown");
    }

    static void prepare_unconfirmed_stop(RobotMotorSession& motors)
    {
        install_feedback_controller(motors);
        // Model an active RT controller that never confirms STOP, without hardware.
        motors.rt_started_ = true;
        const auto request = motors.request_stop();
        check(request.status == motor_base::CommandSubmitStatus::ACCEPTED &&
              request.command_id.has_value(), "test STOP request was rejected");
        check(motors.shutdown(request) == ShutdownResult::RetryRequired &&
              motors.controller_ && motors.rt_started_,
              "explicit failed shutdown did not return with its controller retained");
        const auto repeated_request = motors.request_stop();
        check(repeated_request.command_id == request.command_id,
              "repeated STOP replaced an outstanding request");
    }

    static void verify_destructor_shutdown_failure(bool interface)
    {
        RobotInterfaceConfig config;
        config.motor.discrete_command_completion_timeout_ms = 0;
        if (interface) {
            RobotInterface robot(config);
            prepare_unconfirmed_stop(robot.motor_session_);
            check(robot.shutdown() == ShutdownResult::RetryRequired,
                  "explicit interface shutdown did not return its error");
        } else {
            RobotMotorSession motors(config);
            prepare_unconfirmed_stop(motors);
        }
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
            const auto& read_only_motors = motors;
            std::string error = "previous error";
            std::int64_t deadline_ns = 0;
            check(read_only_motors.get_feedback_for_policy().timestamp_ns == 0,
                  "policy getter accepted a cache without a controller");
            check(!motors.refresh_feedback_for_command(error) &&
                  error == "policy motor feedback is not initialized",
                  "feedback refresh accepted without a controller");
            check(!motors.refresh_feedback_for_policy(error) &&
                  error == "policy motor feedback is not initialized" &&
                  motors.get_feedback_for_command().timestamp_ns == 0,
                  "policy refresh accepted without a controller");
            check(!motors.validate_feedback_timing_for_command(1'000'000'000, deadline_ns, error),
                  "feedback validation accepted an empty cache");

            auto& controller = install_feedback_controller(motors);
            // Even an existing controller cannot be consumed before session initialization.
            motors.initialized_.store(false);
            check(!motors.refresh_feedback_for_policy(error) &&
                  !motors.refresh_feedback_for_command(error),
                  "feedback refresh accepted an uninitialized session");
            motors.initialized_.store(true);
            check(!motors.refresh_feedback_for_command(error) &&
                  error == "policy motor feedback is not initialized",
                  "feedback refresh accepted before the first frame");
            check(!motors.refresh_feedback_for_policy(error) &&
                  error == "policy motor feedback is not initialized",
                  "policy refresh accepted before the first frame");

            std::array<motor_base::MotorStatusSnapshot, motor_base::kMotorNum> feedback{};
            for (int i = 0; i < config.motor.num_motors; ++i) {
                feedback[i].motor_index = i;
                feedback[i].host_timestamp_ns = 1'001'000'000;
                feedback[i].position_rad = 0.01 * i;
                feedback[i].velocity_rad_s = -0.02 * i;
                feedback[i].torque_percent = 0.03 * i;
                feedback[i].comm_ok = i % 2 == 0;
                feedback[i].enabled = i % 3 == 0;
            }
            // Poison unused slots: neither their values nor timestamps belong to the snapshot.
            for (std::size_t i = config.motor.num_motors; i < feedback.size(); ++i) {
                feedback[i].position_rad = 999;
                feedback[i].velocity_rad_s = 999;
                feedback[i].torque_percent = 999;
                feedback[i].comm_ok = true;
                feedback[i].enabled = true;
            }
            feedback[4].host_timestamp_ns = 1'000'000'000;
            controller.publish_feedback(feedback);
            check(read_only_motors.get_feedback_for_policy().timestamp_ns == 0 &&
                  !motors.feedback_for_policy_.has_feedback,
                  "policy getter consumed a frame before the first refresh");
            check(motors.refresh_feedback_for_command(error) && error.empty() &&
                  motors.get_feedback_for_command().timestamp_ns == 1'000'000'000 &&
                  motors.get_feedback_for_command().position_rad[4] == feedback[4].position_rad,
                  "first feedback frame was not cached");
            check(!motors.feedback_for_policy_.has_feedback,
                  "command refresh modified the policy cache");
            check(motors.refresh_feedback_for_policy(error) && error.empty(),
                  "first policy feedback frame was not cached");
            const auto policy_snapshot = motors.get_feedback_for_policy();
            for (const auto* snapshot : {&motors.get_feedback_for_policy(),
                                         &motors.get_feedback_for_command()}) {
                check(snapshot->timestamp_ns == 1'000'000'000,
                      "snapshot timestamp included unused slots or ignored the oldest motor");
                for (std::size_t i = 0; i < feedback.size(); ++i) {
                    const bool active = i < static_cast<std::size_t>(config.motor.num_motors);
                    check(snapshot->position_rad[i] == (active ? feedback[i].position_rad : 0) &&
                          snapshot->velocity_rad_s[i] == (active ? feedback[i].velocity_rad_s : 0) &&
                          snapshot->torque_percent[i] == (active ? feedback[i].torque_percent : 0) &&
                          snapshot->comm_ok[i] == (active && feedback[i].comm_ok) &&
                          snapshot->enabled[i] == (active && feedback[i].enabled),
                          "snapshot field conversion or unused slot clearing is incorrect");
                }
            }
            check(&motors.get_feedback_for_policy() != &motors.get_feedback_for_command(),
                  "getters do not expose independent cache references");
            check(motors.feedback_for_policy_.has_feedback &&
                  motors.feedback_for_policy_.snapshot.timestamp_ns == 1'000'000'000 &&
                  policy_snapshot.position_rad[4] == feedback[4].position_rad,
                  "command consumer drained the policy feedback channel");
            check(read_only_motors.get_feedback_for_policy().position_rad == policy_snapshot.position_rad &&
                  read_only_motors.get_feedback_for_policy().timestamp_ns == policy_snapshot.timestamp_ns,
                  "policy getter did not return the cached snapshot");

            check(motors.validate_feedback_timing_for_command(1'000'000'000, deadline_ns, error) &&
                  error.empty() && deadline_ns == 1'000'000'000 + max_age_ns,
                  "feedback deadline ignores the oldest motor or sensor configuration");
            const auto original_deadline_ns = deadline_ns;
            error = "previous error";
            check(motors.refresh_feedback_for_policy(error) && error.empty() &&
                  motors.get_feedback_for_policy().position_rad == policy_snapshot.position_rad &&
                  motors.get_feedback_for_policy().timestamp_ns == policy_snapshot.timestamp_ns,
                  "policy refresh without a new frame changed the cache or retained an error");
            check(motors.refresh_feedback_for_command(error) && error.empty() &&
                  motors.get_feedback_for_command().timestamp_ns == 1'000'000'000 &&
                  motors.validate_feedback_timing_for_command(deadline_ns - 1, deadline_ns, error) &&
                  deadline_ns == original_deadline_ns,
                  "refresh without a new frame changed or renewed cached feedback");

            // Simulate a new frame arriving while a command uses the cached frame.
            // Validation must still expire the frame used by that calculation.
            for (int i = 0; i < config.motor.num_motors; ++i) {
                feedback[i].host_timestamp_ns += 100'000'000;
                feedback[i].position_rad += 0.1;
            }
            controller.publish_feedback(feedback);
            const auto& command_snapshot = motors.get_feedback_for_command();
            check(read_only_motors.get_feedback_for_policy().position_rad == policy_snapshot.position_rad &&
                  motors.feedback_for_policy_.snapshot.timestamp_ns == 1'000'000'000 &&
                  command_snapshot.timestamp_ns == 1'000'000'000 &&
                  command_snapshot.position_rad == policy_snapshot.position_rad,
                  "policy getter consumed a newly published frame or changed its cache");
            check(!motors.validate_feedback_timing_for_command(
                      original_deadline_ns, deadline_ns, error),
                  "feedback accepted at its deadline or silently refreshed during validation");
            const auto max_age_us = std::to_string(max_age_ns / 1000);
            check(error == "motor feedback expired: motor_index=4, feedback_age_us=" +
                      max_age_us + ", max_motor_age_us=" + max_age_us,
                  "feedback timeout diagnostic changed");
            check(!motors.validate_feedback_timing_for_command(
                      original_deadline_ns + 1, deadline_ns, error),
                  "cached feedback accepted after expiry");

            check(motors.refresh_feedback_for_command(error) && error.empty() &&
                  motors.get_feedback_for_command().timestamp_ns == 1'100'000'000 &&
                  motors.validate_feedback_timing_for_command(1'100'000'000, deadline_ns, error) &&
                  deadline_ns == 1'100'000'000 + max_age_ns,
                  "new feedback did not replace the expired cache");
            check(motors.feedback_for_policy_.snapshot.timestamp_ns == 1'000'000'000 &&
                  motors.feedback_for_policy_.snapshot.position_rad[4] == policy_snapshot.position_rad[4],
                  "command refresh changed the previously consumed policy snapshot");
            check(motors.refresh_feedback_for_policy(error) && error.empty() &&
                  motors.get_feedback_for_policy().position_rad[4] == feedback[4].position_rad &&
                  motors.feedback_for_policy_.snapshot.timestamp_ns == 1'100'000'000 &&
                  motors.get_feedback_for_command().timestamp_ns == 1'100'000'000,
                  "policy refresh changed or could not independently consume feedback");
            check(read_only_motors.get_feedback_for_policy().position_rad[4] == feedback[4].position_rad,
                  "policy getter did not expose the refreshed cache");

            feedback[4].host_timestamp_ns = 0;
            controller.publish_feedback(feedback);
            check(motors.refresh_feedback_for_command(error) &&
                  !motors.validate_feedback_timing_for_command(1'100'000'000, deadline_ns, error) &&
                  error.find("motor feedback timestamp missing: motor_index=4") == 0,
                  "missing feedback timestamp accepted");
            check(motors.refresh_feedback_for_policy(error) &&
                  motors.get_feedback_for_policy().timestamp_ns == 0,
                  "policy snapshot hid a missing motor timestamp");
            feedback[4].host_timestamp_ns = 1'100'000'000;
            controller.publish_feedback(feedback);
            check(motors.refresh_feedback_for_command(error) &&
                  motors.validate_feedback_timing_for_command(1'100'000'000, deadline_ns, error) &&
                  error.empty() && motors.is_initialized(),
                  "feedback did not recover after a valid frame");
            check(motors.refresh_feedback_for_policy(error) &&
                  motors.get_feedback_for_policy().timestamp_ns == 1'100'000'000,
                  "policy snapshot did not recover after a valid frame");

            check(motors.shutdown(motors.request_stop()) == ShutdownResult::Confirmed &&
                  !motors.feedback_for_command_.has_feedback &&
                  motors.get_feedback_for_command().timestamp_ns == 0 &&
                  !motors.refresh_feedback_for_command(error) &&
                  !motors.validate_feedback_timing_for_command(1'100'000'000, deadline_ns, error),
                  "released session retained usable command feedback");
            check(read_only_motors.get_feedback_for_policy().timestamp_ns == 0 &&
                  read_only_motors.get_feedback_for_policy().position_rad[4] == 0.0 &&
                  !motors.feedback_for_policy_.has_feedback &&
                  !motors.refresh_feedback_for_policy(error),
                  "policy getter exposed feedback after controller release");
            auto& next_controller = install_feedback_controller(motors);
            check(!motors.refresh_feedback_for_command(error),
                  "new controller reused feedback from the previous controller");
            check(!motors.refresh_feedback_for_policy(error) &&
                  motors.get_feedback_for_policy().timestamp_ns == 0,
                  "new controller reused policy feedback from the previous controller");
            next_controller.publish_feedback(feedback);
            check(motors.refresh_feedback_for_command(error) &&
                  motors.validate_feedback_timing_for_command(1'100'000'000, deadline_ns, error),
                  "new controller could not populate the cleared command cache");
            check(motors.refresh_feedback_for_policy(error) &&
                  motors.get_feedback_for_policy().timestamp_ns == 1'100'000'000,
                  "new controller could not populate the cleared policy cache");
        }
    }

    static void verify_motor_health()
    {
        for (const bool has_target : {false, true}) {
            RobotInterfaceConfig config;
            RobotInterface robot(config);
            auto& motors = robot.motor_session_;
            std::string error = "previous error";
            check(!motors.check_health(error) && error == "motors are not initialized",
                  "uninitialized session reported healthy");
            auto& controller = install_feedback_controller(motors);
            check(motors.check_health(error) && error.empty(),
                  "health query requires a feedback frame");
            std::array<motor_base::MotorStatusSnapshot, motor_base::kMotorNum> feedback{};
            const auto now_ns = robot_base::monotonic_now_ns();
            for (std::size_t i = 0; i < feedback.size(); ++i) {
                feedback[i].motor_index = static_cast<int>(i);
                feedback[i].host_timestamp_ns = now_ns;
                // These driver flags must not be reinterpreted by the session.
                feedback[i].faulted = true;
            }
            controller.publish_feedback(feedback);
            check(motors.check_health(error) && error.empty() &&
                  !motors.feedback_for_command_.has_feedback &&
                  !motors.feedback_for_policy_.has_feedback,
                  "health query consumed feedback or rechecked motor flags");
            check(motors.refresh_feedback_for_command(error), "health test feedback missing");
            std::int64_t deadline_ns = 0;
            check(motors.validate_feedback_timing_for_command(now_ns, deadline_ns, error),
                  "health test feedback is not fresh");
            check(robot.initialize_processors(), "health test processors failed");
            robot.policy_command_worker_ = std::make_unique<PolicyCommandWorker>(
                robot.config_, motors, *robot.action_processor_,
                robot.first_policy_inference_started_ns_);
            auto& worker = *robot.policy_command_worker_;
            check(controller.latch_terminal_fault(), "failed to inject terminal fault");
            check(!motors.check_health(error) &&
                  error == "motor controller terminal fault is latched",
                  "session failed to report the driver's terminal fault");
            check(motors.refresh_feedback_for_command(error) && error.empty() &&
                  motors.refresh_feedback_for_policy(error) && error.empty() &&
                  motors.validate_feedback_timing_for_command(now_ns, deadline_ns, error),
                  "feedback refresh or timing unexpectedly checks motor health");
            check(!worker.start({}) && !worker.is_running() &&
                  !worker.worker_thread_.joinable(),
                  "worker started after a terminal fault");
            motor_base::ControlCommand command;
            check(!motor_base::MotorControllerTimingTestAccess::consume_setpoint(controller, command),
                  "faulted worker start published a hold command");
            if (has_target) {
                PolicyTargetFrame target;
                InferenceRecord record;
                worker.publish_target(target, &record);
            }
            worker.running_.store(true);
            check(!worker.check_health(error) &&
                  error == "motor controller terminal fault is latched" && worker.is_running(),
                  "worker health query missed the fault or mutated running state");
            // Run synchronously: the first loop must fail even before the first target.
            worker.run();
            check(!worker.is_running() && worker.failed_.load() &&
                  !worker.check_health(error) &&
                  error == "motor controller terminal fault is latched" &&
                  !motor_base::MotorControllerTimingTestAccess::consume_setpoint(controller, command),
                  "worker did not stop on a terminal fault or published a command");
            const auto fault_stop_id =
                motor_base::MotorControllerTimingTestAccess::latest_stop(controller);
            const auto stop = motors.request_stop();
            check(fault_stop_id != 0 && stop.command_id == fault_stop_id,
                  "worker fault did not request STOP");
            // Policy must also see the driver's fault before the worker records it.
            worker.failed_.store(false);
            worker.running_.store(true);
            std::ostringstream diagnostic;
            auto* previous_buffer = std::cerr.rdbuf(diagnostic.rdbuf());
            const bool stepped = robot.policy_step();
            std::cerr.rdbuf(previous_buffer);
            check(!stepped && diagnostic.str().find(error) != std::string::npos,
                  "policy did not propagate the motor fault");
            check(robot.shutdown() == ShutdownResult::Confirmed &&
                  !motors.check_health(error) && error == "motors are not initialized",
                  "released session still reported healthy");
        }
    }

    static void verify_action_snapshot()
    {
        RobotInterfaceConfig config;
        const JointMapping mapping(static_cast<int>(kDof), config.joint_mapping);
        ActionProcessor ready_processor(config, mapping);
        ActionProcessor disabled_processor(config, mapping);
        MotorStateSnapshot feedback;
        std::string error;
        check(ready_processor.build_motor_position_targets(
                  config.default_joint_pos_rad, feedback.position_rad, error),
              "failed to build neutral motor feedback");
        feedback.velocity_rad_s.fill(0.01);
        feedback.comm_ok.fill(1);
        feedback.enabled.fill(1);
        PolicyMotorCommand ready_command;
        check(ready_processor.build_policy_motor_command(
                  config.default_joint_pos_rad, feedback, ready_command, error),
              "valid snapshot could not build a motor command");
        feedback.comm_ok.fill(0);
        feedback.enabled.fill(0);
        PolicyMotorCommand disabled_command;
        check(disabled_processor.build_policy_motor_command(
                  config.default_joint_pos_rad, feedback, disabled_command, error),
              "action calculation still checks communication or enable flags");
        for (std::size_t i = 0; i < ready_command.setpoints.size(); ++i) {
            const auto& expected = ready_command.setpoints[i];
            const auto& actual = disabled_command.setpoints[i];
            check(actual.position_rad == expected.position_rad &&
                  actual.velocity_rad_s == expected.velocity_rad_s &&
                  actual.effort_ff == expected.effort_ff &&
                  actual.kp == expected.kp && actual.kd == expected.kd &&
                  std::isfinite(actual.effort_ff),
                  "motor flags changed action calculation");
        }
        const auto ankle_motor = static_cast<std::size_t>(mapping.left_ankle().upper_motor_index);
        const auto neutral_feedback = feedback;
        feedback.position_rad[ankle_motor] = config.ankle.motor_pos_max_rad[0] + 0.1;
        check(!disabled_processor.build_policy_motor_command(
                  config.default_joint_pos_rad, feedback, disabled_command, error) &&
                  error.find("feedback ankle motor") == 0,
              "snapshot migration bypassed ankle position limits");
        feedback = neutral_feedback;
        feedback.position_rad[ankle_motor] = std::numeric_limits<double>::quiet_NaN();
        check(!disabled_processor.build_policy_motor_command(
                  config.default_joint_pos_rad, feedback, disabled_command, error),
              "snapshot migration bypassed invalid ankle calculation rejection");
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
    Access::verify_shutdown_with_base_controller();
    expect_destructor_termination(
        [] { Access::verify_destructor_shutdown_failure(false); },
        "[RobotMotorSession] Fatal: destructor shutdown failed;");
    expect_destructor_termination(
        [] { Access::verify_destructor_shutdown_failure(true); },
        "[RobotInterface] Fatal: destructor shutdown failed;");
    Access::verify_target_submission_after_stop();
    Access::verify_impedance_motor_count();
    Access::verify_startup_hold_modes();
    Access::verify_discrete_command_wait();
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
        inference::RobotMotorSession motors(config);
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
    Access::verify_motor_health();
    Access::verify_action_snapshot();
}
