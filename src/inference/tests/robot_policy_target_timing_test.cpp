#include "config/deploy_config.hpp"
#include "driver/myact/myact_motor_controller.hpp"
#include "protocol/xsens_mti/can_parser.hpp"
#include "robot/action_processor.hpp"
#include "robot/observation_builder.hpp"
#include "robot/robot_interface.hpp"

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kStart = 1'000'000'000;

class FakeAdapter : public myactua::EthercatAdapter {
public:
    bool init() override { return true; }
    void receive_physical() override {}
    void send_physical() override {}
    void send(int, const myactua::TxPDO&) override {}
    myactua::RxPDO receive(int) override { return {}; }
    bool is_configured(int) override { return true; }
    myactua::EthercatBusHealthSnapshot get_bus_health() const override
    {
        return {true, EC_WC_COMPLETE, 12};
    }
};

class BlockingErrorBuffer : public std::streambuf {
public:
    bool wait_blocked()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, std::chrono::seconds(1), [this] { return blocked_; });
    }

    void release()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        released_ = true;
        cv_.notify_all();
    }

protected:
    std::streamsize xsputn(const char*, std::streamsize count) override
    {
        std::unique_lock<std::mutex> lock(mutex_);
        blocked_ = true;
        cv_.notify_all();
        cv_.wait(lock, [this] { return released_; });
        return count;
    }

    int_type overflow(int_type character) override
    {
        const char value = traits_type::to_char_type(character);
        xsputn(&value, 1);
        return traits_type::not_eof(character);
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    bool blocked_{false};
    bool released_{false};
};

void expect(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

inference::RobotInterfaceConfig make_config()
{
    inference::RobotInterfaceConfig config;
    std::string error;
    expect(inference::load_deploy_config(ROBOT_DEPLOY_CONFIG_PATH, config, error), error);
    config.recorder.enabled = false;
    config.policy.gait.enabled = true;
    config.policy.gait.period = 1.0;
    return config;
}

std::vector<std::string> cells(const std::string& line)
{
    std::vector<std::string> result;
    std::stringstream stream(line);
    std::string cell;
    while (std::getline(stream, cell, ',')) result.push_back(cell);
    return result;
}

std::size_t column(const std::vector<std::string>& header, const std::string& name)
{
    const auto it = std::find(header.begin(), header.end(), name);
    expect(it != header.end(), "missing CSV column " + name);
    return static_cast<std::size_t>(it - header.begin());
}

}  // namespace

namespace motor_base {

struct MotorControllerTimingTestAccess {
    using Feedback = std::array<MotorStatusSnapshot, kMaxMotorCommandSetpoints>;

    static void publish(MotorControllerBase& controller, const Feedback& feedback)
    {
        controller.publish_feedback(feedback);
    }

    static bool consume(MotorControllerBase& controller, ControlCommand& command)
    {
        return controller.setpoint_channel_policy_.try_consume_latest(command);
    }

    static bool stop_requested(const MotorControllerBase& controller)
    {
        return controller.latest_stop_id(0) != 0;
    }
};

}  // namespace motor_base

namespace inference {

// Exercise the production admission, publication, completion and deadline paths.
// No model load, EtherCAT acquisition, IMU reader or live motor motion is needed.
struct RobotInterfacePolicyTimingTestAccess {
    using Target = robot_detail::PolicyTargetFrame;
    using PolicyStepPhase = RobotInterface::PolicyStepPhase;
    using Admission = RobotInterface::PolicyResultAdmission;
    using MotorAccess = motor_base::MotorControllerTimingTestAccess;
    using Feedback = MotorAccess::Feedback;

    static void prepare(RobotInterface& robot)
    {
        expect(robot.initialize_model_processors(), "model processor setup failed");
        robot.worker_.action_processor_ = robot.action_processor_.get();
        robot.initialize_policy_runtime_state();
    }

    static std::uint64_t begin(RobotInterface& robot, std::int64_t now)
    {
        return robot.begin_policy_inference(now);
    }

    static Admission admit(RobotInterface& robot, std::int64_t obs, std::int64_t now)
    {
        return robot.admit_policy_result(obs, now);
    }

    static InferenceRecord record(RobotInterface& robot, std::uint64_t seq,
                                  std::int64_t obs, Admission admission, float action)
    {
        InferenceRecord record;
        record.frame_index = robot.observation_builder_->frame_index();
        record.policy_seq = seq;
        record.policy_observation_time_ns = obs;
        record.obs_to_action_age_us = admission.obs_to_action_age_us;
        record.target_hold_age_us = admission.target_hold_age_us;
        record.policy_result_dropped = admission.dropped;
        record.raw_action.fill(action);
        record.target_q_model_rad.fill(action);
        return record;
    }

    static Target publish(RobotInterface& robot, const InferenceRecord& record)
    {
        expect(!record.policy_result_dropped, "test attempted to publish a drop");
        Target target;
        target.policy_seq = record.policy_seq;
        target.inference_record = record;
        const auto before_publish = steady_now_ns();
        robot.last_policy_target_published_ns_ = robot.worker_.publish_target(target);
        const auto after_publish = steady_now_ns();
        robot.observation_builder_->commit_policy_action(record.raw_action);
        robot.observation_builder_->advance_frame();
        robot.record_completed_policy_frame();
        robot.set_policy_step_phase(PolicyStepPhase::Idle);
        Target consumed;
        expect(robot.worker_.target_channel_.try_consume_latest(consumed), "target not published");
        expect(consumed.published_at_ns >= before_publish && consumed.published_at_ns <= after_publish &&
                   robot.last_policy_target_published_ns_ == consumed.published_at_ns &&
                   consumed.valid_until_ns == consumed.published_at_ns + 60*kMs &&
                   consumed.inference_record.policy_valid_until_ns == consumed.valid_until_ns,
               "publication lost its actual timestamp or target deadline");
        expect(consumed.inference_record.target_q_model_rad == record.target_q_model_rad &&
                   consumed.inference_record.raw_action == record.raw_action,
               "publication lost target or inference data");
        return consumed;
    }

    static void drop(RobotInterface& robot, const InferenceRecord& record)
    {
        expect(record.policy_result_dropped, "test drop record is not stale");
        robot.observation_builder_->advance_frame();
        robot.record_completed_policy_frame();
        robot.record_policy_frame(record);
        robot.set_policy_step_phase(PolicyStepPhase::Idle);
        Target consumed;
        expect(!robot.worker_.target_channel_.try_consume_latest(consumed), "drop refreshed channel");
    }

    static PolicyObservation observation(RobotInterface& robot)
    {
        MotorStateSnapshot motor;
        motor.position_rad.assign(12, 0.0);
        motor.velocity_rad_s.assign(12, 0.0);
        AhrsStateSnapshot imu;
        imu.projected_gravity_valid = true;
        imu.projected_gravity = {0.0, 0.0, -1.0};
        PolicyObservation obs;
        std::string error;
        expect(robot.observation_builder_->build(motor, imu, {0.2, 0.0, 0.0}, obs, error), error);
        return obs;
    }

    static motor_base::CommandTiming command(RobotInterface& robot, const Target& target,
                                             std::int64_t now)
    {
        return robot.worker_.policy_command_timing(target, now, now);
    }

    static void prepare_motor_feedback(RobotInterface& robot)
    {
        myactua::MyActMotorController::Options options;
        options.rt_thread_options.scheduling_policy = robot_base::ThreadSchedulingPolicy::OTHER;
        options.rt_thread_options.priority = 0;
        robot.motor_session_.controller_ = std::make_unique<myactua::MyActMotorController>(
            std::make_shared<FakeAdapter>(), robot.config_.motor.num_motors, options);
        robot.motor_session_.initialized_.store(true);
        robot.motor_session_.motion_enabled_.store(true);
    }

    static Feedback feedback_at(std::int64_t timestamp)
    {
        Feedback feedback{};
        for (std::size_t i = 0; i < policy_observation::kDof; ++i) {
            feedback[i].motor_index = static_cast<int>(i);
            feedback[i].host_timestamp_ns = timestamp;
            feedback[i].comm_ok = true;
            feedback[i].enabled = true;
            feedback[i].control_ready = true;
        }
        return feedback;
    }

    static bool expired(const Target& target, std::int64_t now)
    {
        return robot_detail::PolicyCommandWorker::policy_deadline_expired(
            target.valid_until_ns, now);
    }

    static void run_expired_worker(RobotInterface& robot, const Target* target = nullptr)
    {
        if (target) robot.worker_.target_channel_.publish(*target);
        robot.worker_.running_.store(true);
        robot.initialized_.store(true);
        robot.set_policy_step_phase(PolicyStepPhase::Inference);
        robot.worker_.loop();
        expect(robot.worker_.failed_.load() && !robot.worker_.running_.load() &&
                   !robot.is_initialized(),
               "expired worker did not fail/stop");
        expect(robot.worker_.error_.find("target_hold_age_us=") != std::string::npos &&
                   robot.worker_.error_.find("target_age_us=") == std::string::npos &&
                   robot.worker_.error_.find("policy_seq=") != std::string::npos &&
                   robot.worker_.error_.find("obs_to_action_age_us=") != std::string::npos &&
                   robot.worker_.error_.find("overdue_us=") != std::string::npos &&
                   robot.worker_.error_.find("policy_step_phase=") == std::string::npos &&
                   robot.worker_.error_.find("phase_elapsed_us=") == std::string::npos,
               "deadline error lost target diagnostics or retained Interface phase data");
    }

    static void test_sequences_and_state()
    {
        RobotInterface robot(make_config());
        prepare(robot);
        observation(robot);
        const auto seq1 = begin(robot, kStart);
        const auto target1 = publish(robot, record(robot, seq1, kStart,
            admit(robot, kStart, kStart + 25*kMs), 0.1F));
        expect(target1.policy_seq == 1 && target1.inference_record.policy_seq == 1,
               "first sequence mismatch");
        observation(robot);  // Keep the history assembled for the dropped policy step.
        const auto seq2 = begin(robot, kStart + 25*kMs);
        const auto stale = record(robot, seq2, kStart + 5*kMs,
            admit(robot, kStart + 5*kMs, kStart + 50*kMs), 0.9F);
        drop(robot, stale);
        expect(stale.policy_seq == 2 && !stale.command_applied,
               "drop lost its policy step identity");
        expect(robot.next_policy_seq_ == 3 &&
                   robot.last_policy_target_published_ns_ == target1.published_at_ns,
               "drop lost its policy sequence or renewed the hold");
        const auto next_obs = observation(robot);
        for (std::size_t i = 0; i < policy_observation::kDof; ++i) {
            expect(next_obs[next_obs.size() - policy_observation::kDof + i] == 0.1F,
                   "drop committed the stale raw action");
        }
        constexpr std::size_t gait_current = 3*3*15 + 2*(15-1);
        expect(std::abs(next_obs[gait_current] - std::sin(2*6.283185307179586*0.02)) < 1e-6 &&
                   robot.observation_builder_->frame_index() == 2,
               "drop failed to advance frame/gait time");
        const auto seq3 = begin(robot, kStart + 50*kMs);
        const auto target2 = publish(robot, record(robot, seq3, kStart + 50*kMs,
            admit(robot, kStart + 50*kMs, kStart + 75*kMs), 0.3F));
        expect(target2.policy_seq == 3 && target2.inference_record.policy_seq == 3,
               "recovery did not preserve the policy sequence gap after a drop");
        const auto early = command(robot, target2, target2.published_at_ns + kMs);
        const auto late = command(robot, target2, target2.published_at_ns + 59*kMs);
        expect(early.source_policy_seq == 3 && late.source_policy_seq == 3 &&
                   early.valid_until_ns == target2.published_at_ns + 11*kMs &&
                   late.valid_until_ns == target2.valid_until_ns,
               "hold changed the source policy or extended the deadline");
        expect(!expired(target2, target2.valid_until_ns - 1) && expired(target2, target2.valid_until_ns),
               "60ms deadline equality must expire");
        robot.reset_policy_command_state();
        expect(robot.observation_builder_->frame_index() == 3,
               "worker startup must not reset existing model processor state");
        robot.worker_.target_channel_.publish(target2);
        robot.worker_.completed_record_channel_.publish(target2.inference_record);
        robot.initialize_policy_runtime_state();
        Target ignored;
        InferenceRecord ignored_record;
        expect(robot.next_policy_seq_ == 1 &&
                   robot.last_policy_target_published_ns_ == 0 &&
                   robot.first_policy_inference_started_ns_.load() == 0 &&
                   robot.observation_builder_->frame_index() == 0 &&
                   !robot.worker_.target_channel_.try_consume_latest(ignored) &&
                   !robot.worker_.try_consume_completed_record(ignored_record),
               "reinitialization did not clear runtime state");
    }

    static void test_admission_and_logging_delay()
    {
        RobotInterface robot(make_config());
        prepare(robot);
        const auto observation_time = steady_now_ns() - 45*kMs;
        const auto seq = begin(robot, observation_time);
        const auto at_boundary = admit(robot, observation_time, observation_time + 40*kMs);
        expect(!at_boundary.dropped && at_boundary.obs_to_action_age_us == 40000 &&
                   at_boundary.target_hold_age_us == 40000, "40ms must be admitted");
        expect(admit(robot, observation_time, observation_time + 40*kMs + 1).dropped,
               "admission must compare nanoseconds");
        // Synthetic 5ms spent assembling logs AFTER admission: do not re-admit.
        const auto target = publish(robot, record(robot, seq, observation_time, at_boundary, 0.1F));
        expect(target.published_at_ns >= observation_time + 45*kMs &&
                   target.inference_record.obs_to_action_age_us == 40000 &&
                   target.inference_record.target_hold_age_us == 40000,
               "logging delay contaminated admission or publication time");
        // Same observation timestamp may yield another valid target within 40ms.
        robot.initialize_policy_runtime_state();
        const auto first_seq = begin(robot, kStart);
        publish(robot, record(robot, first_seq, kStart,
            admit(robot, kStart, kStart + 20*kMs), 0.1F));
        const auto second_seq = begin(robot, kStart + 20*kMs);
        const auto same_obs = publish(robot, record(robot, second_seq, kStart,
            admit(robot, kStart, kStart + 39*kMs), 0.2F));
        expect(same_obs.policy_seq == 2 && same_obs.inference_record.policy_seq == 2,
               "duplicate observation was rejected");
    }

    static void test_sensors_and_startup()
    {
        RobotInterface robot(make_config());
        prepare(robot);
        std::string error;
        expect(!robot.validate_policy_sensor_timing(0, kStart, kStart, error), "missing motor timestamp accepted");
        expect(!robot.validate_policy_sensor_timing(kStart, 0, kStart, error), "missing IMU timestamp accepted");
        expect(robot.validate_policy_sensor_timing(kStart + 1, kStart, kStart, error),
               "a sensor sampled after now must not be rejected for negative age");
        expect(robot.validate_policy_sensor_timing(kStart, kStart + 1, kStart, error),
               "a sensor sampled after now must not be rejected for negative age");
        expect(robot.config_.sensor_guard.max_motor_sample_age_s == 0.020 &&
                   robot.config_.sensor_guard.max_imu_sample_age_s == 0.050 &&
                   robot.config_.sensor_guard.max_sensor_state_skew_s == 0.030,
               "sensor defaults changed outside the motor age limit");
        expect(robot.validate_policy_sensor_timing(kStart, kStart, kStart + 20*kMs, error), "20ms motor boundary rejected");
        expect(!robot.validate_policy_sensor_timing(kStart, kStart, kStart + 20*kMs + 1, error), "20ms motor guard relaxed");
        // Isolate the IMU age guard from the tighter motor age and skew guards.
        auto imu_config = make_config();
        imu_config.sensor_guard.max_sensor_state_skew_s = 0.060;
        RobotInterface imu_robot(imu_config);
        expect(imu_robot.validate_policy_sensor_timing(kStart + 50*kMs, kStart, kStart + 50*kMs, error), "50ms IMU boundary rejected");
        expect(!imu_robot.validate_policy_sensor_timing(kStart + 50*kMs + 1, kStart, kStart + 50*kMs + 1, error), "50ms IMU guard relaxed");
        // A controlled stale parser snapshot: no newer measurements are fed.
        imu::XsensMtiCanParser parser;
        const std::uint8_t quaternion[] = {0x7F, 0xFF, 0, 0, 0, 0, 0, 0};
        const std::uint8_t rate[] = {0x02, 0, 0, 0, 0, 0};
        parser.feed(imu::XCDI_QUATERNION_ID, quaternion, 8, kStart);
        parser.feed(imu::XCDI_RATE_OF_TURN_ID, rate, 6, kStart + kMs);
        imu_base::AHRSData stale_ahrs;
        expect(parser.get_ahrs_data(stale_ahrs), "controlled stale AHRS was not published");
        auto stale_config = make_config();
        stale_config.sensor_guard.max_sensor_state_skew_s = 0.200;
        RobotInterface stale_robot(stale_config);
        const auto checked_at = kStart + 100*kMs;
        expect(stale_ahrs.receive_timestamp_ns == kStart &&
                   !stale_robot.validate_policy_sensor_timing(
                       checked_at, stale_ahrs.receive_timestamp_ns, checked_at, error),
               "default 50ms IMU age guard accepted a 100ms-old assembled AHRS");
        expect(robot.validate_policy_sensor_timing(kStart + 30*kMs, kStart, kStart + 30*kMs, error), "30ms skew rejected");
        expect(!robot.validate_policy_sensor_timing(kStart + 30*kMs + 1, kStart, kStart + 30*kMs + 1, error), "skew guard relaxed");
        expect(!robot.worker_.startup_policy_target_expired(kStart + 1000*kMs), "startup timer armed before inference");
        begin(robot, kStart);
        begin(robot, kStart + 25*kMs);
        expect(robot.worker_.startup_policy_deadline_ns() == kStart + 60*kMs &&
                   !robot.worker_.startup_policy_target_expired(kStart + 60*kMs - 1) &&
                   robot.worker_.startup_policy_target_expired(kStart + 60*kMs),
               "startup timer renewed or missed equality");
        const auto normal = robot.motor_session_.target_command_timing(kStart, 0);
        const auto near_deadline = robot.motor_session_.target_command_timing(
            kStart + 59*kMs, robot.worker_.startup_policy_deadline_ns());
        expect(normal.source_policy_seq == 0 && normal.valid_until_ns == kStart + 10*kMs &&
                   near_deadline.valid_until_ns == kStart + 60*kMs, "startup command not capped");
        run_expired_worker(robot);  // First inference blocked beyond 60ms, no hardware needed.
    }

    static void test_oldest_motor_snapshot()
    {
        RobotInterface robot(make_config());
        prepare_motor_feedback(robot);
        auto feedback = feedback_at(kStart + 10*kMs);
        feedback[7].host_timestamp_ns = kStart;
        MotorAccess::publish(*robot.motor_session_.controller_, feedback);
        auto snapshot = robot.motor_session_.get_motor_snapshot();
        expect(snapshot.timestamp_ns == kStart && snapshot.position_rad.size() == 12,
               "motor snapshot ignored an older non-first axis");
        feedback[7].host_timestamp_ns = 0;
        MotorAccess::publish(*robot.motor_session_.controller_, feedback);
        snapshot = robot.motor_session_.get_motor_snapshot();
        expect(snapshot.timestamp_ns == 0, "zero axis timestamp was excluded from the oldest sample");
        expect(robot.motor_session_.get_motor_snapshot().timestamp_ns == 0,
               "cached snapshot renewed the missing axis timestamp");
        feedback[7].host_timestamp_ns = kStart + 20*kMs;
        MotorAccess::publish(*robot.motor_session_.controller_, feedback);
        expect(robot.motor_session_.get_motor_snapshot().timestamp_ns == kStart + 10*kMs,
               "snapshot did not recover with the oldest valid axis");
    }

    static void test_feedback_deadline()
    {
        RobotInterface robot(make_config());
        prepare(robot);
        prepare_motor_feedback(robot);
        Target target;
        target.policy_seq = 9;
        target.valid_until_ns = kStart + 60*kMs;
        const auto fresh = robot.worker_.policy_command_timing(target, kStart, kStart);
        const auto near_expiry = robot.worker_.policy_command_timing(target, kStart + 19*kMs, kStart);
        expect(fresh.valid_until_ns == kStart + 10*kMs &&
                   near_expiry.valid_until_ns == kStart + 20*kMs &&
                   near_expiry.source_policy_seq == 9,
               "feedback deadline extended the command budget or source policy");
        target.valid_until_ns = kStart + 19*kMs + 1;
        expect(robot.worker_.policy_command_timing(target, kStart + 19*kMs, kStart).valid_until_ns ==
                   target.valid_until_ns, "feedback cap extended a shorter target deadline");
        target.valid_until_ns = kStart + 60*kMs;
        expect(robot.worker_.policy_command_timing(target, kStart + 20*kMs - 1, kStart).is_well_formed(),
               "feedback deadline expired before its nanosecond boundary");

        // Exercise the real action build between injected pre/post times, as in
        // the existing admission/publication tests. Calculation crosses expiry.
        robot_detail::ActionProcessor::FixedPolicyMotorCommand built;
        std::string error;
        expect(robot.action_processor_->build_policy_impedance_command(
                   target.inference_record.target_q_model_rad, feedback_at(kStart), built, error), error);
        for (const auto produced : {kStart + 20*kMs, kStart + 20*kMs + 1}) {
            const auto expired_timing = robot.worker_.policy_command_timing(target, produced, kStart);
            expect(robot_detail::PolicyCommandWorker::policy_deadline_expired(
                       expired_timing.valid_until_ns, produced) &&
                       !expired_timing.is_well_formed(),
                   "calculation crossing feedback expiry produced usable timing");
            expect(!robot.motor_session_.apply_impedance_setpoints_realtime(
                       built.setpoints, built.setpoint_count, expired_timing),
                   "expired feedback timing was accepted for publication");
        }
        motor_base::ControlCommand submitted;
        expect(!MotorAccess::consume(*robot.motor_session_.controller_, submitted),
               "expired calculation published a motor command");
    }

    static void test_feedback_worker()
    {
        // Publish just one feedback frame: the worker must age its cache even
        // though the target remains valid. No hardware or RT producer is started.
        for (const int scenario : {0, 1, 2}) {
            RobotInterface robot(make_config());
            prepare(robot);
            prepare_motor_feedback(robot);
            const auto now = steady_now_ns();
            auto feedback = feedback_at(now);
            if (scenario == 1) feedback[3].host_timestamp_ns = now - 25*kMs;
            if (scenario == 2) feedback[3].host_timestamp_ns = 0;
            MotorAccess::publish(*robot.motor_session_.controller_, feedback);
            Target target;
            target.policy_seq = 1;
            target.published_at_ns = now;
            target.valid_until_ns = now + 1000*kMs; // Isolate feedback expiry from target expiry.
            robot.worker_.target_channel_.publish(target);
            robot.worker_.running_.store(true);
            robot.initialized_.store(true);
            robot.worker_.loop();
            const auto stopped_at = steady_now_ns();
            expect(robot.worker_.failed_.load() && !robot.worker_.running_.load() &&
                       !robot.is_initialized() &&
                       MotorAccess::stop_requested(*robot.motor_session_.controller_) &&
                       !robot.motor_session_.motion_enabled_.load(),
                   "bad feedback did not fail the worker and request STOP");
            const auto& error = robot.worker_.error_;
            expect(error.find(scenario == 2 ? "timestamp missing" : "feedback expired") != std::string::npos &&
                       error.find(scenario == 0 ? "motor_index=0" : "motor_index=3") != std::string::npos &&
                       error.find("feedback_age_us=") != std::string::npos &&
                       error.find("max_motor_age_us=20000") != std::string::npos,
                   "feedback failure lost its axis, age or limit diagnostics");
            motor_base::ControlCommand submitted;
            const bool has_command = MotorAccess::consume(*robot.motor_session_.controller_, submitted);
            if (scenario == 0) {
                expect(has_command && submitted.timing.produced_at_ns < now + 20*kMs &&
                           submitted.timing.valid_until_ns <= now + 20*kMs &&
                           submitted.timing.valid_until_ns <= submitted.timing.produced_at_ns + 10*kMs &&
                           stopped_at >= now + 20*kMs && stopped_at < target.valid_until_ns,
                       "cached feedback renewed its deadline or stopped only with the target");
                expect(!MotorAccess::consume(*robot.motor_session_.controller_, submitted),
                       "worker kept publishing commands after feedback failure");
            } else {
                expect(!has_command, "initially stale or missing feedback generated a command");
            }
        }
    }

    static void test_worker_lifecycle_and_blocked_diagnostics()
    {
        {
            RobotInterface robot(make_config());
            prepare(robot);
            prepare_motor_feedback(robot);
            std::string error;
            for (int restart = 0; restart < 2; ++restart) {
                // The temporary pose is destroyed when start returns. Later
                // refreshes must use the pose owned by the worker.
                expect(robot.worker_.start(*robot.action_processor_, std::vector<double>(12, 0.0)) &&
                           robot.worker_.healthy(error),
                       "extracted worker failed to start/restart");
                const auto started_at = steady_now_ns();
                const auto refresh_deadline = std::chrono::steady_clock::now() +
                    std::chrono::seconds(1);
                motor_base::ControlCommand held_command;
                bool refreshed = false;
                while (std::chrono::steady_clock::now() < refresh_deadline) {
                    if (MotorAccess::consume(*robot.motor_session_.controller_, held_command) &&
                        held_command.timing.produced_at_ns > started_at) {
                        refreshed = true;
                        break;
                    }
                    std::this_thread::yield();
                }
                robot.worker_.stop();
                expect(refreshed && held_command.payload_size == 12 &&
                           held_command.setpoint_type == motor_base::SetpointCommandType::IMPEDANCE_TARGETS &&
                           held_command.timing.source_policy_seq == 0 &&
                           std::all_of(held_command.impedance_setpoints.begin(),
                                       held_command.impedance_setpoints.begin() + held_command.payload_size,
                                       [](const auto& point) { return point.position_rad == 0.0; }),
                       "worker did not retain its startup pose after start returned");
                expect(!robot.worker_.healthy(error) && !robot.worker_.running_.load(),
                       "extracted worker failed to stop/join");
            }
        }

        for (const bool expired_feedback : {false, true}) {
            RobotInterface robot(make_config());
            prepare(robot);
            prepare_motor_feedback(robot);
            robot.initialized_.store(true);
            BlockingErrorBuffer buffer;
            const bool started = robot.worker_.start(*robot.action_processor_, std::vector<double>(12, 0.0));
            auto* original = std::cerr.rdbuf(&buffer);
            const auto now = steady_now_ns();
            Target target;
            target.policy_seq = 1;
            target.valid_until_ns = expired_feedback ? now + 1000*kMs : now - 1;
            if (expired_feedback) {
                MotorAccess::publish(*robot.motor_session_.controller_, feedback_at(now - 25*kMs));
            }
            robot.worker_.target_channel_.publish(target);
            const bool blocked = buffer.wait_blocked();
            const bool stopped_before_log = MotorAccess::stop_requested(*robot.motor_session_.controller_) &&
                !robot.is_initialized() && !robot.motor_session_.motion_enabled_.load() &&
                !robot.worker_.running_.load();
            buffer.release();
            robot.worker_.stop();
            std::cerr.rdbuf(original);
            expect(started && blocked && stopped_before_log && robot.worker_.failed_.load(),
                   "worker failure waited for blocked diagnostics before requesting STOP");
        }
    }

    static void test_interface_failure_diagnostics()
    {
        {
            RobotInterface robot(make_config());
            prepare(robot);
            prepare_motor_feedback(robot);
            const auto now = steady_now_ns();
            Target target;
            target.policy_seq = 7;
            target.published_at_ns = now - 100*kMs;
            target.valid_until_ns = now - 1;
            target.inference_record.obs_to_action_age_us = 25000;
            run_expired_worker(robot, &target);
            std::string worker_error;
            expect(!robot.worker_.healthy(worker_error), "expired worker appeared healthy");
            expect(worker_error.find("policy_seq=7") != std::string::npos &&
                       worker_error.find("obs_to_action_age_us=25000") != std::string::npos,
                   "worker lost the failed target's identity or observation age");

            // Interface diagnoses the phase at handling time, after inference returned.
            robot.set_policy_step_phase(PolicyStepPhase::PostInference);
            const auto phase_started = steady_now_ns() - 10*kMs;
            robot.policy_step_phase_started_ns_ = phase_started;
            std::ostringstream diagnostic;
            auto* original = std::cerr.rdbuf(diagnostic.rdbuf());
            const auto before = steady_now_ns();
            const bool result = robot.handle_policy_step_failure(worker_error);
            const auto after = steady_now_ns();
            std::cerr.rdbuf(original);

            const auto log = diagnostic.str();
            expect(!result && log.find("[RobotInterface] policy_step failed: " + worker_error) !=
                       std::string::npos &&
                       log.find("policy_step_phase=post_inference") != std::string::npos,
                   "Interface lost the worker fault or handling-time phase");
            const std::string elapsed_field = "phase_elapsed_us=";
            const auto elapsed_position = log.find(elapsed_field);
            expect(elapsed_position != std::string::npos, "Interface omitted phase elapsed time");
            const auto elapsed_us = std::stoll(log.substr(elapsed_position + elapsed_field.size()));
            expect(elapsed_us >= robot_base::ns_to_us(before - phase_started) &&
                       elapsed_us <= robot_base::ns_to_us(after - phase_started),
                   "Interface phase elapsed time did not use handling time");
            expect(robot.policy_step_phase_ == PolicyStepPhase::Idle &&
                       robot.policy_step_phase_started_ns_ >= before &&
                       robot.policy_step_phase_started_ns_ <= after,
                   "Interface did not return to Idle after handling failure");
            expect(robot.worker_.error_ == worker_error,
                   "Interface appended its diagnostic state to the worker fault");

            robot.set_policy_step_phase(PolicyStepPhase::Publish);
            robot.reset_policy_command_state();
            expect(robot.policy_step_phase_ == PolicyStepPhase::Idle &&
                       robot.policy_step_phase_started_ns_ == 0,
                   "runtime reset retained Interface phase diagnostics");
        }

        {
            RobotInterface robot(make_config());
            prepare(robot);
            prepare_motor_feedback(robot);
            robot.initialized_.store(true);
            const bool started = robot.worker_.start(*robot.action_processor_, std::vector<double>(12, 0.0));
            robot.set_policy_step_phase(PolicyStepPhase::PostInference);
            BlockingErrorBuffer buffer;
            auto* original = std::cerr.rdbuf(&buffer);
            bool result = true;
            std::thread control_thread([&] {
                result = robot.handle_policy_step_failure("injected Interface failure");
            });
            const bool blocked = buffer.wait_blocked();
            const bool stopped_before_log = MotorAccess::stop_requested(*robot.motor_session_.controller_) &&
                !robot.motor_session_.motion_enabled_.load() && !robot.worker_.is_running() &&
                !robot.is_initialized();
            buffer.release();
            control_thread.join();
            std::cerr.rdbuf(original);
            expect(started && blocked && stopped_before_log && !result &&
                       robot.policy_step_phase_ == PolicyStepPhase::Idle,
                   "Interface waited for blocked diagnostics before requesting STOP or worker exit");
        }
    }

    static void test_failure_publication_under_contention()
    {
        RobotInterface robot(make_config());
        prepare(robot);
        prepare_motor_feedback(robot);
        const auto now = steady_now_ns();
        Target target;
        target.policy_seq = 7;
        target.published_at_ns = now - 100*kMs;
        target.valid_until_ns = now - 1;
        robot.worker_.target_channel_.publish(target);
        robot.worker_.running_.store(true);
        robot.initialized_.store(true);

        bool stopped = false;
        bool published_before_error = false;
        {
            // Delay the diagnostic write while the real deadline failure runs.
            std::unique_lock<std::mutex> lock(robot.worker_.error_mutex_);
            robot.worker_.worker_thread_ = std::thread([&] { robot.worker_.loop(); });
            const auto stop_deadline = std::chrono::steady_clock::now() +
                std::chrono::seconds(2);
            while (robot.worker_.is_running() &&
                   std::chrono::steady_clock::now() < stop_deadline) {
                std::this_thread::yield();
            }
            stopped = !robot.is_initialized() &&
                MotorAccess::stop_requested(*robot.motor_session_.controller_);

            const auto observation_deadline = std::chrono::steady_clock::now() +
                std::chrono::milliseconds(20);
            do {
                published_before_error = robot.worker_.failed_.load();
                std::this_thread::yield();
            } while (!published_before_error &&
                     std::chrono::steady_clock::now() < observation_deadline);
        }
        robot.worker_.stop();
        std::string error;
        const bool healthy = robot.worker_.healthy(error);
        expect(stopped, "diagnostic contention delayed STOP or robot state update");
        expect(!published_before_error, "failure was published before its diagnostic");
        expect(!healthy && error.find("policy target deadline expired: policy_seq=7") !=
                   std::string::npos,
               "published failure lost its diagnostic");
    }

    static void test_replay_and_continuous_drops()
    {
        // Align recorded intervals with the actual publication time. Publication
        // was not logged: use inference end and first command as its two bounds.
        constexpr std::int64_t obs2694 = 3439582873849;
        constexpr std::int64_t start2694 = 3439584616586;
        constexpr std::int64_t end2694 = 3439609094228;
        constexpr std::int64_t cmd2694 = 3439609394650;
        constexpr std::int64_t obs2695 = 3439590719517;
        constexpr std::int64_t start2695 = 3439609147313;
        constexpr std::int64_t end2695 = 3439634645222;
        constexpr std::int64_t original_fault = obs2695 + 60*kMs + 669000;
        for (const auto published_at : {end2694, cmd2694}) {
            RobotInterface robot(make_config());
            prepare(robot);
            robot.next_policy_seq_ = 2694;
            auto seq = begin(robot, start2694);
            const auto held = publish(robot, record(robot, seq, obs2694,
                admit(robot, obs2694, end2694), 0.1F));
            const auto replay_offset = held.published_at_ns - published_at;
            seq = begin(robot, replay_offset + start2695);
            const auto stale = record(robot, seq, replay_offset + obs2695,
                admit(robot, replay_offset + obs2695, replay_offset + end2695), 0.9F);
            drop(robot, stale);
            const auto replay_fault = replay_offset + original_fault;
            expect(stale.policy_seq == 2695 && stale.obs_to_action_age_us == 43925 &&
                       !expired(held, replay_fault), "2695 replay failed to hold previous target");
            seq = begin(robot, replay_fault);
            const auto recovered = publish(robot, record(robot, seq, replay_fault,
                admit(robot, replay_fault, replay_fault + 5*kMs), 0.2F));
            expect(recovered.policy_seq == 2696 && recovered.inference_record.policy_seq == 2696,
                   "synthetic recovery lost its policy sequence");
        }
        RobotInterface robot(make_config());
        prepare(robot);
        auto seq = begin(robot, kStart);
        const auto held = publish(robot, record(robot, seq, kStart,
            admit(robot, kStart, kStart + 25*kMs), 0.1F));
        for (int i = 0; i < 3; ++i) {
            const auto now = kStart + (50 + i*25)*kMs;
            seq = begin(robot, now - 20*kMs);
            drop(robot, record(robot, seq, now - 45*kMs, admit(robot, now - 45*kMs, now), 0.9F));
        }
        expect(robot.last_policy_target_published_ns_ == held.published_at_ns,
               "continuous drops renewed the watchdog");
        std::this_thread::sleep_until(std::chrono::steady_clock::time_point(
            std::chrono::nanoseconds(held.valid_until_ns)));
        run_expired_worker(robot, &held);  // Also models a next inference stuck while holding.
    }

    static void test_drop_recorder_path()
    {
        auto config = make_config();
        config.recorder.enabled = true;
        const auto dir = std::filesystem::temp_directory_path() /
            ("b1-drop-" + std::to_string(steady_now_ns()));
        config.recorder.directory = dir;
        config.recorder.policy_observation_size = policy_observation::kObservationSizeWithGaitPhase;
        RobotInterface robot(config);
        prepare(robot);
        expect(robot.inference_recorder_.start(config.recorder), "recorder start failed");
        const auto seq = begin(robot, kStart);
        auto dropped = record(robot, seq, kStart, admit(robot, kStart, kStart + 45*kMs), 0.9F);
        dropped.inference_start_ns = kStart;
        dropped.inference_end_ns = kStart + 44*kMs;
        drop(robot, dropped);
        const auto path = robot.inference_recorder_.log_path();
        robot.inference_recorder_.stop();
        std::ifstream file(path);
        std::string header_line, data_line;
        expect(static_cast<bool>(std::getline(file, header_line)) &&
                   static_cast<bool>(std::getline(file, data_line)), "drop never reached recorder");
        const auto header = cells(header_line);
        const auto values = cells(data_line);
        expect(values[column(header, "policy_seq")] == "1" &&
                   values[column(header, "policy_result_dropped")] == "1" &&
                   values[column(header, "command_applied")] == "0" &&
                   values[column(header, "command_timestamp_ns")] == "0" &&
                   values[column(header, "policy_valid_until_ns")] == "0" &&
                   values[column(header, "obs_to_action_age_us")] == "45000" &&
                   values[column(header, "target_hold_age_us")] == "45000" &&
                   std::abs(std::stod(values[column(header, "raw_action_0")]) - 0.9) < 1e-6,
               "drop recorder lost step identity, candidate output or admission ages");
        std::filesystem::remove_all(dir);
    }
};

}  // namespace inference

int main()
{
    try {
        using Access = inference::RobotInterfacePolicyTimingTestAccess;
        Access::test_sequences_and_state();
        Access::test_admission_and_logging_delay();
        Access::test_sensors_and_startup();
        Access::test_oldest_motor_snapshot();
        Access::test_feedback_deadline();
        Access::test_feedback_worker();
        Access::test_worker_lifecycle_and_blocked_diagnostics();
        Access::test_interface_failure_diagnostics();
        Access::test_failure_publication_under_contention();
        Access::test_replay_and_continuous_drops();
        Access::test_drop_recorder_path();
        std::cout << "robot_policy_target_timing_test passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
