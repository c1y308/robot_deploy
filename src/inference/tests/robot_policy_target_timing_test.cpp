#include "config/deploy_config.hpp"
#include "robot/observation_builder.hpp"
#include "robot/robot_interface.hpp"

#include <algorithm>
#include <cmath>
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

namespace inference {

// Exercise the production admission, publication, completion and deadline paths.
// No model load, EtherCAT acquisition, IMU reader or live motor motion is needed.
struct RobotInterfacePolicyTimingTestAccess {
    using Target = RobotInterface::PolicyTargetFrame;
    using Admission = RobotInterface::PolicyResultAdmission;

    static void prepare(RobotInterface& robot)
    {
        expect(robot.initialize_model_processors(), "model processor setup failed");
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

    static Target publish(RobotInterface& robot, const InferenceRecord& record,
                           std::int64_t published_at)
    {
        expect(!record.policy_result_dropped, "test attempted to publish a drop");
        Target* staged = robot.policy_target_channel_.acquire_write_slot();
        staged->policy_seq = record.policy_seq;
        staged->observation_time_ns = record.policy_observation_time_ns;
        staged->target_q_model_rad = record.target_q_model_rad;
        staged->inference_record = record;
        robot.publish_policy_target(*staged, published_at);
        robot.complete_policy_result(staged->inference_record);
        Target consumed;
        expect(robot.policy_target_channel_.try_consume_latest(consumed), "target not published");
        return consumed;
    }

    static void drop(RobotInterface& robot, const InferenceRecord& record)
    {
        expect(record.policy_result_dropped, "test drop record is not stale");
        robot.complete_policy_result(record);
        Target consumed;
        expect(!robot.policy_target_channel_.try_consume_latest(consumed), "drop refreshed channel");
    }

    static PolicyObservation observation(RobotInterface& robot)
    {
        MotorStateSnapshot motor;
        motor.position_rad.assign(12, 0.0);
        motor.velocity_rad_s.assign(12, 0.0);
        AhrsStateSnapshot imu;
        imu.ahrs_ready = true;
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
        return robot.policy_command_timing(target, now);
    }

    static bool expired(const Target& target, std::int64_t now)
    {
        return RobotInterface::policy_deadline_expired(target.valid_until_ns, now);
    }

    static void run_expired_worker(RobotInterface& robot, const Target* target = nullptr)
    {
        if (target) robot.policy_target_channel_.publish(*target);
        robot.policy_command_worker_running_.store(true);
        robot.initialized_.store(true);
        robot.set_policy_step_phase(RobotInterface::PolicyStepPhase::Inference);
        robot.policy_command_worker_loop();
        expect(robot.policy_command_worker_failed_.load() && !robot.initialized_.load() &&
                   !robot.policy_command_worker_running_.load(), "expired worker did not fail/stop");
        expect(robot.policy_command_worker_error_.find("target_hold_age_us=") != std::string::npos &&
                   robot.policy_command_worker_error_.find("target_age_us=") == std::string::npos &&
                   robot.policy_command_worker_error_.find("policy_step_phase=inference") != std::string::npos,
               "deadline error lost hold age or inference diagnostics");
    }

    static void test_sequences_and_state()
    {
        RobotInterface robot(make_config());
        prepare(robot);
        observation(robot);
        const auto seq1 = begin(robot, kStart);
        const auto target1 = publish(robot, record(robot, seq1, kStart,
            admit(robot, kStart, kStart + 25*kMs), 0.1F), kStart + 25*kMs);
        expect(target1.policy_seq == 1 && target1.target_seq == 1, "first sequence mismatch");
        observation(robot);  // Keep the history assembled for the dropped policy step.
        const auto seq2 = begin(robot, kStart + 25*kMs);
        const auto stale = record(robot, seq2, kStart + 5*kMs,
            admit(robot, kStart + 5*kMs, kStart + 50*kMs), 0.9F);
        drop(robot, stale);
        expect(stale.policy_seq == 2 && stale.target_seq == 0 && !stale.command_applied,
               "drop lost its policy step identity");
        expect(robot.next_policy_seq_ == 3 && robot.next_target_seq_ == 2 &&
                   robot.stale_policy_drop_count_ == 1 &&
                   robot.last_policy_target_published_ns_ == target1.published_at_ns,
               "drop consumed target sequence or renewed the hold");
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
            admit(robot, kStart + 50*kMs, kStart + 75*kMs), 0.3F), kStart + 75*kMs);
        expect(target2.policy_seq == 3 && target2.target_seq == 2 &&
                   target2.inference_record.policy_seq == 3 && target2.inference_record.target_seq == 2,
               "recovery did not preserve separate sequence spaces");
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
        robot.completed_policy_record_channel_.publish(target2.inference_record);
        robot.initialize_policy_runtime_state();
        Target ignored;
        InferenceRecord ignored_record;
        expect(robot.next_policy_seq_ == 1 && robot.next_target_seq_ == 1 &&
                   robot.stale_policy_drop_count_ == 0 && robot.last_policy_target_published_ns_ == 0 &&
                   robot.first_policy_inference_started_ns_.load() == 0 &&
                   robot.observation_builder_->frame_index() == 0 &&
                   !robot.policy_target_channel_.try_consume_latest(ignored) &&
                   !robot.completed_policy_record_channel_.try_consume_latest(ignored_record),
               "reinitialization did not clear runtime state");
    }

    static void test_admission_and_logging_delay()
    {
        RobotInterface robot(make_config());
        prepare(robot);
        const auto seq = begin(robot, kStart);
        const auto at_boundary = admit(robot, kStart, kStart + 40*kMs);
        expect(!at_boundary.dropped && at_boundary.obs_to_action_age_us == 40000 &&
                   at_boundary.target_hold_age_us == 40000, "40ms must be admitted");
        expect(admit(robot, kStart, kStart + 40*kMs + 1).dropped, "admission must compare nanoseconds");
        // Synthetic 5ms spent assembling logs AFTER admission: do not re-admit.
        const auto target = publish(robot, record(robot, seq, kStart, at_boundary, 0.1F),
                                    kStart + 45*kMs);
        expect(target.published_at_ns == kStart + 45*kMs &&
                   target.valid_until_ns == kStart + 105*kMs &&
                   target.inference_record.obs_to_action_age_us == 40000 &&
                   target.inference_record.target_hold_age_us == 40000,
               "logging delay contaminated admission or publication time");
        // Same observation timestamp may yield another valid target within 40ms.
        robot.initialize_policy_runtime_state();
        const auto first_seq = begin(robot, kStart);
        publish(robot, record(robot, first_seq, kStart,
            admit(robot, kStart, kStart + 20*kMs), 0.1F), kStart + 20*kMs);
        const auto second_seq = begin(robot, kStart + 20*kMs);
        const auto same_obs = publish(robot, record(robot, second_seq, kStart,
            admit(robot, kStart, kStart + 39*kMs), 0.2F), kStart + 39*kMs);
        expect(same_obs.policy_seq == 2 && same_obs.target_seq == 2, "duplicate observation was rejected");
    }

    static void test_sensors_and_startup()
    {
        RobotInterface robot(make_config());
        prepare(robot);
        std::string error;
        expect(!robot.validate_policy_sensor_timing(0, kStart, kStart, error), "missing motor timestamp accepted");
        expect(!robot.validate_policy_sensor_timing(kStart, 0, kStart, error), "missing IMU timestamp accepted");
        expect(!robot.validate_policy_sensor_timing(kStart + 1, kStart, kStart, error), "future motor accepted");
        expect(!robot.validate_policy_sensor_timing(kStart, kStart + 1, kStart, error), "future IMU accepted");
        expect(robot.validate_policy_sensor_timing(kStart, kStart, kStart + 50*kMs, error), "50ms boundary rejected");
        expect(!robot.validate_policy_sensor_timing(kStart, kStart, kStart + 50*kMs + 1, error), "sensor guard relaxed");
        expect(robot.validate_policy_sensor_timing(kStart + 30*kMs, kStart, kStart + 30*kMs, error), "30ms skew rejected");
        expect(!robot.validate_policy_sensor_timing(kStart + 30*kMs + 1, kStart, kStart + 30*kMs + 1, error), "skew guard relaxed");
        expect(!robot.startup_policy_target_expired(kStart + 1000*kMs), "startup timer armed before inference");
        begin(robot, kStart);
        begin(robot, kStart + 25*kMs);
        expect(robot.startup_policy_deadline_ns() == kStart + 60*kMs &&
                   !robot.startup_policy_target_expired(kStart + 60*kMs - 1) &&
                   robot.startup_policy_target_expired(kStart + 60*kMs),
               "startup timer renewed or missed equality");
        const auto normal = robot.motor_session_.target_command_timing(kStart, 0);
        const auto near_deadline = robot.motor_session_.target_command_timing(
            kStart + 59*kMs, robot.startup_policy_deadline_ns());
        expect(normal.source_policy_seq == 0 && normal.valid_until_ns == kStart + 10*kMs &&
                   near_deadline.valid_until_ns == kStart + 60*kMs, "startup command not capped");
        run_expired_worker(robot);  // First inference blocked beyond 60ms, no hardware needed.
    }

    static void test_replay_and_continuous_drops()
    {
        // Recorded timestamps; publication itself was not logged. Exercise both
        // bounds: inference end and first command submission. Recovery is synthetic.
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
                admit(robot, obs2694, end2694), 0.1F), published_at);
            seq = begin(robot, start2695);
            const auto stale = record(robot, seq, obs2695, admit(robot, obs2695, end2695), 0.9F);
            drop(robot, stale);
            expect(stale.policy_seq == 2695 && stale.obs_to_action_age_us == 43925 &&
                       !expired(held, original_fault), "2695 replay failed to hold previous target");
            seq = begin(robot, original_fault);
            const auto recovered = publish(robot, record(robot, seq, original_fault,
                admit(robot, original_fault, original_fault + 5*kMs), 0.2F), original_fault + 5*kMs);
            expect(recovered.policy_seq == 2696 && recovered.target_seq == 2, "synthetic recovery failed");
        }
        RobotInterface robot(make_config());
        prepare(robot);
        auto seq = begin(robot, kStart);
        const auto held = publish(robot, record(robot, seq, kStart,
            admit(robot, kStart, kStart + 25*kMs), 0.1F), kStart + 25*kMs);
        for (int i = 0; i < 3; ++i) {
            const auto now = kStart + (50 + i*25)*kMs;
            seq = begin(robot, now - 20*kMs);
            drop(robot, record(robot, seq, now - 45*kMs, admit(robot, now - 45*kMs, now), 0.9F));
        }
        expect(robot.stale_policy_drop_count_ == 3 && robot.last_policy_target_published_ns_ == held.published_at_ns,
               "continuous drops renewed the watchdog");
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
                   values[column(header, "target_seq")] == "0" &&
                   values[column(header, "policy_result_dropped")] == "1" &&
                   values[column(header, "command_applied")] == "0" &&
                   values[column(header, "command_produced_at_ns")] == "0" &&
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
        Access::test_replay_and_continuous_drops();
        Access::test_drop_recorder_path();
        std::cout << "robot_policy_target_timing_test passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
