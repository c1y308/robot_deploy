#include "recorder/inference_recorder.hpp"

#include <chrono>
#include <cstddef>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

namespace {

void expect(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

std::vector<std::string> split_csv_line(const std::string& line)
{
    std::vector<std::string> cells;
    std::stringstream stream(line);
    std::string cell;
    while (std::getline(stream, cell, ',')) {
        cells.push_back(cell);
    }
    return cells;
}

std::size_t require_column(const std::vector<std::string>& columns,
                           const std::string& name)
{
    for (std::size_t i = 0; i < columns.size(); ++i) {
        if (columns[i] == name) {
            return i;
        }
    }
    expect(false, "missing CSV column: " + name);
    return 0;
}

bool has_column(const std::vector<std::string>& columns,
                const std::string& name)
{
    for (const auto& column : columns) {
        if (column == name) {
            return true;
        }
    }
    return false;
}

std::filesystem::path unique_test_dir()
{
    const auto now = std::chrono::steady_clock::now()
                         .time_since_epoch()
                         .count();
    return std::filesystem::temp_directory_path() /
           ("inference_recorder_csv_test_" + std::to_string(now));
}

bool near(double actual, double expected)
{
    return std::abs(actual - expected) < 1e-6;
}

void test_policy_observation_csv_columns(std::size_t observation_size)
{
    const std::filesystem::path dir = unique_test_dir();

    inference::InferenceRecorderConfig config;
    config.enabled = true;
    config.directory = dir;
    config.file_prefix = "policy_obs";
    config.flush_interval = std::chrono::milliseconds(1);
    config.max_queue_depth = 8;
    config.policy_observation_size = observation_size;

    inference::InferenceRecorder recorder;
    expect(recorder.start(config),
           "failed to start recorder: " + recorder.last_error());

    inference::InferenceRecord record;
    record.frame_index = 42;
    record.inference_start_ns = inference::steady_now_ns();
    record.inference_end_ns = record.inference_start_ns + 123000;
    record.motor_sample_timestamp_ns = record.inference_start_ns - 10000;
    record.policy_seq = 42;
    record.policy_observation_time_ns = record.inference_start_ns - 20000;
    record.policy_valid_until_ns = record.inference_end_ns + 60000000;
    record.command_timestamp_ns = record.inference_end_ns + 1000;
    record.command_valid_until_ns = record.command_timestamp_ns + 10000000;
    record.imu_receive_timestamp_ns = record.inference_start_ns - 8000;
    record.imu_sample_timestamp_ns = 123456789000ULL;
    record.command_applied = true;
    record.target_seq = 30;
    record.obs_to_action_age_us = 143;
    record.target_hold_age_us = 25000;

    for (std::size_t i = 0; i < record.policy_observation.size(); ++i) {
        record.policy_observation[i] = static_cast<float>(i);
    }
    for (std::size_t i = 0; i < inference::kInferenceDof; ++i) {
        record.raw_action[i] = static_cast<float>(100 + i);
        record.target_q_model_rad[i] = 200.0 + static_cast<double>(i);
        record.target_pos_rad[i] = 300.0 + static_cast<double>(i);
        record.target_effort_permille[i] = 400.0 + static_cast<double>(i);
        record.rx_pos_rad[i] = 300.0 + static_cast<double>(i);
        record.rx_vel_rad_s[i] = 400.0 + static_cast<double>(i);
    }

    expect(recorder.try_record(record),
           "failed to queue record: " + recorder.last_error());

    inference::InferenceRecord drop = record;
    drop.frame_index = 43;
    drop.policy_seq = 43;
    drop.target_seq = 0;
    drop.command_applied = false;
    drop.policy_result_dropped = true;
    drop.obs_to_action_age_us = 44000;
    drop.target_hold_age_us = 26000;
    drop.command_timestamp_ns = 0;
    drop.command_valid_until_ns = 0;
    drop.policy_valid_until_ns = 0;
    expect(recorder.try_record(drop), "failed to queue drop record");

    const std::filesystem::path log_path = recorder.log_path();
    recorder.stop();

    std::ifstream file(log_path);
    expect(file.good(), "failed to open recorder output: " + log_path.string());

    std::string header_line;
    std::string data_line;
    expect(static_cast<bool>(std::getline(file, header_line)), "missing CSV header");
    expect(static_cast<bool>(std::getline(file, data_line)), "missing CSV data row");

    const std::vector<std::string> columns = split_csv_line(header_line);
    const std::vector<std::string> values = split_csv_line(data_line);
    expect(columns.size() == values.size(), "CSV header/data column count mismatch");

    require_column(columns, "elapsed_us");
    require_column(columns, "raw_action_0");
    require_column(columns, "target_q_model_rad_0");
    require_column(columns, "rx_pos_rad_M0");
    require_column(columns, "rx_vel_rad_s_M0");
    const std::size_t command_timestamp_index =
        require_column(columns, "command_timestamp_ns");
    const std::size_t command_applied_index =
        require_column(columns, "command_applied");
    const std::size_t policy_seq_index = require_column(columns, "policy_seq");
    const std::size_t policy_observation_time_index =
        require_column(columns, "policy_observation_time_ns");
    const std::size_t policy_valid_until_index =
        require_column(columns, "policy_valid_until_ns");
    expect(!has_column(columns, "command_produced_at_ns"),
           "duplicate command production timestamp column must be removed");
    const std::size_t command_valid_until_index =
        require_column(columns, "command_valid_until_ns");
    const std::size_t target_pos_index = require_column(columns, "target_pos_rad_M0");
    const std::size_t target_effort_index =
        require_column(columns, "target_effort_permille_M0");
    const std::size_t imu_receive_index =
        require_column(columns, "imu_receive_timestamp_ns");
    const std::size_t imu_sample_index =
        require_column(columns, "imu_sample_timestamp_ns");
    expect(!has_column(columns, "imu_rx_timestamp_ns"),
           "old IMU rx timestamp column must be removed");
    expect(!has_column(columns, "imu_publish_timestamp_ns"),
           "old IMU publish timestamp column must be removed");
    expect(!has_column(columns, "imu_device_timestamp_us"),
           "old IMU device timestamp column must be removed");
    expect(!has_column(columns, "imu_device_timestamp_valid"),
           "old IMU timestamp validity column must be removed");
    expect(!has_column(columns, "imu_rx_to_publish_us"),
           "old IMU rx-to-publish column must be removed");
    expect(!has_column(columns, "imu_motor_skew_us"),
           "old IMU motor skew column must be removed");
    expect(!has_column(columns, "imu_age_us"),
           "old IMU age column must be removed");
    expect(std::stoll(values[imu_receive_index]) ==
               record.imu_receive_timestamp_ns,
           "IMU receive timestamp value mismatch");
    expect(std::stoull(values[imu_sample_index]) ==
               record.imu_sample_timestamp_ns,
           "IMU sample timestamp value mismatch");
    expect(std::stoull(values[policy_seq_index]) == record.policy_seq,
           "policy sequence value mismatch");
    expect(std::stoll(values[command_timestamp_index]) ==
               record.command_timestamp_ns,
           "command timestamp value mismatch");
    expect(std::stoi(values[command_applied_index]) == 1,
           "command applied value mismatch");
    const std::size_t target_seq_index = require_column(columns, "target_seq");
    const std::size_t obs_age_index = require_column(columns, "obs_to_action_age_us");
    const std::size_t hold_age_index = require_column(columns, "target_hold_age_us");
    const std::size_t dropped_index = require_column(columns, "policy_result_dropped");
    expect(target_seq_index == columns.size() - 4 &&
               obs_age_index == target_seq_index + 1 &&
               hold_age_index == target_seq_index + 2 &&
               dropped_index == target_seq_index + 3,
           "B1 columns must be appended without moving existing columns");
    expect(std::stoull(values[target_seq_index]) == 30 &&
               std::stoll(values[obs_age_index]) == 143 &&
               std::stoll(values[hold_age_index]) == 25000 &&
               std::stoi(values[dropped_index]) == 0,
           "normal B1 record values mismatch");
    expect(std::stoll(values[policy_observation_time_index]) ==
               record.policy_observation_time_ns,
           "policy observation timestamp value mismatch");
    expect(std::stoll(values[policy_valid_until_index]) ==
               record.policy_valid_until_ns,
           "policy deadline value mismatch");
    expect(std::stoll(values[command_valid_until_index]) ==
               record.command_valid_until_ns,
           "command deadline value mismatch");
    expect(near(std::stod(values[target_pos_index]), record.target_pos_rad[0]),
           "command target position value mismatch");
    expect(near(std::stod(values[target_effort_index]),
                record.target_effort_permille[0]),
           "command target effort value mismatch");

    const std::size_t policy_obs_0 = require_column(columns, "policy_obs_0");
    expect(policy_obs_0 + observation_size <= columns.size(),
           "CSV does not have enough policy observation columns");
    for (std::size_t i = 0; i < observation_size; ++i) {
        const std::string expected_name = "policy_obs_" + std::to_string(i);
        expect(columns[policy_obs_0 + i] == expected_name,
               "policy observation column is not contiguous at index " +
               std::to_string(i));
        expect(near(std::stod(values[policy_obs_0 + i]), static_cast<double>(i)),
               "policy observation value mismatch at index " + std::to_string(i));
    }
    expect(!has_column(columns, "policy_obs_" + std::to_string(observation_size)),
           "CSV contains a policy observation column past the configured size");

    expect(static_cast<bool>(std::getline(file, data_line)), "missing drop CSV row");
    const auto drop_values = split_csv_line(data_line);
    expect(drop_values.size() == columns.size(), "drop CSV column count mismatch");
    expect(std::stoull(drop_values[policy_seq_index]) == 43 &&
               std::stoull(drop_values[target_seq_index]) == 0 &&
               std::stoi(drop_values[command_applied_index]) == 0 &&
               std::stoi(drop_values[dropped_index]) == 1 &&
               std::stoll(drop_values[obs_age_index]) == 44000 &&
               std::stoll(drop_values[hold_age_index]) == 26000,
           "drop row must preserve policy_seq and admission ages");
    expect(std::stoll(drop_values[command_timestamp_index]) == 0 &&
               std::stoll(drop_values[command_valid_until_index]) == 0 &&
               std::stoll(drop_values[policy_valid_until_index]) == 0,
           "drop row must not claim a target or command deadline");

    std::error_code ignored;
    std::filesystem::remove_all(dir, ignored);
}

void test_idle_recorder_waits_and_resumes()
{
    const std::filesystem::path dir = unique_test_dir();
    inference::InferenceRecorderConfig config;
    config.directory = dir;
    config.flush_interval = std::chrono::milliseconds(50);

    inference::InferenceRecorder recorder;
    expect(recorder.start(config), "failed to start idle recorder");

    // The header is flushed once, then two idle flush deadlines must pass.
    std::this_thread::sleep_for(config.flush_interval * 3);
    const auto wall_start = std::chrono::steady_clock::now();
    const auto cpu_start = std::clock();
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    const double cpu_ms = 1000.0 * (std::clock() - cpu_start) / CLOCKS_PER_SEC;
    const double wall_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - wall_start).count();
    expect(cpu_ms < wall_ms * 0.25,
           "idle recorder is spinning: CPU ms=" + std::to_string(cpu_ms) +
           ", wall ms=" + std::to_string(wall_ms));

    inference::InferenceRecord record;
    record.frame_index = 42;
    expect(recorder.try_record(record), "idle recorder did not resume recording");
    const auto log_path = recorder.log_path();
    recorder.stop();

    std::ifstream file(log_path);
    std::string header;
    std::string data;
    expect(static_cast<bool>(std::getline(file, header)) &&
               static_cast<bool>(std::getline(file, data)),
           "record queued after idle was lost on shutdown");
    const auto columns = split_csv_line(header);
    const auto values = split_csv_line(data);
    expect(values.size() == columns.size() &&
               values[require_column(columns, "frame_index")] == "42",
           "record queued after idle was corrupted");
    file.close();
    std::filesystem::remove_all(dir);
}

void test_invalid_policy_observation_size_is_rejected()
{
    inference::InferenceRecorderConfig config;
    config.directory = unique_test_dir();
    config.policy_observation_size =
        inference::policy_observation::kMaxObservationSize + 1;

    inference::InferenceRecorder recorder;
    expect(!recorder.start(config),
           "recorder accepted an unsupported policy observation size");
    expect(recorder.last_error() ==
               "policy observation size must be 675 or 705",
           "unexpected invalid observation size error: " +
               recorder.last_error());
}

}  // namespace

int main()
{
    test_policy_observation_csv_columns(
        inference::policy_observation::kObservationSizeWithoutGaitPhase);
    test_policy_observation_csv_columns(
        inference::policy_observation::kObservationSizeWithGaitPhase);
    test_invalid_policy_observation_size_is_rejected();
    test_idle_recorder_waits_and_resumes();
    std::cout << "inference_recorder_csv_test passed\n";
    return 0;
}
