#include "recorder/inference_recorder.hpp"

#include <chrono>
#include <cstddef>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <system_error>
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

void test_policy_observation_csv_columns()
{
    static_assert(inference::policy_observation::kObservationSize == 705,
                  "test expects the P1 705-dim observation layout");

    const std::filesystem::path dir = unique_test_dir();

    inference::InferenceRecorderConfig config;
    config.enabled = true;
    config.directory = dir;
    config.file_prefix = "policy_obs";
    config.flush_interval = std::chrono::milliseconds(1);
    config.max_queue_depth = 8;

    inference::InferenceRecorder recorder;
    expect(recorder.start(config),
           "failed to start recorder: " + recorder.last_error());

    inference::InferenceRecord record;
    record.frame_index = 42;
    record.inference_start_ns = inference::steady_now_ns();
    record.inference_end_ns = record.inference_start_ns + 123000;
    record.motor_sample_timestamp_ns = record.inference_start_ns - 10000;
    record.command_timestamp_ns = record.inference_end_ns + 1000;
    record.imu_receive_timestamp_ns = record.inference_start_ns - 8000;
    record.imu_sample_timestamp_ns = 123456789000ULL;
    record.command_applied = true;

    for (std::size_t i = 0; i < record.policy_observation.size(); ++i) {
        record.policy_observation[i] = static_cast<float>(i);
    }
    for (std::size_t i = 0; i < inference::kInferenceDof; ++i) {
        record.raw_action[i] = static_cast<float>(100 + i);
        record.target_q_model_rad[i] = 200.0 + static_cast<double>(i);
        record.rx_pos_rad[i] = 300.0 + static_cast<double>(i);
        record.rx_vel_rad_s[i] = 400.0 + static_cast<double>(i);
    }

    expect(recorder.try_record(record),
           "failed to queue record: " + recorder.last_error());

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

    const std::size_t policy_obs_0 = require_column(columns, "policy_obs_0");
    expect(policy_obs_0 + inference::policy_observation::kObservationSize <= columns.size(),
           "CSV does not have enough policy observation columns");
    for (std::size_t i = 0; i < inference::policy_observation::kObservationSize; ++i) {
        const std::string expected_name = "policy_obs_" + std::to_string(i);
        expect(columns[policy_obs_0 + i] == expected_name,
               "policy observation column is not contiguous at index " +
               std::to_string(i));
        expect(near(std::stod(values[policy_obs_0 + i]), static_cast<double>(i)),
               "policy observation value mismatch at index " + std::to_string(i));
    }

    std::error_code ignored;
    std::filesystem::remove_all(dir, ignored);
}

}  // namespace

int main()
{
    test_policy_observation_csv_columns();
    std::cout << "inference_recorder_csv_test passed\n";
    return 0;
}
