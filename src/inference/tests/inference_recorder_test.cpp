#include "recorder/inference_recorder.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr std::int64_t kFrameStepNs = 1'000'000;

void expect(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

std::filesystem::path test_root()
{
    return std::filesystem::temp_directory_path() / "inference_recorder_test";
}

std::filesystem::path latest_csv(const std::filesystem::path& dir)
{
    std::vector<std::filesystem::path> paths;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (entry.path().extension() == ".csv") {
            paths.push_back(entry.path());
        }
    }
    expect(!paths.empty(), "expected at least one csv under " + dir.string());
    std::sort(paths.begin(), paths.end());
    return paths.back();
}

std::vector<std::string> read_lines(const std::filesystem::path& path)
{
    std::ifstream file(path);
    expect(static_cast<bool>(file), "failed to open " + path.string());

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(file, line)) {
        lines.push_back(line);
    }
    return lines;
}

std::vector<std::string> split_csv_line(const std::string& line)
{
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

inference::InferenceRecorderConfig config_for(const std::filesystem::path& dir)
{
    inference::InferenceRecorderConfig config;
    config.enabled = true;
    config.directory = dir;
    config.flush_interval = std::chrono::milliseconds(1000);
    config.max_queue_depth = inference::kDefaultInferenceRecorderQueueDepth;
    return config;
}

inference::InferenceRecord make_record(std::uint64_t frame_index,
                                       std::int64_t base_timestamp_ns)
{
    inference::InferenceRecord record;
    record.frame_index = frame_index;
    record.state_timestamp_ns = base_timestamp_ns - 100'000;
    record.inference_start_ns =
        base_timestamp_ns + static_cast<std::int64_t>(frame_index) * kFrameStepNs;
    record.inference_end_ns = record.inference_start_ns + 50'000;
    record.command_timestamp_ns = record.inference_end_ns + 10'000;
    record.command_applied = true;

    for (std::size_t i = 0; i < inference::kInferenceMotorCount; ++i) {
        record.raw_action[i] = static_cast<float>(frame_index) + static_cast<float>(i) * 0.01F;
        record.target_q_model_rad[i] = static_cast<double>(frame_index) + static_cast<double>(i) * 0.1;
        record.target_pos_rad[i] = static_cast<double>(i) * 0.2;
        record.rx_pos_rad[i] = static_cast<double>(i) * 0.1;
        record.rx_vel_rad_s[i] = static_cast<double>(i) * 0.01;
        record.torque_percent[i] = static_cast<double>(i);
        record.comm_ok[i] = 1;
        record.enabled[i] = static_cast<std::uint8_t>(i % 2);
        record.faulted[i] = 0;
    }

    return record;
}

void test_disabled()
{
    const auto dir = test_root() / "disabled";
    std::filesystem::remove_all(dir);

    inference::InferenceRecorderConfig config;
    config.enabled = false;
    config.directory = dir;

    inference::InferenceRecorder recorder;
    expect(recorder.start(config), "disabled start should succeed");
    expect(!recorder.running(), "disabled recorder should not run");
    expect(!std::filesystem::exists(dir), "disabled recorder should not create a directory");
    expect(!recorder.try_record(make_record(0, inference::steady_now_ns())),
           "disabled recorder should reject records");
    recorder.stop();
}

void test_start_header()
{
    const auto dir = test_root() / "header";
    std::filesystem::remove_all(dir);

    inference::InferenceRecorder recorder;
    expect(recorder.start(config_for(dir)), "start should succeed");
    expect(recorder.running(), "recorder should run after enabled start");
    recorder.stop();

    const auto lines = read_lines(latest_csv(dir));
    expect(std::regex_match(latest_csv(dir).filename().string(),
                            std::regex("[0-9]{4}_[0-9]{4}\\.csv")),
           "default log filename should be MMDD_HHMM.csv");
    expect(lines.size() == 1, "header-only log should have one line");
    expect(lines[0].find("frame_index,elapsed_us,state_timestamp_ns") == 0,
           "header should start with frame and timing columns");
    expect(lines[0].find("target_pos_rad_M0") != std::string::npos,
           "header should include motor target columns");
    expect(lines[0].find("torque_pct_M11") != std::string::npos,
           "header should include all torque columns");
}

void test_record_stop_and_elapsed()
{
    const auto dir = test_root() / "record";
    std::filesystem::remove_all(dir);

    inference::InferenceRecorder recorder;
    auto config = config_for(dir);
    expect(recorder.start(config), "record test start should succeed");

    const std::int64_t base = inference::steady_now_ns();
    for (std::uint64_t frame = 0; frame < 3; ++frame) {
        expect(recorder.try_record(make_record(frame, base)),
               "try_record should accept frame " + std::to_string(frame));
    }
    recorder.stop();

    const auto csv = latest_csv(dir);
    const auto lines = read_lines(csv);
    expect(lines.size() == 4, "three records plus header should be written");
    expect(lines[1].find("0,") == 0, "first data row should be frame 0");
    expect(lines[2].find("1,") == 0, "second data row should be frame 1");
    expect(lines[3].find("2,") == 0, "third data row should be frame 2");

    const auto row0 = split_csv_line(lines[1]);
    const auto row1 = split_csv_line(lines[2]);
    expect(row0.size() == row1.size(), "CSV rows should have stable width");
    const auto elapsed0 = std::stoll(row0[1]);
    const auto elapsed1 = std::stoll(row1[1]);
    expect(elapsed1 - elapsed0 == kFrameStepNs / 1000,
           "elapsed_us should share one session start timestamp");
    expect(std::stoll(row0[5]) == 50, "inference_duration_us should be derived");
}

void test_stop_drains_queue()
{
    const auto dir = test_root() / "drain";
    std::filesystem::remove_all(dir);

    inference::InferenceRecorder recorder;
    expect(recorder.start(config_for(dir)), "drain test start should succeed");

    const std::int64_t base = inference::steady_now_ns();
    for (std::uint64_t frame = 0; frame < 100; ++frame) {
        expect(recorder.try_record(make_record(frame, base)),
               "drain try_record should accept records");
    }
    recorder.stop();

    const auto lines = read_lines(latest_csv(dir));
    expect(lines.size() == 101, "stop should drain all queued records");
}

void test_overflow_drops_oldest()
{
    const auto dir = test_root() / "overflow";
    std::filesystem::remove_all(dir);

    auto config = config_for(dir);
    config.max_queue_depth = 2;

    inference::InferenceRecorder recorder;
    expect(recorder.start(config), "overflow test start should succeed");

    const std::int64_t base = inference::steady_now_ns();
    for (std::uint64_t frame = 0; frame < 20000; ++frame) {
        expect(recorder.try_record(make_record(frame, base)),
               "overflow try_record should remain non-blocking and return true");
    }
    recorder.stop();

    expect(recorder.dropped_records() > 0,
           "bounded queue should drop records when producer outruns worker");
}

void test_zero_queue_depth_uses_default_capacity()
{
    const auto dir = test_root() / "zero_depth";
    std::filesystem::remove_all(dir);

    auto config = config_for(dir);
    config.max_queue_depth = 0;

    inference::InferenceRecorder recorder;
    expect(recorder.start(config), "zero-depth start should use default capacity");
    expect(recorder.max_queue_depth() == inference::kDefaultInferenceRecorderQueueDepth,
           "zero max_queue_depth should be normalized to bounded default capacity");

    const std::int64_t base = inference::steady_now_ns();
    for (std::uint64_t frame = 0; frame < 3; ++frame) {
        expect(recorder.try_record(make_record(frame, base)),
               "zero-depth recorder should accept records with bounded default capacity");
    }
    recorder.stop();
}

}  // namespace

int main()
{
    std::filesystem::remove_all(test_root());
    std::filesystem::create_directories(test_root());

    test_disabled();
    test_start_header();
    test_record_stop_and_elapsed();
    test_stop_drains_queue();
    test_overflow_drops_oldest();
    test_zero_queue_depth_uses_default_capacity();

    std::cout << "inference_recorder_test passed: " << test_root() << "\n";
    return 0;
}
