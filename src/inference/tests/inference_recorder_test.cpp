#include "recorder/inference_recorder.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

namespace {
void check(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}
}

int main()
{
    const auto directory = std::filesystem::temp_directory_path() /
        ("inference_recorder_test_" + std::to_string(getpid()));
    inference::InferenceRecorderConfig config;
    config.directory = directory;
    inference::InferenceRecorder recorder;

    config.enabled = false;
    check(recorder.start(config, 1) && !recorder.running(), "disabled recorder behavior changed");
    config.enabled = true;
    check(!recorder.start(config, 1) && !recorder.running(), "invalid observation size accepted");
    check(!recorder.last_error().empty(), "missing error for invalid size");

    for (const std::size_t size : {675U, 705U}) {
        config.file_prefix = std::to_string(size);
        check(recorder.start(config, size), "recorder failed to start");
        inference::InferenceRecord record;
        record.policy_observation[size - 1] = 123.0F;
        check(recorder.try_record(record), "record rejected");
        const auto path = recorder.log_path();
        recorder.stop();
        std::ifstream file(path);
        std::string header;
        std::string row;
        check(static_cast<bool>(std::getline(file, header)) && static_cast<bool>(std::getline(file, row)),
              "missing log header or record");
        check(header.find(",policy_obs_" + std::to_string(size - 1) + ",raw_action_0") != std::string::npos,
              "incorrect observation columns or order");
        check(header.find(",policy_obs_" + std::to_string(size) + ",") == std::string::npos,
              "extra observation column");
        check(std::count(header.begin(), header.end(), ',') == std::count(row.begin(), row.end(), ','),
              "record and header have different column counts");
        check(row.find(",123,") != std::string::npos, "last observation value missing");
    }
    std::filesystem::remove_all(directory);
}
