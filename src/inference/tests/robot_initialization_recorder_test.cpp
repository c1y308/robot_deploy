#include "config/deploy_config.hpp"
#include "robot/robot_interface.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

int ethercat_request_calls = 0;

void expect(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void expect_failure_before_hardware(const inference::RobotInterfaceConfig& config)
{
    inference::RobotInterface robot(config);
    expect(!robot.initialize(), "recorder startup failure did not fail initialization");
    expect(ethercat_request_calls == 0,
           "EtherCAT was initialized after recorder startup failed");
    expect(!robot.is_initialized(), "failed robot was marked initialized");
    expect(robot.shutdown() == inference::ShutdownResult::Confirmed,
           "failed initialization did not shut down cleanly");
}

}  // namespace

// Never acquire real hardware even if initialization continues past a failure.
struct ec_master;
extern "C" ec_master* __wrap_ecrt_request_master(unsigned int)
{
    ++ethercat_request_calls;
    return nullptr;
}

int main()
{
    inference::RobotInterfaceConfig config;
    std::string error;
    expect(inference::load_deploy_config(ROBOT_DEPLOY_CONFIG_PATH, config, error),
           error);

    char directory_template[] = "/tmp/robot-recorder-test-XXXXXX";
    const char* directory = ::mkdtemp(directory_template);
    expect(directory != nullptr, "mkdtemp failed");
    const std::filesystem::path root(directory);
    const auto blocked_directory = root / "regular-file";
    {
        std::ofstream file(blocked_directory);
        file << "not a directory\n";
        expect(file.good(), "failed to write recorder fixture");
    }

    config.recorder.enabled = true;
    config.recorder.directory = blocked_directory;
    expect_failure_before_hardware(config);

    config.recorder.directory = root / "records";
    config.runtime.background.cpu_ids = {-1};
    expect_failure_before_hardware(config);
    expect(std::filesystem::exists(config.recorder.directory),
           "initialization did not reach recorder thread startup");

    config.recorder.enabled = false;
    config.recorder.directory = blocked_directory;
    {
        inference::RobotInterface robot(config);
        expect(!robot.initialize(), "mock EtherCAT connection unexpectedly succeeded");
        expect(ethercat_request_calls == 1,
               "disabled recorder prevented hardware initialization");
        expect(robot.shutdown() == inference::ShutdownResult::Confirmed,
               "mock hardware failure did not shut down cleanly");
    }

    std::filesystem::remove_all(root);
    std::cout << "robot_initialization_recorder_test passed\n";
    return 0;
}
