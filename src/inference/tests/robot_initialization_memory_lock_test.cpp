#include "config/deploy_config.hpp"
#include "robot/robot_interface.hpp"
#include "tool/thread_runtime.hpp"

#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

int memory_lock_calls = 0;
int memory_lock_flags = 0;
int ethercat_request_calls = 0;

void expect(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

}  // namespace

extern "C" int __wrap_mlockall(int flags)
{
    ++memory_lock_calls;
    memory_lock_flags = flags;
    errno = EPERM;
    return -1;
}

// Never acquire real hardware even if an initialization-order regression occurs.
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

    cpu_set_t allowed;
    CPU_ZERO(&allowed);
    expect(::sched_getaffinity(0, sizeof(allowed), &allowed) == 0,
           "sched_getaffinity failed");
    config.runtime.enabled = true;
    for (int cpu = 0; cpu < CPU_SETSIZE &&
                      config.runtime.policy_main.cpu_ids.size() < 2U; ++cpu) {
        if (CPU_ISSET(cpu, &allowed)) {
            config.runtime.policy_main.cpu_ids.push_back(cpu);
        }
    }
    expect(!config.runtime.policy_main.cpu_ids.empty(), "no allowed CPU");
    config.runtime.policy_main.scheduling_policy =
        robot_base::ThreadSchedulingPolicy::OTHER;
    config.runtime.policy_main.stack_prefault_bytes = 128U * 1024U;
    config.runtime.torch_intra_op_threads = 2;
    config.runtime.torch_inter_op_threads = 1;
    config.runtime.openblas_threads = 1;
    config.runtime.require_process_memory_lock = true;
    const auto setup = robot_base::configure_current_thread(
        "lock_fail_test", config.runtime.policy_main);
    expect(setup.success, setup.error);

    char directory_template[] = "/tmp/robot-lock-test-XXXXXX";
    const char* directory = ::mkdtemp(directory_template);
    expect(directory != nullptr, "mkdtemp failed");
    const std::filesystem::path root(directory);
    config.recorder.enabled = true;
    config.recorder.directory = root / "records";
    {
        inference::RobotInterface robot(config);
        expect(!robot.initialize(), "memory lock failure was not fail-closed");
        expect(memory_lock_calls == 1, "initialization did not reach memory lock");
        expect(memory_lock_flags == (MCL_CURRENT | MCL_FUTURE | MCL_ONFAULT),
               "process memory lock must avoid eagerly populating future thread stacks");
        expect(ethercat_request_calls == 0,
               "EtherCAT was initialized before memory lock succeeded");
        expect(!std::filesystem::exists(config.recorder.directory),
               "recorder started before memory lock succeeded");
        expect(!robot.is_initialized(), "failed robot was marked initialized");
        expect(robot.shutdown() == inference::ShutdownResult::Confirmed,
               "failed initialization did not shut down cleanly");
    }
    std::filesystem::remove_all(root);
    std::cout << "robot_initialization_memory_lock_test passed\n";
    return 0;
}
