#include "policy/policy_runtime.hpp"
#include "robot/robot_config.hpp"
#include "tool/thread_runtime.hpp"

#include <ATen/Parallel.h>
#include <dlfcn.h>
#include <sched.h>

#include <algorithm>
#include <cerrno>
#include <filesystem>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef ROBOT_TEST_POLICY_PATH
#define ROBOT_TEST_POLICY_PATH ""
#endif

namespace {

void expect(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::vector<int> first_allowed_cpus(std::size_t limit)
{
    cpu_set_t allowed;
    CPU_ZERO(&allowed);
    expect(::sched_getaffinity(0, sizeof(allowed), &allowed) == 0,
           "sched_getaffinity failed");
    std::vector<int> result;
    for (int cpu = 0; cpu < CPU_SETSIZE && result.size() < limit; ++cpu) {
        if (CPU_ISSET(cpu, &allowed)) {
            result.push_back(cpu);
        }
    }
    expect(!result.empty(), "test process has no allowed CPU");
    return result;
}

std::set<pid_t> process_task_ids()
{
    std::set<pid_t> result;
    for (const auto& entry :
         std::filesystem::directory_iterator("/proc/self/task")) {
        const std::string name = entry.path().filename().string();
        std::size_t parsed = 0;
        const long task_id = std::stol(name, &parsed);
        if (parsed == name.size() && task_id > 0) {
            result.insert(static_cast<pid_t>(task_id));
        }
    }
    return result;
}

void expect_worker_configuration(pid_t task_id, const std::vector<int>& cpus)
{
    cpu_set_t affinity;
    CPU_ZERO(&affinity);
    if (::sched_getaffinity(task_id, sizeof(affinity), &affinity) != 0) {
        expect(errno == ESRCH, "failed to read new worker affinity");
        return;
    }
    for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
        if (!CPU_ISSET(cpu, &affinity)) {
            continue;
        }
        expect(std::find(cpus.begin(), cpus.end(), cpu) != cpus.end(),
               "new Torch/OpenMP worker can run outside policy CPU set");
    }

    const int policy = ::sched_getscheduler(task_id);
    expect(policy == SCHED_OTHER, "new Torch/OpenMP worker is not SCHED_OTHER");
    sched_param parameters{};
    expect(::sched_getparam(task_id, &parameters) == 0,
           "failed to read new worker priority");
    expect(parameters.sched_priority == 0,
           "new Torch/OpenMP worker priority is not zero");
}

}  // namespace

int main()
{
    const std::vector<int> cpus = first_allowed_cpus(2);

    inference::PolicyRuntimeConfig config;
    config.model_path = ROBOT_TEST_POLICY_PATH;
    config.gait.enabled = true;
    inference::PolicyRuntime runtime;

    const std::vector<int> wrong_cpus =
        cpus.size() > 1U
            ? std::vector<int>{cpus.front()}
            : std::vector<int>{(cpus.front() + 1) % CPU_SETSIZE};
    expect(!runtime.load(config, 2, 1, 1, wrong_cpus),
           "PolicyRuntime accepted incorrect creator affinity");
    expect(runtime.last_error().find("affinity") != std::string::npos,
           "incorrect creator affinity failure was not identified");

    robot_base::ThreadRuntimeOptions main_options;
    main_options.cpu_ids = cpus;
    main_options.scheduling_policy = robot_base::ThreadSchedulingPolicy::OTHER;
    const auto setup = robot_base::configure_current_thread("torch_rt_test",
                                                            main_options);
    expect(setup.success, setup.error);

    const std::set<pid_t> tasks_before = process_task_ids();
    expect(runtime.load(config, 2, 1, 1, cpus), runtime.last_error());
    const std::set<pid_t> tasks_after = process_task_ids();
    std::size_t new_workers = 0U;
    for (const pid_t task_id : tasks_after) {
        if (tasks_before.count(task_id) == 0U) {
            expect_worker_configuration(task_id, cpus);
            ++new_workers;
        }
    }
    expect(new_workers > 0U,
           "dry-run did not create any workers for independent TID verification");
    expect(at::get_num_threads() == 2, "ATen intra-op readback mismatch");
    expect(at::get_num_interop_threads() == 1,
           "ATen inter-op readback mismatch");

    using GetInt = int (*)();
    const auto openblas_get = reinterpret_cast<GetInt>(
        ::dlsym(RTLD_DEFAULT, "openblas_get_num_threads"));
    const auto omp_get_dynamic = reinterpret_cast<GetInt>(
        ::dlsym(RTLD_DEFAULT, "omp_get_dynamic"));
    expect(openblas_get != nullptr && openblas_get() == 1,
           "OpenBLAS thread readback mismatch");
    expect(omp_get_dynamic != nullptr && omp_get_dynamic() == 0,
           "OpenMP dynamic scheduling was not disabled");

    runtime.shutdown();
    std::cout << "policy_runtime_threading_test passed, verified "
              << new_workers << " new workers\n";
    return 0;
}
