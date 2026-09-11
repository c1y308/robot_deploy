#include "policy/policy_runtime.hpp"
#include "robot/robot_config.hpp"
#include "tool/thread_runtime.hpp"

#include <ATen/Parallel.h>
#include <dlfcn.h>
#include <sched.h>

#include <iostream>
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

}  // namespace

int main()
{
    const std::vector<int> cpus = first_allowed_cpus(2);
    robot_base::ThreadRuntimeOptions main_options;
    main_options.cpu_ids = cpus;
    main_options.scheduling_policy = robot_base::ThreadSchedulingPolicy::OTHER;
    const auto setup = robot_base::configure_current_thread("torch_rt_test",
                                                            main_options);
    expect(setup.success, setup.error);

    inference::PolicyRuntimeConfig config;
    config.model_path = ROBOT_TEST_POLICY_PATH;
    inference::PolicyRuntime runtime;
    expect(runtime.load(config, 2, 1, 1, cpus), runtime.last_error());
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
    std::cout << "policy_runtime_threading_test passed\n";
    return 0;
}
