#include "robot/rk3588_runtime_profile.hpp"

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void expect(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void expect_options(const robot_base::ThreadRuntimeOptions& options,
                    const std::vector<int>& cpus,
                    robot_base::ThreadSchedulingPolicy policy,
                    int priority,
                    const std::string& name)
{
    expect(options.cpu_ids == cpus, name + " CPU set mismatch");
    expect(options.scheduling_policy == policy, name + " policy mismatch");
    expect(options.priority == priority, name + " priority mismatch");
}

}  // namespace

int main()
{
    const inference::RobotInterfaceConfig defaults;
    expect(!defaults.runtime.enabled,
           "generic RobotInterfaceConfig unexpectedly enables affinity");

    const auto profile = inference::make_rk3588_runtime_profile();
    expect(profile.enabled, "RK3588 profile is disabled");
    expect(profile.require_host_preflight, "RK3588 preflight is disabled");
    expect_options(profile.policy_main, {4, 5},
                   robot_base::ThreadSchedulingPolicy::OTHER, 0, "policy_main");
    expect_options(profile.motor_rt, {7},
                   robot_base::ThreadSchedulingPolicy::FIFO, 80, "ecat_rt");
    expect_options(profile.policy_command, {6},
                   robot_base::ThreadSchedulingPolicy::FIFO, 70, "policy_cmd");
    expect_options(profile.imu_reader, {2},
                   robot_base::ThreadSchedulingPolicy::OTHER, 0, "imu_rx");
    expect_options(profile.background, {0, 1},
                   robot_base::ThreadSchedulingPolicy::OTHER, 0, "background");
    expect(profile.torch_intra_op_threads == 2, "Torch intra-op mismatch");
    expect(profile.torch_inter_op_threads == 1, "Torch inter-op mismatch");
    expect(profile.openblas_threads == 1, "OpenBLAS mismatch");

    std::cout << "rk3588_runtime_profile_test passed\n";
    return 0;
}
