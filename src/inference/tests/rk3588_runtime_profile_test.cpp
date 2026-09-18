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
                    std::size_t stack_prefault_bytes,
                    const std::string& name)
{
    expect(options.cpu_ids == cpus, name + " CPU set mismatch");
    expect(options.scheduling_policy == policy, name + " policy mismatch");
    expect(options.priority == priority, name + " priority mismatch");
    expect(options.stack_prefault_bytes == stack_prefault_bytes,
           name + " stack prefault mismatch");
}

}  // namespace

int main()
{
    const inference::RobotInterfaceConfig defaults;
    expect(!defaults.runtime.require_host_preflight,
           "generic RobotInterfaceConfig unexpectedly requires host preflight");
    expect(!defaults.runtime.require_process_memory_lock,
           "generic RobotInterfaceConfig unexpectedly requires memory locking");
    expect_options(defaults.runtime.policy_main, {},
                   robot_base::ThreadSchedulingPolicy::INHERIT, 0, 0U, "default policy_main");
    expect_options(defaults.runtime.motor_rt, {},
                   robot_base::ThreadSchedulingPolicy::FIFO, 80, 0U, "default motor_rt");
    expect_options(defaults.runtime.policy_command, {},
                   robot_base::ThreadSchedulingPolicy::INHERIT, 0, 0U, "default policy_command");
    expect_options(defaults.runtime.imu_reader, {},
                   robot_base::ThreadSchedulingPolicy::INHERIT, 0, 0U, "default imu_reader");
    expect_options(defaults.runtime.background, {},
                   robot_base::ThreadSchedulingPolicy::INHERIT, 0, 0U, "default background");
    expect(defaults.runtime.torch_intra_op_threads == 0, "default Torch intra-op mismatch");
    expect(defaults.runtime.torch_inter_op_threads == 0, "default Torch inter-op mismatch");
    expect(defaults.runtime.openblas_threads == 0, "default OpenBLAS mismatch");

    const auto profile = inference::make_rk3588_runtime_profile();
    expect(profile.require_host_preflight, "RK3588 preflight is disabled");
    expect(profile.require_process_memory_lock,
           "RK3588 process memory lock is disabled");
    expect_options(profile.policy_main, {4, 5},
                   robot_base::ThreadSchedulingPolicy::OTHER, 0,
                   128U * 1024U, "policy_main");
    expect_options(profile.motor_rt, {7},
                   robot_base::ThreadSchedulingPolicy::FIFO, 80,
                   128U * 1024U, "ecat_rt");
    expect_options(profile.policy_command, {6},
                   robot_base::ThreadSchedulingPolicy::FIFO, 70,
                   128U * 1024U, "policy_cmd");
    expect_options(profile.imu_reader, {2},
                   robot_base::ThreadSchedulingPolicy::OTHER, 0,
                   64U * 1024U, "imu_rx");
    expect_options(profile.background, {0, 1},
                   robot_base::ThreadSchedulingPolicy::OTHER, 0,
                   0U, "background");
    expect(profile.torch_intra_op_threads == 2, "Torch intra-op mismatch");
    expect(profile.torch_inter_op_threads == 1, "Torch inter-op mismatch");
    expect(profile.openblas_threads == 1, "OpenBLAS mismatch");

    std::cout << "rk3588_runtime_profile_test passed\n";
    return 0;
}
