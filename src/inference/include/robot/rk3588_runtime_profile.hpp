#pragma once

#include "robot/robot_config.hpp"

#include <utility>

namespace inference {

inline robot_base::ThreadRuntimeOptions make_other_thread_options(
    std::vector<int> cpu_ids)
{
    robot_base::ThreadRuntimeOptions options;
    options.cpu_ids = std::move(cpu_ids);
    options.scheduling_policy = robot_base::ThreadSchedulingPolicy::OTHER;
    options.priority = 0;
    return options;
}

inline robot_base::ThreadRuntimeOptions make_fifo_thread_options(
    int cpu_id,
    int priority)
{
    robot_base::ThreadRuntimeOptions options;
    options.cpu_ids = {cpu_id};
    options.scheduling_policy = robot_base::ThreadSchedulingPolicy::FIFO;
    options.priority = priority;
    return options;
}

inline RuntimeThreadingConfig make_rk3588_runtime_profile()
{
    RuntimeThreadingConfig profile;
    profile.enabled = true;
    profile.require_host_preflight = true;
    profile.policy_main = make_other_thread_options({4, 5});
    profile.motor_rt = make_fifo_thread_options(7, 80);
    profile.policy_command = make_fifo_thread_options(6, 70);
    profile.imu_reader = make_other_thread_options({2});
    profile.background = make_other_thread_options({0, 1});
    profile.torch_intra_op_threads = 2;
    profile.torch_inter_op_threads = 1;
    profile.openblas_threads = 1;
    return profile;
}

}  // namespace inference
