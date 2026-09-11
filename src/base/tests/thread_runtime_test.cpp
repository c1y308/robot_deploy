#include "tool/thread_runtime.hpp"

#include <atomic>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

void expect(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

int first_allowed_cpu()
{
    cpu_set_t allowed;
    CPU_ZERO(&allowed);
    expect(sched_getaffinity(0, sizeof(allowed), &allowed) == 0,
           "sched_getaffinity failed");
    for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
        if (CPU_ISSET(cpu, &allowed)) {
            return cpu;
        }
    }
    throw std::runtime_error("no allowed CPU");
}

void test_setup_before_body()
{
    const int cpu = first_allowed_cpu();
    robot_base::ThreadRuntimeOptions options;
    options.cpu_ids = {cpu};
    options.scheduling_policy = robot_base::ThreadSchedulingPolicy::OTHER;

    std::thread thread;
    std::atomic<bool> body_ran{false};
    std::string error;
    expect(robot_base::start_configured_thread(
               thread, "rt_test", options,
               [&] {
                   char name[16]{};
                   expect(pthread_getname_np(pthread_self(), name, sizeof(name)) == 0,
                          "pthread_getname_np failed in body");
                   expect(std::string(name) == "rt_test", "body observed wrong name");
                   expect(sched_getcpu() == cpu, "body executed on wrong CPU");
                   body_ran.store(true);
               },
               error),
           error);
    thread.join();
    expect(body_ran.load(), "configured thread body did not run");
}

void test_invalid_cpu_blocks_body()
{
    robot_base::ThreadRuntimeOptions options;
    options.cpu_ids = {CPU_SETSIZE};

    std::thread thread;
    std::atomic<bool> body_ran{false};
    std::string error;
    expect(!robot_base::start_configured_thread(
               thread, "bad_cpu", options,
               [&] { body_ran.store(true); }, error),
           "invalid CPU unexpectedly succeeded");
    expect(!body_ran.load(), "body ran after setup failure");
    expect(!error.empty(), "setup failure did not report an error");
    expect(!thread.joinable(), "failed configured thread remained joinable");
}

void test_long_name_blocks_body()
{
    std::thread thread;
    std::atomic<bool> body_ran{false};
    std::string error;
    expect(!robot_base::start_configured_thread(
               thread, "thread_name_is_too_long", {},
               [&] { body_ran.store(true); }, error),
           "long name unexpectedly succeeded");
    expect(!body_ran.load(), "body ran after name failure");
}

void test_invalid_fifo_priority_blocks_body()
{
    robot_base::ThreadRuntimeOptions options;
    options.scheduling_policy = robot_base::ThreadSchedulingPolicy::FIFO;
    options.priority = sched_get_priority_max(SCHED_FIFO) + 1;

    std::thread thread;
    std::atomic<bool> body_ran{false};
    std::string error;
    expect(!robot_base::start_configured_thread(
               thread, "bad_fifo", options,
               [&] { body_ran.store(true); }, error),
           "invalid FIFO priority unexpectedly succeeded");
    expect(!body_ran.load(), "body ran after FIFO setup failure");
    expect(error.find("priority") != std::string::npos,
           "FIFO setup failure did not identify priority");
}

}  // namespace

int main()
{
    test_setup_before_body();
    test_invalid_cpu_blocks_body();
    test_long_name_blocks_body();
    test_invalid_fifo_priority_blocks_body();
    std::cout << "thread_runtime_test passed\n";
    return 0;
}
