#pragma once

#include <pthread.h>
#include <sched.h>

#include <cerrno>
#include <cstring>
#include <future>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace robot_base {

enum class ThreadSchedulingPolicy {
    INHERIT,
    OTHER,
    FIFO,
};

struct ThreadRuntimeOptions {
    std::vector<int> cpu_ids;
    ThreadSchedulingPolicy scheduling_policy{ThreadSchedulingPolicy::INHERIT};
    int priority{0};
};

struct ThreadSetupResult {
    bool success{false};
    int error_code{0};
    std::string error;
};

inline ThreadSetupResult thread_setup_error(const std::string& operation,
                                            int error_code)
{
    ThreadSetupResult result;
    result.error_code = error_code;
    result.error = operation + ": " +
                   (error_code != 0 ? std::strerror(error_code) : "verification failed");
    return result;
}

inline ThreadSetupResult configure_current_thread(
    const char* name,
    const ThreadRuntimeOptions& options)
{
    if (!name || name[0] == '\0') {
        return thread_setup_error("thread name is empty", EINVAL);
    }
    if (std::strlen(name) > 15U) {
        return thread_setup_error("thread name exceeds Linux 15-byte limit", ERANGE);
    }

    const pthread_t self = pthread_self();
    int result = pthread_setname_np(self, name);
    if (result != 0) {
        return thread_setup_error("pthread_setname_np", result);
    }

    char actual_name[16]{};
    result = pthread_getname_np(self, actual_name, sizeof(actual_name));
    if (result != 0) {
        return thread_setup_error("pthread_getname_np", result);
    }
    if (std::strcmp(name, actual_name) != 0) {
        return thread_setup_error("thread name verification", 0);
    }

    if (!options.cpu_ids.empty()) {
        cpu_set_t expected;
        CPU_ZERO(&expected);
        for (const int cpu : options.cpu_ids) {
            if (cpu < 0 || cpu >= CPU_SETSIZE) {
                return thread_setup_error("CPU id is outside cpu_set_t", EINVAL);
            }
            CPU_SET(cpu, &expected);
        }

        result = pthread_setaffinity_np(self, sizeof(expected), &expected);
        if (result != 0) {
            return thread_setup_error("pthread_setaffinity_np", result);
        }

        cpu_set_t actual;
        CPU_ZERO(&actual);
        result = pthread_getaffinity_np(self, sizeof(actual), &actual);
        if (result != 0) {
            return thread_setup_error("pthread_getaffinity_np", result);
        }
        if (!CPU_EQUAL(&expected, &actual)) {
            return thread_setup_error("thread affinity verification", 0);
        }
    }

    if (options.scheduling_policy != ThreadSchedulingPolicy::INHERIT) {
        const int policy = options.scheduling_policy == ThreadSchedulingPolicy::FIFO
                               ? SCHED_FIFO
                               : SCHED_OTHER;
        if (policy == SCHED_OTHER && options.priority != 0) {
            return thread_setup_error("SCHED_OTHER priority must be zero", EINVAL);
        }
        if (policy == SCHED_FIFO) {
            const int minimum = sched_get_priority_min(SCHED_FIFO);
            const int maximum = sched_get_priority_max(SCHED_FIFO);
            if (minimum < 0 || maximum < 0 || options.priority < minimum ||
                options.priority > maximum) {
                return thread_setup_error("SCHED_FIFO priority is out of range", EINVAL);
            }
        }

        sched_param requested{};
        requested.sched_priority = options.priority;
        result = pthread_setschedparam(self, policy, &requested);
        if (result != 0) {
            return thread_setup_error("pthread_setschedparam", result);
        }

        int actual_policy = 0;
        sched_param actual{};
        result = pthread_getschedparam(self, &actual_policy, &actual);
        if (result != 0) {
            return thread_setup_error("pthread_getschedparam", result);
        }
        if (actual_policy != policy || actual.sched_priority != options.priority) {
            return thread_setup_error("thread scheduling verification", 0);
        }
    }

    ThreadSetupResult success;
    success.success = true;
    return success;
}

template <typename Callable>
bool start_configured_thread(std::thread& output,
                             const char* name,
                             const ThreadRuntimeOptions& options,
                             Callable&& callable,
                             std::string& error)
{
    if (output.joinable()) {
        error = "thread is already joinable";
        return false;
    }

    std::promise<ThreadSetupResult> setup_promise;
    std::future<ThreadSetupResult> setup_future = setup_promise.get_future();

    try {
        output = std::thread(
            [name_string = std::string(name ? name : ""),
             options,
             setup_promise = std::move(setup_promise),
             function = std::forward<Callable>(callable)]() mutable {
                ThreadSetupResult setup;
                try {
                    setup = configure_current_thread(name_string.c_str(), options);
                } catch (const std::exception& exception) {
                    setup.error = std::string("thread setup exception: ") +
                                  exception.what();
                } catch (...) {
                    setup.error = "thread setup exception";
                }
                setup_promise.set_value(setup);
                if (setup.success) {
                    function();
                }
            });
    } catch (const std::exception& exception) {
        error = std::string("failed to create thread: ") + exception.what();
        return false;
    } catch (...) {
        error = "failed to create thread";
        return false;
    }

    ThreadSetupResult setup;
    try {
        setup = setup_future.get();
    } catch (const std::exception& exception) {
        error = std::string("thread setup handshake failed: ") +
                exception.what();
        if (output.joinable()) {
            output.join();
        }
        return false;
    }
    if (!setup.success) {
        error = setup.error;
        if (output.joinable()) {
            output.join();
        }
        return false;
    }

    error.clear();
    return true;
}

}  // namespace robot_base
