#include "thread/thread_runtime.hpp"

#include <alloca.h>
#include <cerrno>
#include <cstring>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

namespace robot_base {

ThreadSetupResult thread_setup_error(const std::string& operation,
                                    int error_code)
{
    ThreadSetupResult result;
    result.error = operation + ": " +
                   (error_code != 0 ? std::strerror(error_code) : "verification failed");
    return result;
}

[[gnu::noinline]] int prefault_current_thread_stack(
    std::size_t bytes) noexcept
{
    if (bytes == 0U) {
        return 0;
    }
    if (bytes > kMaxStackPrefaultBytes) {
        return E2BIG;
    }

    errno = 0;
    const long page_size_value = ::sysconf(_SC_PAGESIZE);
    if (page_size_value <= 0) {
        return errno != 0 ? errno : EINVAL;
    }

    const std::size_t page_size = static_cast<std::size_t>(page_size_value);
    const std::size_t rounded_bytes =
        ((bytes + page_size - 1U) / page_size) * page_size;
    volatile unsigned char* const stack_pages =
        static_cast<volatile unsigned char*>(::alloca(rounded_bytes));
    for (std::size_t offset = 0; offset < rounded_bytes; offset += page_size) {
        stack_pages[offset] = 0U;
    }
    stack_pages[rounded_bytes - 1U] = 0U;

    return 0;
}

ThreadSetupResult configure_current_thread(
    const char* name,
    const ThreadRuntimeOptions& options)
{
    // 检测线程名称是否为空
    if (!name || name[0] == '\0') {
        return thread_setup_error("thread name is empty", EINVAL);
    }
    // 检测线程名称长度是否超过 Linux 的 15 字节限制
    if (std::strlen(name) > 15U) {
        return thread_setup_error("thread name exceeds Linux 15-byte limit", ERANGE);
    }

    // 获取当前线程的 pthread_t 对象 设置线程名称
    const pthread_t self = pthread_self();
    int result = pthread_setname_np(self, name);
    if (result != 0) {
        return thread_setup_error("pthread_setname_np", result);
    }

    // 进行 CPU 绑核
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

        // 验证 CPU 绑核是否成功
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

    // 在进入可能的 FIFO 实时调度前建立所需栈页映射。
    const int prefault_error =
        prefault_current_thread_stack(options.stack_prefault_bytes);
    if (prefault_error != 0) {
        return thread_setup_error("stack prefault (maximum 1 MiB; valid page size required)",
                                  prefault_error);
    }

    // 设置线程调度策略和优先级（采用带有时间片的是否更好？）
    if (options.scheduling_policy != ThreadSchedulingPolicy::INHERIT) {
        const int policy = options.scheduling_policy == ThreadSchedulingPolicy::FIFO
                               ? SCHED_FIFO
                               : SCHED_OTHER;
        // OTHER 策略的优先级必须为 0，FIFO 策略的优先级必须在 [1, 99] 范围内
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

        // 设置调度策略与优先级
        sched_param requested{};
        requested.sched_priority = options.priority;
        result = pthread_setschedparam(self, policy, &requested);
        if (result != 0) {
            return thread_setup_error("pthread_setschedparam", result);
        }

        // 验证调度策略与优先级是否设置成功
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

}  // namespace robot_base
