#pragma once

#include <pthread.h>
#include <sched.h>
#include <alloca.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstring>
#include <future>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace robot_base {

// 调度策略
enum class ThreadSchedulingPolicy {
    INHERIT,    // 继承，不修改当前线程的调度策略和优先级
    OTHER,      // SCHED_OTHER，普通调度策略，优先级为 0（用 nice 值）
    FIFO,       // SCHED_FIFO，实时调度策略，优先级范围为 [1, 99]
};

// 线程运行时选项
struct ThreadRuntimeOptions {
    std::vector<int> cpu_ids;   // 允许线程运行的 CPU 集合
    ThreadSchedulingPolicy scheduling_policy{ThreadSchedulingPolicy::INHERIT};  // 调度策略 
    int priority{0};    // 调度优先级
    std::size_t stack_prefault_bytes{0};  // 进入业务循环前预触碰的栈空间
};

// 线程设置结果（这里是否过度设计？）
struct ThreadSetupResult {
    bool success{false};
    int error_code{0};
    std::string error;
};


/// @param operation  操作名称
/// @param error_code 错误码
inline ThreadSetupResult thread_setup_error(const std::string& operation,
                                            int error_code)
{
    ThreadSetupResult result;
    result.error_code = error_code;
    result.error = operation + ": " +
                   (error_code != 0 ? std::strerror(error_code) : "verification failed");
    return result;
}

inline constexpr std::size_t kMaxStackPrefaultBytes = 1024U * 1024U;

// 预先建立当前线程的栈页映射。noinline 保证 alloca 的栈帧真实存在于本函数中，
// volatile 写入保证逐页触碰不会被优化器消除。
// 返回 0 或 errno 风格错误码；错误消息由调用者构造，使 helper 本身无堆分配。
[[gnu::noinline]] inline int prefault_current_thread_stack(
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


// 配置顺序固定为：名称与 affinity -> 栈预触碰 -> 调度策略与优先级。
inline ThreadSetupResult configure_current_thread(
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

    // 获取当前线程的 pthread_t 对象
    const pthread_t self = pthread_self();
    // 设置线程名称
    int result = pthread_setname_np(self, name);

    if (result != 0) {
        return thread_setup_error("pthread_setname_np", result);
    }

    char actual_name[16]{};
    result = pthread_getname_np(self, actual_name, sizeof(actual_name));
    if (result != 0) {
        return thread_setup_error("pthread_getname_np", result);
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


// 启动配置好的线程（用启动握手消除竞态，什么意思？）
template <typename Callable>
bool start_configured_thread(std::thread& output,   // 线程对象
                             const char*  name,     // 线程名称
                             const ThreadRuntimeOptions& options,   // 线程运行时选项
                             Callable&& callable,   // 线程执行函数
                             std::string& error)
{
    if (output.joinable()) {
        error = "thread is already joinable";
        return false;
    }

    // 一次性结果发送端（producer）
    std::promise<ThreadSetupResult> setup_promise;
    // 一次性结果接收端（consumer）
    std::future<ThreadSetupResult>  setup_future = setup_promise.get_future();

    try {
        output = std::thread(
            [name_string = std::string(name ? name : ""),
             options,
             setup_promise = std::move(setup_promise),
             function      = std::forward<Callable>(callable)
            ] () mutable {

                ThreadSetupResult setup;

                // 配置子线程
                try {
                    setup = configure_current_thread(name_string.c_str(), options);
                } catch (const std::exception& exception) {
                    setup.error = std::string("thread setup exception: ") +
                                  exception.what();
                } catch (...) {
                    setup.error = "thread setup exception";
                }

                // 传递配置结果给主线程
                setup_promise.set_value(setup);

                // 配置成功才执行线程函数（保证业务函数绝不会在配置成功前执行）
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

    // 父线程等待子线程配置完成（阻塞等待，直到子线程调用 set_value() 或者异常退出）
    ThreadSetupResult setup;
    try {
        setup = setup_future.get();  // 阻塞行为（休眠-唤醒）
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
