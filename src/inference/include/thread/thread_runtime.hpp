#pragma once

#include <cstddef>
#include <future>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace robot_base {

// 线程启动握手的设置结果。
struct ThreadSetupResult {
    bool success{false};
    std::string error;
};

ThreadSetupResult thread_setup_error(const std::string& operation, int error_code);

inline constexpr std::size_t kMaxStackPrefaultBytes = 1024U * 1024U;

// 预触碰当前线程的栈页；返回 0 或 errno 风格错误码，不分配堆内存。
[[gnu::noinline]] int prefault_current_thread_stack(std::size_t bytes) noexcept;

// 调度策略
enum class ThreadSchedulingPolicy {
    INHERIT,    // 继承，不修改当前线程的调度策略和优先级
    OTHER,      // SCHED_OTHER，普通调度策略，优先级为 0（用 nice 值）
    FIFO,       // SCHED_FIFO，实时调度策略，优先级范围为 [1, 99]
};


// 线程运行时选项
struct ThreadRuntimeOptions {
    std::vector<int> cpu_ids;               // 允许线程运行的 CPU 集合
    ThreadSchedulingPolicy scheduling_policy{ThreadSchedulingPolicy::INHERIT};  // 调度策略 
    int priority{0};                        // 调度优先级
    std::size_t stack_prefault_bytes{0};    // 进入业务循环前预触碰的栈空间
};

// 配置顺序：名称与 affinity -> 栈预触碰 -> 调度策略与优先级。
ThreadSetupResult configure_current_thread(const char* name,
                                          const ThreadRuntimeOptions& options);

// 启动配置好的线程（用启动握手消除竞态）
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
