#include "policy/policy_execution_setup.hpp"
#include "policy_session_options_impl.hpp"
#include "robot/robot_config.hpp"

#include <onnxruntime_session_options_config_keys.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <sched.h>
#include <set>
#include <sstream>
#include <thread>

namespace inference {
namespace {

bool list_task_ids(std::set<pid_t>& ids, std::string& error)
{
    ids.clear();
    try {
        for (const auto& entry :
             std::filesystem::directory_iterator("/proc/self/task")) {
            const std::string name = entry.path().filename().string();
            std::size_t parsed = 0;
            const long value = std::stol(name, &parsed);
            if (parsed == name.size() && value > 0) {
                ids.insert(static_cast<pid_t>(value));
            }
        }
    } catch (const std::exception& e) {
        error = std::string("failed to enumerate process threads: ") + e.what();
        return false;
    }
    return true;
}

bool cpu_mask(const std::vector<int>& cpus, cpu_set_t& mask, std::string& error)
{
    CPU_ZERO(&mask);
    if (cpus.empty()) {
        error = "expected CPU set is empty";
        return false;
    }
    for (int cpu : cpus) {
        if (cpu < 0 || cpu >= CPU_SETSIZE) {
            error = "CPU id is outside cpu_set_t";
            return false;
        }
        CPU_SET(cpu, &mask);
    }
    return true;
}

std::string format_mask(const cpu_set_t& mask)
{
    std::ostringstream out;
    out << '{';
    bool first = true;
    for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
        if (!CPU_ISSET(cpu, &mask)) continue;
        if (!first) out << ',';
        out << cpu;
        first = false;
    }
    out << '}';
    return out.str();
}

bool verify_task(pid_t tid, const cpu_set_t& expected,
                 const char* role, std::string& error)
{
    cpu_set_t actual;
    CPU_ZERO(&actual);
    if (::sched_getaffinity(tid, sizeof(actual), &actual) != 0) {
        error = std::string("sched_getaffinity failed for ") + role +
                " TID " + std::to_string(tid) + ": " + std::strerror(errno);
        return false;
    }
    if (!CPU_EQUAL(&actual, &expected)) {
        error = std::string(role) + " TID " + std::to_string(tid) +
                " affinity=" + format_mask(actual) +
                " expected=" + format_mask(expected);
        return false;
    }
    sched_param param{};
    const int policy = ::sched_getscheduler(tid);
    if (policy < 0 || ::sched_getparam(tid, &param) != 0) {
        error = std::string("failed to read scheduler for ") + role +
                " TID " + std::to_string(tid) + ": " + std::strerror(errno);
        return false;
    }
    if (policy != SCHED_OTHER || param.sched_priority != 0) {
        error = std::string(role) + " TID " + std::to_string(tid) +
                " expected SCHED_OTHER/0";
        return false;
    }
    return true;
}

}  // namespace

struct PolicyExecutionSetup::Impl {
    bool dual{false};
    cpu_set_t main_mask{};
    cpu_set_t worker_mask{};
    std::set<pid_t> tasks_before;
};

PolicySessionOptions::PolicySessionOptions() = default;
PolicySessionOptions::~PolicySessionOptions() = default;
PolicySessionOptions::PolicySessionOptions(PolicySessionOptions&&) noexcept = default;
PolicySessionOptions& PolicySessionOptions::operator=(PolicySessionOptions&&) noexcept = default;

PolicyExecutionSetup::PolicyExecutionSetup() = default;
PolicyExecutionSetup::~PolicyExecutionSetup() = default;

bool PolicyExecutionSetup::prepare(const RuntimeThreadingConfig& runtime, std::string& error)
{
    impl_.reset();
    options_.impl_.reset();
    error.clear();
    if (runtime.onnx_intra_op_threads != 1 && runtime.onnx_intra_op_threads != 2) {
        error = "ONNX intra-op threads must be 1 or 2";
        return false;
    }
    const bool dual = runtime.onnx_intra_op_threads == 2;
    if (dual && (runtime.policy_main.cpu_ids.size() != 1 ||
                 runtime.onnx_worker_cpu_id < 0 ||
                 runtime.onnx_worker_cpu_id >= CPU_SETSIZE ||
                 runtime.policy_main.cpu_ids[0] == runtime.onnx_worker_cpu_id)) {
        error = "two-thread ONNX mode requires distinct main and worker CPUs";
        return false;
    }
    try {
        auto prepared = std::make_unique<Impl>();
        prepared->dual = dual;
        if (!runtime.policy_main.cpu_ids.empty() &&
            !cpu_mask(runtime.policy_main.cpu_ids, prepared->main_mask, error)) {
            return false;
        }
        // 在构造 ORT 对象前记录基线，准备、模型加载和核验期间不得启动其他后台线程。
        if (dual) {
            CPU_SET(runtime.onnx_worker_cpu_id, &prepared->worker_mask);
            if (!list_task_ids(prepared->tasks_before, error)) return false;
        }
        auto options = std::make_unique<PolicySessionOptions::Impl>();
        options->options.SetIntraOpNumThreads(runtime.onnx_intra_op_threads);
        options->options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
        options->options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        options->options.AddConfigEntry("session.intra_op.allow_spinning", "0");
        options->options.AddConfigEntry("session.inter_op.allow_spinning", "0");
        if (dual) {
            // ORT affinity 使用从 1 开始的编号，Linux CPU ID 从 0 开始。
            const std::string cpu = std::to_string(runtime.onnx_worker_cpu_id + 1);
            options->options.AddConfigEntry(
                kOrtSessionOptionsConfigIntraOpThreadAffinities, cpu.c_str());
        }
        options_.impl_ = std::move(options);
        impl_ = std::move(prepared);
    } catch (const std::exception& exception) {
        error = std::string("failed to prepare ONNX execution: ") + exception.what();
        return false;
    }
    return true;
}

const PolicySessionOptions& PolicyExecutionSetup::session_options() const
{
    return options_;
}

bool PolicyExecutionSetup::verify_workers(std::string& error) const
{
    error.clear();
    if (!impl_ || !options_.impl_) {
        error = "ONNX execution setup is not prepared";
        return false;
    }
    if (!impl_->dual) return true;

    // affinity 设置是异步的；保持原有 100 ms 就绪窗口和 1 ms 重试间隔。
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    std::set<pid_t> tasks_after;
    std::string setup_error;
    for (;;) {
        if (!list_task_ids(tasks_after, error)) return false;
        std::size_t workers = 0;
        bool tasks_valid = true;
        for (pid_t tid : tasks_after) {
            if (impl_->tasks_before.count(tid)) continue;
            if (verify_task(tid, impl_->worker_mask, "ONNX worker", setup_error)) {
                ++workers;
            } else if (!verify_task(tid, impl_->main_mask,
                                    "ONNX auxiliary thread", setup_error)) {
                tasks_valid = false;
                break;
            }
        }
        if (tasks_valid && workers == 1) return true;
        if (std::chrono::steady_clock::now() >= deadline) {
            error = tasks_valid
                ? "expected exactly one ONNX worker, found " + std::to_string(workers)
                : setup_error;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

}  // namespace inference
