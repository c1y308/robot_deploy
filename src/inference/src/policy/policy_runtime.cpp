#include "policy/policy_runtime.hpp"
#include "robot/robot_config.hpp"
#include "tool/tool.hpp"

#include <onnxruntime_cxx_api.h>
#include <onnxruntime_session_options_config_keys.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <sched.h>
#include <set>
#include <sstream>
#include <thread>
#include <unistd.h>

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

bool validate_port(const Ort::TypeInfo& type,
                   const std::vector<int64_t>& expected,
                   const char* role, std::string& error)
{
    if (type.GetONNXType() != ONNX_TYPE_TENSOR) {
        error = std::string("ONNX ") + role + " must be a tensor";
        return false;
    }
    const auto info = type.GetTensorTypeAndShapeInfo();
    if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        error = std::string("ONNX ") + role + " must be float32";
        return false;
    }
    const auto shape = info.GetShape();
    // Model metadata may declare a dynamic batch; runtime tensors use batch 1.
    if (shape.size() != expected.size() ||
        (shape[0] != expected[0] && shape[0] != -1) ||
        !std::equal(shape.begin() + 1, shape.end(), expected.begin() + 1)) {
        std::ostringstream out;
        out << "ONNX " << role << " shape mismatch: expected [";
        for (std::size_t i = 0; i < expected.size(); ++i) {
            if (i) out << ',';
            out << expected[i];
        }
        out << "], got [";
        for (std::size_t i = 0; i < shape.size(); ++i) {
            if (i) out << ',';
            out << shape[i];
        }
        out << ']';
        error = out.str();
        return false;
    }
    return true;
}

}  // namespace

struct PolicyRuntime::Impl {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "robot_policy"};
    Ort::Session session{nullptr};
    Ort::RunOptions run_options;
    std::string input_name;
    std::string output_name;
    const char* input_names[1]{nullptr};
    const char* output_names[1]{nullptr};
    std::size_t input_size{0};
    PolicyObservation input_buffer{};
    PolicyAction output_buffer{};
    Ort::Value input_tensor{nullptr};
    Ort::Value output_tensor{nullptr};
};

PolicyRuntime::PolicyRuntime() = default;
PolicyRuntime::~PolicyRuntime() = default;

bool PolicyRuntime::load(const PolicyRuntimeConfig& config,
                         const RuntimeThreadingConfig& runtime)
{
    shutdown();
    last_error_.clear();
    const std::size_t input_size =
        policy_observation::observation_size(config.gait.enabled);
    if (std::filesystem::path(config.model_path).extension() != ".onnx") {
        last_error_ = "policy model path must name an .onnx file";
        return false;
    }
    if (runtime.onnx_intra_op_threads != 1 &&
        runtime.onnx_intra_op_threads != 2) {
        last_error_ = "ONNX intra-op threads must be 1 or 2";
        return false;
    }
    const bool dual = runtime.onnx_intra_op_threads == 2;
    if (dual && (runtime.policy_main.cpu_ids.size() != 1 ||
                 runtime.onnx_worker_cpu_id < 0 ||
                 runtime.onnx_worker_cpu_id >= CPU_SETSIZE ||
                 runtime.policy_main.cpu_ids[0] == runtime.onnx_worker_cpu_id)) {
        last_error_ = "two-thread ONNX mode requires distinct main and worker CPUs";
        return false;
    }

    std::string setup_error;
    if (!runtime.policy_main.cpu_ids.empty()) {
        cpu_set_t main_mask;
        if (!cpu_mask(runtime.policy_main.cpu_ids, main_mask, setup_error) ||
            !verify_task(::gettid(), main_mask, "policy thread", setup_error)) {
            last_error_ = setup_error;
            return false;
        }
    }
    std::set<pid_t> tasks_before;
    if (dual && !list_task_ids(tasks_before, setup_error)) {
        last_error_ = setup_error;
        return false;
    }

    try {
        impl_ = std::make_unique<Impl>();
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(runtime.onnx_intra_op_threads);
        options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
        options.SetGraphOptimizationLevel(
            GraphOptimizationLevel::ORT_ENABLE_ALL);
        options.AddConfigEntry("session.intra_op.allow_spinning", "0");
        options.AddConfigEntry("session.inter_op.allow_spinning", "0");
        if (dual) {
            // ORT's affinity string uses one-based processor numbers; Linux
            // affinity APIs and RuntimeThreadingConfig use zero-based IDs.
            // ORT sets this only on the extra intra-op worker.
            const std::string cpu =
                std::to_string(runtime.onnx_worker_cpu_id + 1);
            options.AddConfigEntry(
                kOrtSessionOptionsConfigIntraOpThreadAffinities, cpu.c_str());
        }
        impl_->session = Ort::Session(
            impl_->env, config.model_path.c_str(), options);
        if (impl_->session.GetInputCount() != 1 ||
            impl_->session.GetOutputCount() != 1) {
            last_error_ = "ONNX policy requires exactly one input and one output";
            shutdown();
            return false;
        }
        std::string port_error;
        if (!validate_port(impl_->session.GetInputTypeInfo(0),
                           {1, static_cast<int64_t>(input_size)},
                           "input", port_error) ||
            !validate_port(impl_->session.GetOutputTypeInfo(0),
                           {1, static_cast<int64_t>(kDof)},
                           "output", port_error)) {
            last_error_ = port_error;
            shutdown();
            return false;
        }

        Ort::AllocatorWithDefaultOptions allocator;
        impl_->input_name =
            impl_->session.GetInputNameAllocated(0, allocator).get();
        impl_->output_name =
            impl_->session.GetOutputNameAllocated(0, allocator).get();
        if (impl_->input_name.empty() || impl_->output_name.empty()) {
            last_error_ = "ONNX input or output name is empty";
            shutdown();
            return false;
        }
        impl_->input_names[0] = impl_->input_name.c_str();
        impl_->output_names[0] = impl_->output_name.c_str();

        const std::array<int64_t, 2> input_shape{
            1, static_cast<int64_t>(input_size)};
        constexpr std::array<int64_t, 2> output_shape{
            1, static_cast<int64_t>(kDof)};
        const auto memory =
            Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        impl_->input_tensor = Ort::Value::CreateTensor<float>(
            memory, impl_->input_buffer.data(), input_size,
            input_shape.data(), input_shape.size());
        impl_->output_tensor = Ort::Value::CreateTensor<float>(
            memory, impl_->output_buffer.data(), impl_->output_buffer.size(),
            output_shape.data(), output_shape.size());
        impl_->input_size = input_size;
        if (!warm_up()) {
            shutdown();
            return false;
        }

        if (dual) {
            cpu_set_t worker_mask;
            CPU_ZERO(&worker_mask);
            CPU_SET(runtime.onnx_worker_cpu_id, &worker_mask);
            cpu_set_t main_mask;
            if (!cpu_mask(runtime.policy_main.cpu_ids, main_mask, setup_error)) {
                last_error_ = setup_error;
                shutdown();
                return false;
            }
            // ORT 设置 worker affinity 是异步的；短预热不保证 worker 已调度。
            // 仅在硬件启动前等待有界的就绪窗口，超时仍拒绝加载。
            const auto deadline = std::chrono::steady_clock::now() +
                                  std::chrono::milliseconds(100);
            std::set<pid_t> tasks_after;
            for (;;) {
                if (!list_task_ids(tasks_after, setup_error)) {
                    last_error_ = setup_error;
                    shutdown();
                    return false;
                }
                std::size_t workers = 0;
                bool tasks_valid = true;
                for (pid_t tid : tasks_after) {
                    if (tasks_before.count(tid)) continue;
                    if (verify_task(tid, worker_mask, "ONNX worker", setup_error)) {
                        ++workers;
                    } else if (!verify_task(tid, main_mask,
                                            "ONNX auxiliary thread", setup_error)) {
                        tasks_valid = false;
                        break;
                    }
                }
                if (tasks_valid && workers == 1) break;
                if (std::chrono::steady_clock::now() >= deadline) {
                    last_error_ = tasks_valid
                        ? "expected exactly one ONNX worker, found " + std::to_string(workers)
                        : setup_error;
                    shutdown();
                    return false;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    } catch (const std::exception& e) {
        last_error_ = "failed to load ONNX policy: " + std::string(e.what());
        shutdown();
        return false;
    }
    return true;
}

void PolicyRuntime::shutdown()
{
    impl_.reset();
}

bool PolicyRuntime::is_loaded() const
{
    return impl_ != nullptr;
}

bool PolicyRuntime::warm_up()
{
    try {
        // This happens before any motor or IMU startup, outside the control loop.
        for (int i = 0; i < 20; ++i) {
            impl_->session.Run(impl_->run_options,
                               impl_->input_names, &impl_->input_tensor, 1,
                               impl_->output_names, &impl_->output_tensor, 1);
        }
        if (!std::all_of(impl_->output_buffer.begin(),
                         impl_->output_buffer.end(),
                         [](float x) { return std::isfinite(x); })) {
            last_error_ = "ONNX warmup output contains non-finite value";
            return false;
        }
    } catch (const Ort::Exception& e) {
        last_error_ = "ONNX warmup failed: " + std::string(e.what());
        return false;
    }
    return true;
}

bool PolicyRuntime::infer(const PolicyObservation& observation,
                          PolicyRuntimeStepResult& result)
{
    if (!is_loaded()) {
        last_error_ = "policy is not loaded";
        return false;
    }
    result.inference_start_ns = robot_base::monotonic_now_ns();
    try {
        if (!std::all_of(observation.begin(), observation.begin() + impl_->input_size,
                         [](float x) { return std::isfinite(x); })) {
            result.inference_end_ns = robot_base::monotonic_now_ns();
            last_error_ = "policy input contains non-finite value";
            return false;
        }
        std::copy_n(observation.begin(), impl_->input_size,
                    impl_->input_buffer.begin());
        impl_->session.Run(impl_->run_options,
                           impl_->input_names, &impl_->input_tensor, 1,
                           impl_->output_names, &impl_->output_tensor, 1);
        // load 已校验 float32 [1,12]；Run 会校验预分配输出的类型和形状。
        // 不在每帧重新分配/查询 TensorTypeAndShapeInfo。
        std::copy(impl_->output_buffer.begin(), impl_->output_buffer.end(),
                  result.raw_action.begin());
    } catch (const std::exception& e) {
        result.inference_end_ns = robot_base::monotonic_now_ns();
        last_error_ = "ONNX inference failed: " + std::string(e.what());
        return false;
    }
    result.inference_end_ns = robot_base::monotonic_now_ns();
    if (!std::all_of(result.raw_action.begin(), result.raw_action.end(),
                     [](float x) { return std::isfinite(x); })) {
        last_error_ = "policy output contains non-finite value";
        return false;
    }
    last_error_.clear();
    return true;
}

}  // namespace inference
