#include "policy/policy_runtime.hpp"
#include "robot/robot_config.hpp"
#include "tool/tool.hpp"

#include <torch/script.h>
#include <ATen/Parallel.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <dlfcn.h>
#include <exception>
#include <filesystem>
#include <memory>
#include <sched.h>
#include <set>
#include <sstream>
#include <utility>
#include <unistd.h>

namespace inference {

namespace {

std::int64_t policy_runtime_now_ns() noexcept
{
    return robot_base::monotonic_now_ns();
}

bool tensor_is_valid_policy_output(const torch::Tensor& tensor,
                                   std::size_t expected_count,
                                   std::string& error)
{
    if (tensor.scalar_type() != torch::kFloat32) {
        std::ostringstream oss;
        oss << "TorchScript policy output must be float32, got scalar_type="
            << tensor.scalar_type();
        error = oss.str();
        return false;
    }
    if (tensor.numel() != static_cast<int64_t>(expected_count)) {
        std::ostringstream oss;
        oss << "TorchScript policy output size mismatch, expected "
            << expected_count << " got " << tensor.numel();
        error = oss.str();
        return false;
    }
    return true;
}

bool list_process_task_ids(std::set<pid_t>& task_ids, std::string& error)
{
    task_ids.clear();
    try {
        for (const auto& entry :
             std::filesystem::directory_iterator("/proc/self/task")) {
            const std::string name = entry.path().filename().string();
            std::size_t parsed = 0;
            const long value = std::stol(name, &parsed);
            if (parsed == name.size() && value > 0) {
                task_ids.insert(static_cast<pid_t>(value));
            }
        }
    } catch (const std::exception& exception) {
        error = std::string("failed to enumerate process threads: ") +
                exception.what();
        return false;
    }
    return true;
}

bool build_cpu_mask(const std::vector<int>& cpu_ids,
                    cpu_set_t& mask,
                    std::string& error)
{
    CPU_ZERO(&mask);
    if (cpu_ids.empty()) {
        error = "expected Torch worker CPU set is empty";
        return false;
    }
    for (const int cpu : cpu_ids) {
        if (cpu < 0 || cpu >= CPU_SETSIZE) {
            error = "expected Torch worker CPU is outside cpu_set_t";
            return false;
        }
        CPU_SET(cpu, &mask);
    }
    return true;
}

bool verify_task_affinity(pid_t task_id,
                          const cpu_set_t& expected,
                          bool exact,
                          std::string& error)
{
    cpu_set_t actual;
    CPU_ZERO(&actual);
    if (::sched_getaffinity(task_id, sizeof(actual), &actual) != 0) {
        if (errno == ESRCH) {
            return true;
        }
        error = "sched_getaffinity failed for TID " +
                std::to_string(task_id) + ": " + std::strerror(errno);
        return false;
    }
    if (exact) {
        if (!CPU_EQUAL(&actual, &expected)) {
            error = "policy thread affinity readback does not match its profile";
            return false;
        }
        return true;
    }
    for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
        if (CPU_ISSET(cpu, &actual) && !CPU_ISSET(cpu, &expected)) {
            error = "Torch/OpenMP worker TID " + std::to_string(task_id) +
                    " can run outside the configured policy CPU set";
            return false;
        }
    }
    return true;
}

}  // namespace

struct PolicyRuntime::Impl {
    std::unique_ptr<torch::jit::script::Module> module;
};

PolicyRuntime::PolicyRuntime()
    : impl_(std::make_unique<Impl>())
{
}

PolicyRuntime::~PolicyRuntime()
{
    shutdown();
}

bool PolicyRuntime::load(const PolicyRuntimeConfig& config,
                         int intra_op_threads,
                         int inter_op_threads,
                         int openblas_threads,
                         const std::vector<int>& expected_worker_cpus)
{
    unload();
    last_error_.clear();

    std::set<pid_t> tasks_before;
    cpu_set_t expected_worker_mask;
    if (!expected_worker_cpus.empty()) {
        std::string affinity_error;
        if (!build_cpu_mask(expected_worker_cpus,
                            expected_worker_mask,
                            affinity_error) ||
            !verify_task_affinity(::gettid(),
                                  expected_worker_mask,
                                  true,
                                  affinity_error) ||
            !list_process_task_ids(tasks_before, affinity_error)) {
            set_error(affinity_error);
            return false;
        }
    }

    if (!configure_parallel_runtime(intra_op_threads,
                                    inter_op_threads,
                                    openblas_threads)) {
        return false;
    }

    try {
        impl_->module = std::make_unique<torch::jit::script::Module>(
            torch::jit::load(config.model_path, torch::kCPU));
        impl_->module->eval();
        loaded_ = true;

        if (!dry_run_and_validate_output()) {
            unload();
            return false;
        }

        if (!expected_worker_cpus.empty()) {
            std::set<pid_t> tasks_after;
            std::string affinity_error;
            if (!list_process_task_ids(tasks_after, affinity_error)) {
                set_error(affinity_error);
                unload();
                return false;
            }
            for (const pid_t task_id : tasks_after) {
                if (tasks_before.count(task_id) == 0U &&
                    !verify_task_affinity(task_id,
                                          expected_worker_mask,
                                          false,
                                          affinity_error)) {
                    set_error(affinity_error);
                    unload();
                    return false;
                }
            }
        }
        
    } catch (const c10::Error& e) {
        set_error("failed to load TorchScript model: " + std::string(e.what()));
        unload();
        return false;
    } catch (const std::exception& e) {
        set_error("failed to load TorchScript model: " + std::string(e.what()));
        unload();
        return false;
    }

    return true;
}

bool PolicyRuntime::configure_parallel_runtime(int intra_op_threads,
                                               int inter_op_threads,
                                               int openblas_threads)
{
    if (intra_op_threads <= 0 && inter_op_threads <= 0 && openblas_threads <= 0) {
        return true;
    }
    if (intra_op_threads <= 0 || inter_op_threads <= 0 || openblas_threads <= 0) {
        set_error("parallel runtime thread counts must all be positive");
        return false;
    }

    try {
        const std::string intra = std::to_string(intra_op_threads);
        const std::string openblas = std::to_string(openblas_threads);
        if (::setenv("OMP_NUM_THREADS", intra.c_str(), 1) != 0 ||
            ::setenv("OMP_DYNAMIC", "FALSE", 1) != 0 ||
            ::setenv("OPENBLAS_NUM_THREADS", openblas.c_str(), 1) != 0) {
            set_error("failed to set parallel runtime environment");
            return false;
        }

        using OmpSetDynamic = void (*)(int);
        using OmpGetDynamic = int (*)();
        const auto omp_set_dynamic_fn = reinterpret_cast<OmpSetDynamic>(
            ::dlsym(RTLD_DEFAULT, "omp_set_dynamic"));
        const auto omp_get_dynamic_fn = reinterpret_cast<OmpGetDynamic>(
            ::dlsym(RTLD_DEFAULT, "omp_get_dynamic"));
        if (!omp_set_dynamic_fn || !omp_get_dynamic_fn) {
            set_error("OpenMP runtime controls are unavailable");
            return false;
        }
        omp_set_dynamic_fn(0);

        using OpenblasSetThreads = void (*)(int);
        using OpenblasGetThreads = int (*)();
        const auto openblas_set_threads = reinterpret_cast<OpenblasSetThreads>(
            ::dlsym(RTLD_DEFAULT, "openblas_set_num_threads"));
        const auto openblas_get_threads = reinterpret_cast<OpenblasGetThreads>(
            ::dlsym(RTLD_DEFAULT, "openblas_get_num_threads"));
        if (!openblas_set_threads || !openblas_get_threads) {
            set_error("OpenBLAS runtime controls are unavailable");
            return false;
        }
        openblas_set_threads(openblas_threads);

        at::set_num_threads(intra_op_threads);
        if (at::get_num_interop_threads() != inter_op_threads) {
            at::set_num_interop_threads(inter_op_threads);
        }

        if (at::get_num_threads() != intra_op_threads ||
            at::get_num_interop_threads() != inter_op_threads ||
            openblas_get_threads() != openblas_threads ||
            omp_get_dynamic_fn() != 0) {
            set_error("parallel runtime thread configuration verification failed");
            return false;
        }
    } catch (const c10::Error& error) {
        set_error("failed to configure parallel runtime: " +
                  std::string(error.what()));
        return false;
    } catch (const std::exception& error) {
        set_error("failed to configure parallel runtime: " +
                  std::string(error.what()));
        return false;
    }

    return true;
}

void PolicyRuntime::shutdown()
{
    unload();
}

void PolicyRuntime::unload()
{
    if (impl_) {
        impl_->module.reset();
    }
    loaded_ = false;
}

bool PolicyRuntime::is_loaded() const
{
    return loaded_ && impl_ && impl_->module;
}

bool PolicyRuntime::dry_run_and_validate_output()
{
    if (!is_loaded()) {
        set_error("TorchScript policy is not loaded");
        return false;
    }

    try {
        torch::NoGradGuard no_grad;
        torch::Tensor input = torch::zeros(
            {1, static_cast<int64_t>(kObservationSize)},
            torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCPU));

        torch::jit::IValue output_value = impl_->module->forward({input});
        if (!output_value.isTensor()) {
            set_error("TorchScript policy output must be a tensor");
            return false;
        }

        std::string output_error;
        if (!tensor_is_valid_policy_output(output_value.toTensor(),
                                           kDof,
                                           output_error)) {
            set_error(output_error);
            return false;
        }
    } catch (const c10::Error& e) {
        set_error("TorchScript dry-run failed: " + std::string(e.what()));
        return false;
    } catch (const std::exception& e) {
        set_error("TorchScript dry-run failed: " + std::string(e.what()));
        return false;
    }

    return true;
}

bool PolicyRuntime::infer(const PolicyObservation& observation,
                          PolicyRuntimeStepResult& result)
{
    if (!is_loaded()) {
        set_error("policy is not loaded");
        return false;
    }

    result.inference_start_ns = policy_runtime_now_ns();
    try {
        torch::NoGradGuard no_grad;
        torch::Tensor input = torch::from_blob(
            const_cast<float*>(observation.data()),
            {1, static_cast<int64_t>(observation.size())},
            torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCPU));

        torch::jit::IValue output_value = impl_->module->forward({input});
        torch::Tensor output = output_value.toTensor();

        output = output.to(torch::kCPU).contiguous();
        const float* output_data = output.data_ptr<float>();
        std::copy(output_data, output_data + result.raw_action.size(),
                  result.raw_action.begin());
    } catch (const c10::Error& e) {
        result.inference_end_ns = policy_runtime_now_ns();
        set_error("TorchScript inference failed: " + std::string(e.what()));
        return false;
    } catch (const std::exception& e) {
        result.inference_end_ns = policy_runtime_now_ns();
        set_error("TorchScript inference failed: " + std::string(e.what()));
        return false;
    }

    result.inference_end_ns = policy_runtime_now_ns();
    if (!std::all_of(result.raw_action.begin(),
                     result.raw_action.end(),
                     [](float value) { return std::isfinite(value); })) {
        set_error("policy output contains non-finite value");
        return false;
    }

    last_error_.clear();
    return true;
}

void PolicyRuntime::set_error(std::string message)
{
    last_error_ = std::move(message);
}

}  // namespace inference
