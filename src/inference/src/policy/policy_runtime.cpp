#include "policy/policy_runtime.hpp"
#include "robot/robot_config.hpp"
#include "tool/tool.hpp"

#include <torch/script.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <sstream>
#include <utility>

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

bool PolicyRuntime::load(const PolicyRuntimeConfig& config)
{
    unload();
    last_error_.clear();

    try {
        impl_->module = std::make_unique<torch::jit::script::Module>(
            torch::jit::load(config.model_path, torch::kCPU));
        impl_->module->eval();
        loaded_ = true;

        if (!dry_run_and_validate_output()) {
            unload();
            return false;
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
