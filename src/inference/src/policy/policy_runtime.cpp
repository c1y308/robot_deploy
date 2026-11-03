#include "policy/policy_runtime.hpp"
#include "robot/robot_config.hpp"
#include "policy_session_options_impl.hpp"
#include "tool/tool.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <sstream>
#include <vector>

namespace inference {
namespace {

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
                         const PolicySessionOptions& session_options)
{
    shutdown();
    last_error_.clear();
    const std::size_t input_size =
        policy_observation::observation_size(config.gait.enabled);
    if (std::filesystem::path(config.model_path).extension() != ".onnx") {
        last_error_ = "policy model path must name an .onnx file";
        return false;
    }
    if (!session_options.impl_) {
        last_error_ = "ONNX session options are not prepared";
        return false;
    }

    try {
        impl_ = std::make_unique<Impl>();
        impl_->session = Ort::Session(
            impl_->env, config.model_path.c_str(), session_options.impl_->options);
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
