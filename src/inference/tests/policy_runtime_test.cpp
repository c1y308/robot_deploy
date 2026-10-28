#include "config/deploy_config.hpp"
#include "policy/policy_runtime.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

void check_contracts(const std::filesystem::path& fixtures)
{
    inference::PolicyRuntime policy;
    inference::PolicyRuntimeConfig config;
    inference::RuntimeThreadingConfig runtime;
    inference::PolicyObservation observation{};
    inference::PolicyRuntimeStepResult result;
    require(!policy.is_loaded(), "new policy should be unloaded");
    require(!policy.infer(observation, result), "unloaded inference should fail");
    for (const char* name : {"missing", "corrupt", "wrong_input_size",
                             "wrong_output_size", "wrong_input_type",
                             "wrong_output_type", "wrong_rank", "two_inputs",
                             "two_outputs", "warmup_nonfinite"}) {
        config.model_path = (fixtures / (std::string(name) + ".onnx")).string();
        require(!policy.load(config, runtime), std::string(name) + " should fail load");
        require(!policy.is_loaded(), "failed load retained a session");
        require(!policy.last_error().empty(), "missing load diagnostic");
    }
    config.model_path = "policy.pt";
    require(!policy.load(config, runtime), "PT path should be rejected");
    config.model_path = (fixtures / "slice675.onnx").string();
    runtime.onnx_intra_op_threads = 3;
    require(!policy.load(config, runtime), "unsupported thread count accepted");
    runtime.onnx_intra_op_threads = 2;
    require(!policy.load(config, runtime), "unbound two-thread mode accepted");
    runtime.onnx_intra_op_threads = 1;

    for (const char* name : {"slice675", "slice705", "dynamic_batch"}) {
        config.gait.enabled = std::string(name) == "slice705";
        config.model_path = (fixtures / (std::string(name) + ".onnx")).string();
        require(policy.load(config, runtime), policy.last_error());
        for (std::size_t i = 0; i < observation.size(); ++i)
            observation[i] = static_cast<float>(i) * 0.01F;
        // Unused capacity is not part of the 675-dimensional input.
        if (!config.gait.enabled) observation.back() = std::numeric_limits<float>::quiet_NaN();
        require(policy.infer(observation, result), policy.last_error());
        for (std::size_t i = 0; i < result.raw_action.size(); ++i)
            require(result.raw_action[i] == observation[i], "slice output/order mismatch");
        require(result.inference_start_ns > 0 &&
                    result.inference_end_ns >= result.inference_start_ns, "invalid timestamps");
        for (float invalid : {std::numeric_limits<float>::infinity(),
                              std::numeric_limits<float>::quiet_NaN()}) {
            observation[0] = invalid;
            require(!policy.infer(observation, result), "non-finite input accepted");
            require(policy.last_error().find("input") != std::string::npos, "input diagnostic missing");
        }
        observation[0] = 0;
        require(policy.infer(observation, result), policy.last_error());
        require(policy.last_error().empty(), "success retained stale error");
        policy.shutdown();
        policy.shutdown();
        require(!policy.is_loaded(), "shutdown retained session");
    }

    config.gait.enabled = false;
    observation.fill(0);
    config.model_path = (fixtures / "output_overflow.onnx").string();
    require(policy.load(config, runtime), policy.last_error());
    observation[0] = 2;
    require(!policy.infer(observation, result), "non-finite output accepted");
    require(policy.last_error().find("output") != std::string::npos, "output diagnostic missing");
    observation.fill(0);
    config.model_path = (fixtures / "changing_output_batch.onnx").string();
    require(policy.load(config, runtime), policy.last_error());
    observation[1] = 2;
    require(!policy.infer(observation, result), "oversized runtime output accepted");
    require(policy.last_error().find("ONNX inference failed") != std::string::npos,
            "Run shape diagnostic missing");
    // A failed reload releases the previous good model too.
    config.model_path = (fixtures / "missing.onnx").string();
    require(!policy.load(config, runtime) && !policy.is_loaded(), "failed reload retained model");
}

void check_actual_model()
{
    inference::RobotInterfaceConfig config;
    std::string error;
    require(inference::load_deploy_config(ROBOT_DEPLOY_CONFIG_PATH, config, error), error);
    inference::PolicyRuntime policy;
    require(policy.load(config.policy, config.runtime), policy.last_error());

    // Independent allocating ORT path checks names, shapes and every output;
    // production uses persistent tensors/buffers instead.
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "policy_reference");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    Ort::Session reference(env, config.policy.model_path.c_str(), options);
    Ort::AllocatorWithDefaultOptions allocator;
    auto input_name = reference.GetInputNameAllocated(0, allocator);
    auto output_name = reference.GetOutputNameAllocated(0, allocator);
    const char* inputs[] = {input_name.get()};
    const char* outputs[] = {output_name.get()};
    const auto input_shape = reference.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    const auto output_shape = reference.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    const auto input_size = inference::policy_observation::observation_size(config.policy.gait.enabled);
    require(input_shape == std::vector<int64_t>({1, static_cast<int64_t>(input_size)}),
            "actual model input differs from deploy.yaml");
    require(output_shape == std::vector<int64_t>({1, 12}), "actual output shape mismatch");
    std::cout << "model " << inputs[0] << " float32[1," << input_size << "] -> "
              << outputs[0] << " float32[1,12]\n";
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::RunOptions run_options;
    for (int frame = 0; frame < 100; ++frame) {
        inference::PolicyObservation observation{};
        for (std::size_t i = 0; i < input_size; ++i)
            observation[i] = std::sin(float(frame * (i + 1)) * 0.001F);
        auto input = Ort::Value::CreateTensor<float>(memory, observation.data(), input_size,
                                                     input_shape.data(), input_shape.size());
        auto expected = reference.Run(run_options, inputs, &input, 1, outputs, 1);
        inference::PolicyRuntimeStepResult result;
        require(policy.infer(observation, result), policy.last_error());
        const float* data = expected[0].GetTensorData<float>();
        for (std::size_t i = 0; i < result.raw_action.size(); ++i)
            require(std::abs(result.raw_action[i] - data[i]) <= 1e-6F,
                    "actual model output differs from reference");
    }
}

void check_dual_mode()
{
    cpu_set_t allowed;
    CPU_ZERO(&allowed);
    require(sched_getaffinity(0, sizeof(allowed), &allowed) == 0, "read allowed CPUs failed");
    std::vector<int> cpus;
    for (int cpu = 0; cpu < CPU_SETSIZE && cpus.size() < 2; ++cpu)
        if (CPU_ISSET(cpu, &allowed)) cpus.push_back(cpu);
    if (cpus.size() < 2) {
        std::cout << "dual-mode check skipped: fewer than two allowed CPUs\n";
        return;
    }
    inference::RuntimeThreadingConfig runtime;
    runtime.policy_main.cpu_ids = {cpus[0]};
    runtime.onnx_intra_op_threads = 2;
    runtime.onnx_worker_cpu_id = cpus[1];
    inference::RobotInterfaceConfig deploy;
    std::string error;
    require(inference::load_deploy_config(ROBOT_DEPLOY_CONFIG_PATH, deploy, error), error);
    const auto& config = deploy.policy;
    bool success = false;
    std::thread thread;
    require(robot_base::start_configured_thread(thread, "ort_dual_test", runtime.policy_main,
        [&] {
            for (int iteration = 0; iteration < 30; ++iteration) {
                inference::PolicyRuntime policy;
                inference::PolicyRuntimeStepResult result;
                success = policy.load(config, runtime) && policy.infer({}, result);
                error = policy.last_error();
                if (!success) break;
            }
        }, error), error);
    thread.join();
    require(success, error);
    std::cout << "dual mode verified across 30 loads: main CPU" << cpus[0] << ", worker CPU" << cpus[1] << '\n';
}

}  // namespace

int main(int argc, char** argv)
{
    try {
        require(argc == 2, "expected fixture directory");
        check_contracts(argv[1]);
        check_actual_model();
        check_dual_mode();
        std::cout << "policy_runtime_test passed\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
