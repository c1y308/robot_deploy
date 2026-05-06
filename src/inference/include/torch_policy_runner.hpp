#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>

namespace inference {

class TorchPolicyRunner {
public:
    static constexpr std::size_t kInputSize  = 47;
    static constexpr std::size_t kOutputSize = 12;

    TorchPolicyRunner();
    ~TorchPolicyRunner();

    TorchPolicyRunner(const TorchPolicyRunner&) = delete;
    TorchPolicyRunner& operator=(const TorchPolicyRunner&) = delete;

    bool load(const std::string& model_path);
    void unload();
    bool is_loaded() const { return loaded_; }

    bool infer(const std::array<float, kInputSize>& observation,
               std::array<float, kOutputSize>& action);

    const std::string& last_error() const { return last_error_; }

private:
    struct Impl;  // std::unique_ptr<torch::jit::script::Module> module;

    bool dry_run_and_validate_output();
    void set_error(const std::string& message);

    std::unique_ptr<Impl> impl_;
    bool loaded_ = false;
    std::string last_error_;
};

}  // namespace inference
