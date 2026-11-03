#pragma once

#include <memory>
#include <string>

namespace inference {

struct RuntimeThreadingConfig;
class PolicyRuntime;

// 不暴露 ORT 类型；默认构造的对象尚未准备。
class PolicySessionOptions {
public:
    PolicySessionOptions();
    ~PolicySessionOptions();
    PolicySessionOptions(PolicySessionOptions&&) noexcept;
    PolicySessionOptions& operator=(PolicySessionOptions&&) noexcept;
    PolicySessionOptions(const PolicySessionOptions&) = delete;
    PolicySessionOptions& operator=(const PolicySessionOptions&) = delete;

private:
    friend class PolicyRuntime;
    friend class PolicyExecutionSetup;
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// 同一所有者线程串行使用：加载前准备选项和线程基线，加载完成后核验 ORT worker。
// 准备、模型加载与核验期间，不得并发启动其他后台线程。
class PolicyExecutionSetup {
public:
    PolicyExecutionSetup();
    ~PolicyExecutionSetup();
    PolicyExecutionSetup(const PolicyExecutionSetup&) = delete;
    PolicyExecutionSetup& operator=(const PolicyExecutionSetup&) = delete;

    bool prepare(const RuntimeThreadingConfig& runtime, std::string& error);
    const PolicySessionOptions& session_options() const;
    bool verify_workers(std::string& error) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    PolicySessionOptions options_;
};

}  // namespace inference
