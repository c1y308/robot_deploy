#pragma once

#include "motor_base/command_types.hpp"
#include "robot/robot_config.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace motor_base {
class MotorControllerBase;
}

namespace inference {

enum class ShutdownResult {
    Confirmed,              // stop 命令执行成功
    ReleasedWithCommLoss,   // stop 命令执行失败且通信故障 下 控制器释放
    RetryRequired,          // stop 命令执行失败但通信正常，需重试
};

// inference层需要的电机数据，从底层的 MotorStatusSnapshot 获取
struct MotorStateSnapshot {
    std::int64_t timestamp_ns{0};   // 时间戳

    std::array<double, motor_base::kMotorNum> position_rad{};   // 位置
    std::array<double, motor_base::kMotorNum> velocity_rad_s{}; // 速度
    std::array<double, motor_base::kMotorNum> torque_percent{}; // 力矩

    std::array<std::uint8_t, motor_base::kMotorNum> comm_ok{};  // 通信状态
    std::array<std::uint8_t, motor_base::kMotorNum> enabled{};  // 使能状态
};

class RobotMotorSession {
public:
    explicit RobotMotorSession(const RobotInterfaceConfig& config);
    ~RobotMotorSession() noexcept;

    // 禁止拷贝构造和拷贝赋值
    RobotMotorSession(const RobotMotorSession&) = delete;
    RobotMotorSession& operator=(const RobotMotorSession&) = delete;

    // 初始化；RT 启动后失败会保留控制器，调用方须进入 shutdown()。
    bool initialize();
    // 调用前结束命令生产线程；确认先前的 STOP，按通信状态决定是否释放控制器。
    ShutdownResult shutdown(const motor_base::CommandSubmitResult& request);
    bool is_initialized() const noexcept { return initialized_.load(); }
    // 查询底层锁存故障，不消费反馈或重新判断逐轴状态。
    bool check_health(std::string& error) const;


    // 会话层仅提供全轴 STOP/RESTART；单轴操作由底层控制器提供。
    bool restart();
    // 提交全轴 STOP 并同步等待确认；不释放控制器。
    bool stop();
    motor_base::CommandSubmitResult request_stop();

    // 以下连续目标接口的 true 仅表示提交成功，不表示 RT 已应用或电机已执行。
    // 两个接口均原样保留调用方提供的完整时序
    bool submit_position_targets_rad(
        const std::array<double, motor_base::kMotorNum>& target_motor_rad,
        const motor_base::CommandTiming& timing);
    // 提交数组前 config_.num_motors 项；保留调用方提供的完整时序。
    bool submit_impedance_setpoints(
        const std::array<motor_base::ImpedanceSetpoint,
                         motor_base::kMotorNum>& setpoints,
        const motor_base::CommandTiming& timing);


    // policy_cmd 线程独占；true 表示已有缓存，不表示本次有新帧或反馈未过期。
    bool refresh_feedback_for_command(std::string& error);
    // 仅读取缓存；引用在所属缓存下次刷新或生命周期重置前使用完毕。
    const MotorStateSnapshot& get_feedback_for_command() const noexcept;

    // policy/inference 线程独占；刷新与读取语义同 command 接口。
    bool refresh_feedback_for_policy(std::string& error);
    const MotorStateSnapshot& get_feedback_for_policy() const noexcept;

    // 校验当前缓存，成功时输出最老反馈的截止期；不刷新反馈。
    bool validate_feedback_timing_for_command(
        std::int64_t now_ns,
        std::int64_t& feedback_valid_until_ns,
        std::string& error) const;


private:
    friend struct RobotInterfacePolicyTimingTestAccess;
    // 集中装配具体驱动和通信适配器，不连接硬件或启动线程。
    std::unique_ptr<motor_base::MotorControllerBase> create_controller() const;
    void release_controller();

    bool wait_for_discrete_command(const motor_base::CommandSubmitResult& request,
                                   const char* context);
    bool submit_discrete_command(const motor_base::ControlCommand& command,
                                 const char* context);

    MotorConfig config_;
    const robot_base::ThreadRuntimeOptions motor_rt_options_;
    const robot_base::ThreadRuntimeOptions background_thread_options_;
    const std::int64_t max_feedback_age_ns_;

    std::unique_ptr<motor_base::MotorControllerBase> controller_;

    std::atomic<bool> initialized_{false};
    bool rt_started_{false}; // Owner thread; also covers incomplete initialization.


    enum class FeedbackChannel { Policy, Command };
    struct FeedbackCache {
        MotorStateSnapshot snapshot{};
        bool has_feedback{false};
        int oldest_motor_index{-1};
    };
    bool refresh_feedback(FeedbackCache& cache, FeedbackChannel channel,
                          std::string& error);
    // 两侧线程独占精简缓存；无新帧时沿用旧帧，不延长时间戳。
    // 生命周期重置须在两个消费者停止时进行。
    FeedbackCache feedback_for_policy_{};
    FeedbackCache feedback_for_command_{};
};

}  // namespace inference
