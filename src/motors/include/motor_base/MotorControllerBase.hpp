#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "motor_base/CommandQueue.hpp"
#include "motor_base/CommandTypes.hpp"
#include "motor_base/RtEventDispatcher.hpp"
#include "motor_base/StatusChannel.hpp"

namespace motor_base {

/// @brief 电机控制器抽象基类。
///
/// 定义所有电机控制器共有的生命周期、指令下发、状态反馈接口。
/// 应用层仅依赖此接口，无需关心具体电机型号和底层通信实现。
///
/// 具体控制器实现负责：
///   - 通信适配与数据收发
///   - 电机状态机
///   - 物理量单位换算
class MotorControllerBase {
public:
    struct RealtimeOptions {
        std::size_t command_queue_capacity = 64;
        std::size_t discrete_queue_capacity_per_motor = 16;
        std::size_t max_commands_per_cycle = 16;
        long rt_period_ns = 1000000;
        int rt_priority = 80;
        std::size_t rt_event_queue_capacity = 256;
        int status_publish_period_ms = 1;
    };

    /// @brief 电机状态快照回调
    using StatusCallback  = std::function<void(const std::vector<MotorStatusSnapshot>&)>;
    using RtEventCallback = std::function<void(const RtEvent&)>;

    explicit MotorControllerBase(std::size_t motor_count);
    MotorControllerBase(std::size_t motor_count, RealtimeOptions options);
    virtual ~MotorControllerBase();

    MotorControllerBase(const MotorControllerBase&) = delete;
    MotorControllerBase& operator=(const MotorControllerBase&) = delete;

    // ──────────────────── 生命周期 ────────────────────

    /// @brief 连接底层通信接口
    /// @param interface_name 接口名称，由具体控制器解释
    /// @return 连接是否成功
    bool connect(const char* interface_name);


    /// @brief 阻塞等待所有电机进入可操作状态
    /// @param timeout_ms  超时时间 (ms)，0 表示仅检查一次
    /// @param poll_ms     日志打印间隔 (ms)
    /// @param should_stop 外部中断回调，返回 true 时提前退出
    /// @return 是否全部就绪
    virtual bool wait_all_motors_ready(
        int timeout_ms = 30000,
        int poll_ms = 100,
        const std::function<bool()>& should_stop = {}) const = 0;

        
    /// @brief 启动实时控制线程（1 kHz 典型周期）
    void start();


    /// @brief 停止实时控制线程，释放实时资源
    void shutdown();


    /// @brief 实时控制线程是否正在运行
    bool is_running() const;


    // ──────────────────── 指令下发 ────────────────────

    /// @brief 异步发送控制命令（stop / restart / set_mode / setpoints）
    /// @param cmd 控制命令，详见 ControlCommand
    /// @return 命令提交结果（ACCEPTED / QUEUE_FULL / INVALID_*）
    CommandSubmitResult send_command(const ControlCommand& cmd);


    // ──────────────────── 状态反馈（物理量） ────────────────────

    /// @brief 获取全部电机公共状态快照
    std::vector<MotorStatusSnapshot> get_status();

    /// @brief 获取全部电机关节位置，单位 rad
    std::vector<double> get_joint_q_rad();

    /// @brief 获取全部电机关节速度，单位 rad/s
    std::vector<double> get_joint_vel_rad_s();

    /// @brief 获取全部电机关节力矩反馈百分比
    std::vector<double> get_joint_torque_percent();


    // ──────────────────── 回调 ────────────────────

    /// @brief 设置电机状态快照回调（异步，非实时线程）
    void set_status_callback(StatusCallback cb);

    /// @brief 设置实时事件回调（离散命令失败、丢帧等）
    void set_rt_event_callback(RtEventCallback cb);


    // ──────────────────── 终端监控 ────────────────────

    /// @brief 配置终端状态打印
    /// @param motor_index 需要打印的电机索引，空列表关闭，-1 表示全部
    virtual void set_print_info(const std::vector<int>& motor_index) = 0;


    // ──────────────────── 静态工具方法 ────────────────────

    /// @brief 弧度转角度的通用数学换算
    static double rad_to_deg(double rad);

protected:
    using StatusWriteToken = MotorStatusChannel::WriteToken;

    // ============================================================
    // 派生类可使用的基类能力
    // ============================================================

    std::size_t motor_count() const noexcept { return motor_count_; }
    uint64_t    discrete_command_tick_rt() const noexcept { return discrete_cmd_tick_; }

    bool try_begin_status_write_rt(StatusWriteToken& token);
    bool publish_status_rt(const StatusWriteToken& token);
    void push_rt_event(const RtEvent& event);
    void set_rt_event_fallback_printer(RtEventDispatcher::EventPrinter printer);

    // ============================================================
    // REQUIRED OVERRIDES
    // 新电机控制器必须实现
    // ============================================================

    virtual bool connect_impl(const char* interface_name) = 0;
    virtual void realtime_cycle_callback() = 0;
    virtual void apply_setpoint_command_callback(const ControlCommand& cmd) = 0;
    virtual void apply_discrete_command_callback(
        int motor_index,
        const DiscreteCommand& cmd) = 0;
    virtual DiscreteCommandEvaluation evaluate_discrete_command_callback(
        int motor_index,
        const DiscreteCommand& cmd) const = 0;

    // ============================================================
    // OPTIONAL OVERRIDES
    // 派生类按需实现
    // ============================================================

    virtual CommandSubmitResult validate_command(const ControlCommand& cmd) const;
    virtual void discrete_queue_full_callback(
        int motor_index,
        const ControlCommand& cmd);
    virtual void discrete_command_failed_callback(
        int motor_index,
        const DiscreteCommand& cmd,
        DiscreteFailReason reason);
    virtual bool realtime_start_callback();
    virtual void realtime_stop_callback() noexcept;

private:
    //  rt_thread_func()中调用，从命令队列中取出命令进行分发
    void process_queued_commands_rt();

    // 直接在process_queued_commands_rt()中调用，将离散命令入各个电机的命令队列
    void enqueue_discrete_command_rt(const ControlCommand& cmd);
    // rt_thread_func()中调用，处理各个电机的离散命令队列（状态机）
    void service_discrete_commands_rt();

    void rt_thread_func();

    RealtimeOptions rt_options_;
    std::size_t motor_count_;

    // 电机控制命令队列（stop / restart / set_mode / setpoints）
    CommandQueue cmd_queue_;
    // 每个电机的离散命令队列（stop / restart / set_mode）
    std::vector<DiscreteCommandQueue> discrete_cmd_queues_;
    // 离散命令队列的全局时钟，单位 tick，1 tick = 1 ms
    uint64_t discrete_cmd_tick_{0};

    MotorStatusChannel status_channel_;
    RtEventDispatcher  rt_event_dispatcher_;

    std::thread rt_thread_;
    std::atomic<bool> running_{false};
    mutable std::mutex lifecycle_mutex_;
};

} // namespace motor_base
