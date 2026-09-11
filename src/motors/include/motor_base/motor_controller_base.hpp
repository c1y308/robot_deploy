#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "spsc_latest_channel/spsc_latest_channel.hpp"
#include "motor_base/discrete_command_channel.hpp"
#include "motor_base/command_types.hpp"
#include "motor_base/rt_event_dispatcher.hpp"
#include "motor_base/status_channel.hpp"
#include "tool/thread_runtime.hpp"

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
        std::size_t command_queue_capacity            = 64;
        std::size_t discrete_queue_capacity_per_motor = 16;
        std::size_t max_commands_per_cycle            = 16;

        long rt_period_ns = 1000000;
        int rt_priority   = 80;
        robot_base::ThreadRuntimeOptions rt_thread_options;
        robot_base::ThreadRuntimeOptions background_thread_options;

        // Used to stamp legacy debug setpoints at the controller boundary.
        std::int64_t setpoint_timeout_ns = 10'000'000;

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
    /// @param interface_name 接口名称；为空时由具体控制器选择默认网卡
    /// @return 连接是否成功
    bool connect(const char* interface_name = nullptr);


    /// @brief 阻塞等待所有电机进入可操作状态
    /// @param timeout_ms  超时时间 (ms)，0 表示仅检查一次
    /// @param poll_ms     日志打印间隔 (ms)
    /// @return 是否全部就绪
    virtual bool wait_all_motors_ready(
        int timeout_ms = 30000,
        int poll_ms = 100) const = 0;


    /// @brief 启动实时控制线程（1 kHz 典型周期）
    /// @return RT 调度前置条件是否满足；rt_priority<=0 表示显式非 RT 模式
    bool start();


    /// @brief 停止实时控制线程，释放实时资源
    void shutdown();


    /// @brief 实时控制线程是否正在运行
    bool is_running() const;
    bool terminal_fault_latched() const noexcept
    {
        return terminal_fault_latched_.load(std::memory_order_acquire);
    }
    bool is_realtime_scheduling_ready() const noexcept
    {
        return rt_scheduling_ready_.load(std::memory_order_acquire);
    }


    // ──────────────────── 指令下发 ────────────────────

    /// @brief 异步发送离散控制命令。
    /// @return 离散命令提交成功时携带可查询的 command_id。
    /// STOP 绕过普通队列；同目标未完成的 STOP 复用 ID，确认前禁止重启。
    CommandSubmitResult send_discrete_command(const ControlCommand& cmd);

    /// @brief 查询离散命令的执行结果。
    /// STOP 每个目标保留最新一次请求；未完成请求不会被普通命令覆盖。
    DiscreteCommandResult get_discrete_command_result(CommandId id) const;

    /// @brief policy_command_worker 专用的 latest-value setpoint 提交入口。
    ///        仅允许单 producer 调用，底层为 SPSC 通道；策略流启动前
    ///        的 setup 命令可使用序号 0。
    CommandSubmitResult send_policy_setpoint(const ControlCommand& cmd);

    /// @brief 测试/手动调试专用的 latest-value setpoint 提交入口。
    ///        仅允许单 producer 调用，底层为 SPSC 通道。
    CommandSubmitResult send_debug_setpoint(const ControlCommand& cmd);

    /// @brief 选择 RT 线程当前消费的 SETPOINT 来源。仅在 start() 前生效。
    void set_active_setpoint_source(SetpointSource source);



    // ──────────────────── 状态反馈（物理量） ────────────────────

    /// @brief 获取全部电机公共状态快照
    std::vector<MotorStatusSnapshot> get_status();

    /// @brief 读取 command worker 专属的 RT feedback latest-value 快照；
    ///        无新快照时返回 false。仅限 policy_command_worker 线程消费（SPSC 单消费者）。
    bool try_consume_latest_status_command(
        std::array<MotorStatusSnapshot, kMaxMotorCommandSetpoints>& feedback);

    /// @brief 读取 policy/inference 线程专属的 RT feedback latest-value 快照；
    ///        无新快照时返回 false。仅限 policy/inference 线程消费（SPSC 单消费者）。
    bool try_consume_latest_status_policy(
        std::array<MotorStatusSnapshot, kMaxMotorCommandSetpoints>& feedback);

    /// @brief 获取全部电机位置，单位 rad（按电机顺序）
    std::vector<double> get_positions_rad();

    // ──────────────────── 回调 ────────────────────

    /// @brief 设置电机状态快照回调（异步，非实时线程）
    void set_status_callback(StatusCallback cb);

    /// @brief 设置实时事件回调（离散命令失败、丢帧等）
    void set_event_callback(RtEventCallback cb);


    // ──────────────────── 终端监控 ────────────────────

    /// @brief 配置终端状态打印
    /// @param motor_index 需要打印的电机索引，空列表关闭，-1 表示全部
    virtual void set_print_info(const std::vector<int>& motor_index) = 0;


protected:
    using StatusWriteToken = LatestStatusChannel<MotorStatusSnapshot>::WriteToken;

    // ============================================================
    // 派生类可使用的基类能力
    // ============================================================

    std::size_t motor_count() const noexcept { return motor_count_; }
    uint64_t    discrete_command_tick() const noexcept { return discrete_cmd_tick_; }

    bool write_status(StatusWriteToken& token);
    void publish_status(const StatusWriteToken& token);
    void publish_feedback(
        const std::array<MotorStatusSnapshot, kMaxMotorCommandSetpoints>& feedback);

    void push_event(const RtEvent& event);
    void set_event_fallback_printer(RtEventDispatcher::EventPrinter printer);
    bool latch_terminal_fault() noexcept;

    // ============================================================
    // REQUIRED OVERRIDES
    // 新电机控制器必须实现
    // ============================================================

    virtual bool connect_impl(const char* interface_name) = 0;
    virtual void realtime_cycle_callback() = 0;

    // 派生类实现具体电机的命令应用
    virtual void apply_setpoint_command_impl(const ControlCommand& cmd) = 0;
    virtual void apply_discrete_command_impl(
        int motor_index,
        const DiscreteCommand& cmd) = 0;

    //离散命令状态评估
    virtual DiscreteCommandEvaluation evaluate_discrete_command_impl(
        int motor_index,
        const DiscreteCommand& cmd) const = 0;

    // ============================================================
    // OPTIONAL OVERRIDES
    // 派生类按需实现
    // ============================================================

    virtual CommandSubmitStatus validate_command(const ControlCommand& cmd) const;

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
    // One mailbox per single-axis target plus one all-axis target. Pending
    // requests are coalesced; ordinary command history cannot evict them.
    struct StopRequest {
        std::atomic<CommandId> requested{0};
        std::atomic<CommandId> confirmed{0};
    };

    static_assert(std::atomic<CommandId>::is_always_lock_free,
                  "STOP mailboxes require lock-free command IDs");

    struct StopAxisState {
        CommandId applied{0};           // 已应用的 STOP 命令 ID
        CommandId released{0};          // 这个轴上的 STOP 命令 ID是否被之后的 RESTART解除
        CommandId confirmed{0};         // 最近一次有效确认的 STOP 命令 ID

        uint64_t next_verify_tick{0};   // 下一次检查的时间
        int stable_success_cycles{0};   // 连续成功的周期数
    };

    CommandSubmitResult submit_stop(int motor_index);
    CommandId latest_stop_id(std::size_t motor_index) const;
    bool stop_pending(std::size_t motor_index) const;
    void service_stop_requests(bool verify);

    //  thread_func()中调用，从命令队列中取出离散命令进行分发
    void process_queued_commands();
    void process_latest_setpoint_commands();
    void apply_terminal_fault_stop();
    void latch_setpoint_timeout_fault(int reason, std::uint64_t policy_seq);
    bool validate_setpoint_timing(const ControlCommand& cmd,
                                  std::int64_t now_ns,
                                  int& reason) const noexcept;

    // 直接在process_queued_commands()中调用，将离散命令入各个电机的命令队列
    void enqueue_discrete_command(const ControlCommand& cmd, CommandId command_id);
    // thread_func()中调用，处理各个电机的离散命令队列（状态机）
    void service_discrete_commands();

    void thread_func();

    RealtimeOptions rt_options_;
    std::size_t     motor_count_;

    // 普通离散命令队列（restart / set_mode）；STOP 使用独立 mailbox。
    DiscreteCommandSubmissionQueue cmd_queue_;
    // 下一个离散命令的 ID
    std::atomic<CommandId> next_discrete_command_id_{1};
    // 保护离散命令提交的互斥锁，避免多线程同时提交离散命令导致命令 ID 冲突
    std::mutex discrete_command_submission_mutex_;

    // 离散命令执行结果追踪器
    DiscreteCommandResultTracker discrete_command_results_;
    
    // STOP 请求的电机序号
    std::array<StopRequest, kMaxMotors + 1> stop_requests_{};
    // STOP 请求的电机状态机
    std::array<StopAxisState, kMaxMotors> stop_axes_{}; // RT thread only


    // 每个电机的普通离散命令队列（restart / set_mode）
    std::vector<DiscreteCommandQueue> discrete_cmd_queues_;

    // 常规反馈通道（有锁），支持多读者读写缓存
    LatestStatusChannel<MotorStatusSnapshot> status_channel_;

    // RT feedback 双通道 fan-out（同一帧数据，两条独立 SPSC 边）：
    // command_feedback_channel_，电机驱动层生产，仅 policy_command_worker 线程脚踝力矩解算消费；
    robot_base::SpscLatestChannel<
        std::array<MotorStatusSnapshot, kMaxMotorCommandSetpoints>> command_feedback_channel_;
    // policy_feedback_channel_，电机驱动层生产，仅 policy 线程构建观测帧消费；
    robot_base::SpscLatestChannel<
        std::array<MotorStatusSnapshot, kMaxMotorCommandSetpoints>> policy_feedback_channel_;
    
    // 调试 SETPOINT 专通道（仅测试/手动调试单 producer，电机驱动层线程消费）
    robot_base::SpscLatestChannel<ControlCommand> setpoint_channel_debug_;

    // policy SETPOINT 专通道（仅 policy_command_worker 生产，电机驱动层线程消费）
    robot_base::SpscLatestChannel<ControlCommand> setpoint_channel_policy_;
    SetpointSource active_setpoint_source_{SetpointSource::POLICY};

    // 离散命令队列的全局时钟，单位 tick，1 tick = 1 ms
    uint64_t discrete_cmd_tick_{0};

    RtEventDispatcher  rt_event_dispatcher_;

    std::thread rt_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> rt_scheduling_ready_{false};
    std::atomic<bool> terminal_fault_latched_{false};
    // Startup/setup setpoints may use sequence zero until the first policy
    // frame has been accepted; policy traffic thereafter must be sequenced.
    std::atomic<bool> policy_sequence_started_{false};
    bool has_active_setpoint_{false};
    ControlCommand active_setpoint_{};
    std::uint64_t last_policy_seq_{0};
    std::int64_t last_produced_at_ns_{0};
    mutable std::mutex lifecycle_mutex_;
};

} // namespace motor_base
