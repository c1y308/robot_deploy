#pragma once
#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include "MotorTypes.hpp"
#include "EthercatAdapter.hpp"
#include "ControlTypes.hpp"
#include "motor_state.hpp"
#include "motor_status_channel.hpp"
#include "motor_status_monitor.hpp"
#include "realtime_queue.hpp"
#include "rt_event_dispatcher.hpp"

#define LIMIT(VAL, MIN, MAX) ((VAL)<(MIN)?(MIN):((VAL)>(MAX)?(MAX):(VAL)))

namespace myactua{


class MYACTUA {
public:
    struct Options {
        std::size_t command_queue_capacity = 64;
        std::size_t discrete_queue_capacity_per_motor = 16;
        std::size_t rt_event_queue_capacity = 256;
        std::size_t max_commands_per_cycle = 16;
        int status_publish_period_ms = 1;
    };

    MYACTUA(std::shared_ptr<EthercatAdapter> adapter, int num_motors);
    MYACTUA(std::shared_ptr<EthercatAdapter> adapter, int num_motors, Options options);

    ~MYACTUA();

    CommandSubmitResult send_command(const ControlCommand& cmd);   // 异步发送控制命令

    bool connect(const char* ifname);
    bool wait_all_slaves_ready(int timeout_ms = 30000, int poll_ms = 100, const std::function<bool()>& should_stop = {}) const;
    void start();       // 启动线程
    void shutdown();    // 关闭线程

    std::vector<MotorStatusSnapshot> get_status();  // 获取电机状态快照
    std::vector<double> get_joint_q_rad();          // 关节位置(rad)
    std::vector<double> get_joint_vel_rad_s();      // 关节速度(rad/s)
    std::vector<double> get_joint_tau_raw();        // 关节力矩(raw)

    using StatusCallback = std::function<void(const std::vector<MotorStatusSnapshot>&)>;  // 电机状态快照回调函数
    void set_status_callback(StatusCallback cb);  // 设置电机状态快照回调函数

    using RtEventCallback = std::function<void(const RtEvent&)>;
    void set_rt_event_callback(RtEventCallback cb);

    bool is_running() const { return running_; }  // 是否正在运行线程

    void set_print_info(const std::vector<int>& slave_indices);  // 决定是否启用终端打印线程

    static double raw_pos_to_rad(double raw_pos);
    static double rad_to_deg(double rad);
    static double raw_vel_to_rad_s(double raw_vel);

    
private:
    Options options_;
    std::shared_ptr<EthercatAdapter> _adapter;
    std::vector<MotorState> _motors;

    CommandQueue cmd_queue_;    // 控制命令队列
    std::vector<DiscreteCommandQueue> discrete_cmd_queues_; // 每个电机的离散命令队列
    uint64_t discrete_cmd_tick_{0};  // 离散命令时间戳（按控制周期递增）
    std::vector<uint8_t> comm_ok_rt_;

    std::thread rt_thread_;
    std::atomic<bool> running_{false};  // 实时线程运行标志
    std::atomic<uint64_t> status_overwrite_count_{0};
    MotorStatusChannel status_channel_;
    RtEventDispatcher rt_event_dispatcher_;
    MotorStatusMonitor status_monitor_;

    void rt_thread_func();                  // 实时线程函数

    CommandSubmitResult validate_command(const ControlCommand& cmd) const;
    void process_commands();

    void update();
    void update_status_snapshot_rt();
    void push_rt_event(const RtEvent& event);
    void push_discrete_failure_event_rt(
        int motor_index,
        const DiscreteCommand& cmd,
        int reason);
    void push_discrete_queue_full_event_rt(int motor_index, const ControlCommand& cmd);
    void push_status_overwritten_event_rt();
    static void rt_event_sink_trampoline(void* context, const RtEvent& event);

    /* 处理离散命令相关 */
    void enqueue_discrete_command(const ControlCommand& cmd);
    void service_discrete_commands();
    void apply_discrete_command_to_motor(int motor_index, const DiscreteCommand& cmd);
    bool is_discrete_command_satisfied(const MotorState& motor, const DiscreteCommand& cmd) const;


    void refresh_observed_state(MotorState& motor);

    void process_single_motor(MotorState& motor, double setvalue);

    void handle_mode_switching(MotorState& motor);

    ControlWordCommand get_next_control_word(uint16_t status_word);
};

} // namespace myactua
