#pragma once
#include <vector>
#include <deque>
#include <memory>
#include <cstdio>
#include <thread>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <functional>
#include "MotorTypes.hpp"
#include "EthercatAdapter.hpp"
#include "ThreadSafeQueue.hpp"
#include "ControlTypes.hpp"

#define LIMIT(VAL, MIN, MAX) ((VAL)<(MIN)?(MIN):((VAL)>(MAX)?(MAX):(VAL)))

namespace myactua{


enum class MotorStep {
    IDLE,
    ENABLING,
    RUNNING,
    STOPPED,
    FAULT,
    MODE_SWITCHING
};


class MYACTUA {
public:
    struct MotorState{
        int slave_index;
        ControlMode target_mode;
        MotorStep step;
        ModeSwitchStep mode_switch_step;
        TxPDO tx;
        RxPDO rx;
        bool comm_ok;
        double setpoint;
        bool print_info;

        MotorState(int index)
        : slave_index(index), target_mode(ControlMode::NONE), step(MotorStep::IDLE), 
          mode_switch_step(ModeSwitchStep::IDLE), tx({}), rx({}), comm_ok(false),
          setpoint(0.0), print_info(false) {}
    };

    MYACTUA(std::shared_ptr<EthercatAdapter> adapter, int num_motors);

    ~MYACTUA();

    void update(const std::vector<double>& setvalues);
    void set_mode(ControlMode mode,int slave_index);
    void stop_motor(int slave_index = -1);
    void restart(int slave_index = -1);
    bool connect(const char* ifname);

    void start();  // 启动实时线程
    void shutdown();  // 关闭实时线程
    void send_command(const ControlCommand& cmd);   // 异步发送控制命令

    std::vector<MotorStatusSnapshot> get_status();  // 获取电机状态快照
    std::vector<double> get_joint_q_rad();          // 关节位置(rad)
    std::vector<double> get_joint_vel_rad_s();      // 关节速度(rad/s)
    std::vector<double> get_joint_tau_raw();        // 关节力矩(raw)

    using StatusCallback = std::function<void(const std::vector<MotorStatusSnapshot>&)>;  // 电机状态快照回调函数
    void set_status_callback(StatusCallback cb);  // 设置电机状态快照回调函数
    bool is_running() const { return running_; }  // 是否正在运行实时线程

    /* 只打印存在于 slave_indices 中ID的电机信息 */
    void set_print_info(const std::vector<int>& slave_indices);
    void print_motors_info(void);

    static double raw_pos_to_rad(double raw_pos);
    static double rad_to_deg(double rad);
    static double raw_vel_to_rad_s(double raw_vel);

    
private:
    struct DiscreteCommand {
        CommandType type;
        ControlMode mode;
        uint64_t next_retry_cycle;

        int retry_count;
        int stable_success_cycles;

        DiscreteCommand(CommandType t = CommandType::STOP, ControlMode m = ControlMode::NONE)
            : type(t), mode(m), next_retry_cycle(0), stable_success_cycles(0), retry_count(0) {}
    };

    std::shared_ptr<EthercatAdapter> _adapter;
    std::vector<MotorState> _motors;

    /* 电机的离散命令队列 */
    std::vector<std::deque<DiscreteCommand>> discrete_cmd_queues_;
    /* 离散命令重试周期 */
    uint64_t control_cycle_{0};

    std::thread rt_thread_;
    std::atomic<bool> running_{false};  // 实时线程运行标志
    
    ThreadSafeQueue<ControlCommand> cmd_queue_;
    std::mutex status_mutex_;
    std::vector<MotorStatusSnapshot> status_snapshot_;

    std::vector<int> print_motor_ids_;
    
    StatusCallback status_callback_;

    void rt_thread_func();
    void process_commands();
    void update_status_snapshot();

    /* 处理离散命令相关 */
    void enqueue_discrete_command(const ControlCommand& cmd);
    void service_discrete_commands(const std::vector<bool>& comm_ok);
    void apply_discrete_command_to_motor(int motor_index, const DiscreteCommand& cmd);
    bool is_discrete_command_satisfied(const MotorState& motor, const DiscreteCommand& cmd) const;

    void process_single_motor(MotorState& motor, double setvalue);

    void handle_mode_switching(MotorState& motor);

    ControlWordCommand get_next_control_word(uint16_t status_word);
};

} // namespace myactua
