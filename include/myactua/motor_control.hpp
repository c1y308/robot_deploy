#pragma once
#include <vector>
#include <memory>
#include <cstdio>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>
#include "myactua/MotorTypes.hpp"
#include "myactua/EthercatAdapter.hpp"
#include "myactua/ThreadSafeQueue.hpp"
#include "myactua/ControlTypes.hpp"

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
        double setpoint;

        MotorState(int index)
        : slave_index(index), target_mode(ControlMode::NONE), step(MotorStep::IDLE), 
          mode_switch_step(ModeSwitchStep::IDLE), tx({}), rx({}), setpoint(0.0) {}
    };

    MYACTUA(std::shared_ptr<EthercatAdapter> adapter,int num_motors);

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
    using StatusCallback = std::function<void(const std::vector<MotorStatusSnapshot>&)>;  // 电机状态快照回调函数
    void set_status_callback(StatusCallback cb);  // 设置电机状态快照回调函数
    bool is_running() const { return running_; }  // 是否正在运行实时线程
    void print_motors_info(void);
private:
    std::shared_ptr<EthercatAdapter> _adapter;
    std::vector<MotorState> _motors;

    std::thread rt_thread_;
    std::atomic<bool> running_{false};  // 实时线程运行标志
    
    ThreadSafeQueue<ControlCommand> cmd_queue_;
    std::mutex status_mutex_;
    std::vector<MotorStatusSnapshot> status_snapshot_;
    
    StatusCallback status_callback_;

    void rt_thread_func();
    void process_commands();
    void update_status_snapshot();

    void process_single_motor(MotorState& motor, double setvalue);

    void handle_mode_switching(MotorState& motor);

    ControlWordCommand get_next_control_word(uint16_t status_word);
};

} // namespace myactua
