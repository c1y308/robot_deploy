#pragma once

#include <vector>
#include <cstdint>
#include "MotorTypes.hpp"

namespace myactua {

enum class CommandType {
    SET_SETPOINTS,  // 设置目标值
    STOP,           // 停止电机
    RESTART,        // 启动电机
    SET_MODE        // 设置电机模式
};


/* 控制命令 */
struct ControlCommand {
    CommandType type;            // 控制命令类型
    int slave_index;             // 电机索引
    std::vector<double> values;  // 目标值 (仅在 SET_SETPOINTS 命令中有效)
    ControlMode mode;            // 电机模式（仅在 SET_MODE 命令中有效）

    ControlCommand() : type(CommandType::STOP), slave_index(-1), mode(ControlMode::NONE) {}

    ControlCommand( CommandType t,
                    int idx = -1,
                    const std::vector<double>& vals = {},
                    ControlMode m = ControlMode::NONE)
                    : type(t), slave_index(idx), values(vals), mode(m) {}
};





/* 离散队列状态机 */
enum class DiscretePhase {
    QUEUED,
    APPLY_PENDING,
    VERIFYING,
    DONE,
    FAILED
};

/* 离散命令重试与超时参数（以 1kHz 控制周期为基准） */
constexpr uint64_t kDiscreteRetryTicks = 300;            // 下次重发命令间隔：300 ms
constexpr uint64_t kDiscreteVerifyIntervalTicks = 20;    // 验证间隔：20 ms
constexpr int kDiscreteSuccessStableTicks = 2;           // 连续满足判据 tick 数
constexpr int kDiscreteMaxRetries = 10;                  // 最多重试次数
constexpr uint64_t kDiscreteTimeoutTicks = 4000;         // 命令超时：4 s

/* 离散命令失败原因 */
enum DiscreteFailReason {
    kDiscreteFailNone = 0,
    kDiscreteFailFault = 1,
    kDiscreteFailTimeout = 2,
    kDiscreteFailMaxRetry = 3,
};

/* 离散队列命令 */
struct DiscreteCommand {
    CommandType type;
    ControlMode mode;

    DiscretePhase phase;
    uint64_t enqueue_tick;
    uint64_t next_retry_tick;
    uint64_t next_verify_tick;
    uint64_t deadline_tick;

    int max_retries;
    int cur_retry;
    int stable_success_cycles;
    int fail_reason;

    DiscreteCommand(CommandType t = CommandType::STOP, ControlMode m = ControlMode::NONE)
        : type(t), mode(m), phase(DiscretePhase::QUEUED),
          enqueue_tick(0), next_retry_tick(0), next_verify_tick(0), deadline_tick(0),
          max_retries(0), cur_retry(0), stable_success_cycles(0), fail_reason(0) {}
};





/* 电机状态快照 */
struct MotorStatusSnapshot {
    int     slave_index;    // ID
    double  position;       // 位置
    double  velocity;       // 速度
    double  torque;         // 扭矩
    bool    comm_ok;        // 通信状态    
    uint16_t status_word;   // 状态字
    uint16_t error_code;    // 错误代码
    ControlMode op_mode;    // 当前模式
    ControlMode target_mode;// 目标模式

    MotorStatusSnapshot() 
        : slave_index(-1), position(0), velocity(0), torque(0), comm_ok(false), status_word(0), error_code(0), 
          op_mode(ControlMode::NONE), target_mode(ControlMode::NONE) {}
};

} // namespace myactua
