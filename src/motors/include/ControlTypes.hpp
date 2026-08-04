#pragma once

#include <vector>
#include <cstdint>
#include <utility>
#include "MotorTypes.hpp"

namespace myactua {

class MYACTUA;

enum class ControlCommandKind {
    DISCRETE,  // STOP/RESTART/SET_MODE 等需要闭环确认的离散命令
    SETPOINT   // 连续目标值命令
};

enum class DiscreteCommandType {
    STOP,     // 停止电机
    RESTART,  // 启动电机
    SET_MODE  // 设置电机模式
};

enum class SetpointCommandType {
    SCALAR_SETPOINTS,  // CSP/CSV/CST 等单标量目标值
    MIT_SETPOINTS      // MIT/PVT 目标值
};

/* MIT/PVT 模式单轴目标值，外部使用角度制(deg) */
struct MitSetpoint {
    double position_deg;         // 0x607A, 单位: 度(deg), 底层自动转为 increments
    double velocity_rad_s;       // 0x60FF, rad/s -> increments/s
    double torque_ff_permille;   // 0x6071, 千分之一额定力矩
    double kp;                   // 0x2000, 写入 kp * 1000
    double kd;                   // 0x2001, 写入 kd * 1000

    MitSetpoint()
        : position_deg(0.0), velocity_rad_s(0.0), torque_ff_permille(0.0),
          kp(0.0), kd(0.0) {}

    MitSetpoint(double pos_deg,
                double vel_rad_s,
                double torque_ff,
                double kp_value,
                double kd_value)
        : position_deg(pos_deg), velocity_rad_s(vel_rad_s),
          torque_ff_permille(torque_ff), kp(kp_value), kd(kd_value) {}
};


/* 控制命令 */
struct ControlCommand {
    static constexpr int kAllSlaves = -1;

    ControlCommandKind kind;             // 命令大类
    DiscreteCommandType discrete_type;   // 离散命令类型
    SetpointCommandType setpoint_type;   // 连续目标值类型
    int slave_index;                     // 电机索引，kAllSlaves 表示全部电机
    std::vector<double> scalar_setpoints;  // 标量目标值，单位由当前控制模式决定
    std::vector<MitSetpoint> mit_setpoints;  // MIT/PVT目标值
    ControlMode mode;                    // 目标电机模式，仅 SET_MODE 使用

    static ControlCommand stop(int slave_index = kAllSlaves) {
        ControlCommand cmd;
        cmd.kind = ControlCommandKind::DISCRETE;
        cmd.discrete_type = DiscreteCommandType::STOP;
        cmd.slave_index = slave_index;
        return cmd;
    }

    static ControlCommand restart(int slave_index = kAllSlaves) {
        ControlCommand cmd;
        cmd.kind = ControlCommandKind::DISCRETE;
        cmd.discrete_type = DiscreteCommandType::RESTART;
        cmd.slave_index = slave_index;
        return cmd;
    }

    static ControlCommand set_mode(ControlMode mode, int slave_index = kAllSlaves) {
        ControlCommand cmd;
        cmd.kind = ControlCommandKind::DISCRETE;
        cmd.discrete_type = DiscreteCommandType::SET_MODE;
        cmd.slave_index = slave_index;
        cmd.mode = mode;
        return cmd;
    }

    static ControlCommand set_scalar_setpoints(std::vector<double> values) {
        ControlCommand cmd;
        cmd.kind = ControlCommandKind::SETPOINT;
        cmd.setpoint_type = SetpointCommandType::SCALAR_SETPOINTS;
        cmd.slave_index = kAllSlaves;
        cmd.scalar_setpoints = std::move(values);
        return cmd;
    }

    static ControlCommand set_scalar_setpoint(int slave_index, double value) {
        ControlCommand cmd;
        cmd.kind = ControlCommandKind::SETPOINT;
        cmd.setpoint_type = SetpointCommandType::SCALAR_SETPOINTS;
        cmd.slave_index = slave_index;
        cmd.scalar_setpoints = {value};
        return cmd;
    }

    static ControlCommand set_mit_setpoints(std::vector<MitSetpoint> values) {
        ControlCommand cmd;
        cmd.kind = ControlCommandKind::SETPOINT;
        cmd.setpoint_type = SetpointCommandType::MIT_SETPOINTS;
        cmd.slave_index = kAllSlaves;
        cmd.mit_setpoints = std::move(values);
        return cmd;
    }

    static ControlCommand set_mit_setpoint(int slave_index, const MitSetpoint& value) {
        ControlCommand cmd;
        cmd.kind = ControlCommandKind::SETPOINT;
        cmd.setpoint_type = SetpointCommandType::MIT_SETPOINTS;
        cmd.slave_index = slave_index;
        cmd.mit_setpoints = {value};
        return cmd;
    }

private:
    friend class MYACTUA;

    ControlCommand()
        : kind(ControlCommandKind::DISCRETE),
          discrete_type(DiscreteCommandType::STOP),
          setpoint_type(SetpointCommandType::SCALAR_SETPOINTS),
          slave_index(kAllSlaves),
          mode(ControlMode::NONE) {}
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
    DiscreteCommandType type;
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

    DiscreteCommand(DiscreteCommandType t = DiscreteCommandType::STOP,
                    ControlMode m = ControlMode::NONE)
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
    ControlMode tx_mode;    // 本周期下发模式
    MotorStep step;         // 控制状态机主状态
    ModeSwitchStep mode_switch_step;  // 模式切换子状态
    bool desired_enabled;   // 期望使能
    uint32_t offline_count; // 离线累计次数
    int32_t tx_target_pos;  // 本周期下发位置目标(raw)
    int32_t tx_target_vel;  // 本周期下发速度目标(raw)
    int16_t tx_target_torque;  // 本周期下发力矩目标(raw)
    int32_t tx_pvt_kp;      // 本周期下发 PVT Kp(raw)
    int32_t tx_pvt_kd;      // 本周期下发 PVT Kd(raw)

    MotorStatusSnapshot() 
        : slave_index(-1), position(0), velocity(0), torque(0), comm_ok(false), status_word(0), error_code(0), 
          op_mode(ControlMode::NONE), target_mode(ControlMode::NONE),
          tx_mode(ControlMode::NONE), step(MotorStep::IDLE),
          mode_switch_step(ModeSwitchStep::IDLE), desired_enabled(false),
          offline_count(0), tx_target_pos(0), tx_target_vel(0),
          tx_target_torque(0), tx_pvt_kp(0), tx_pvt_kd(0) {}
};

} // namespace myactua
