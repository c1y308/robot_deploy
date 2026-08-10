#pragma once

#include <cstdint>
#include <cstddef>
#include <utility>
#include <vector>

namespace motor_base {

constexpr std::size_t kMaxMotorCommandSetpoints = 12;


/* 单轴阻抗式目标值，位置/速度使用公共物理单位。effort_ff 由具体控制器解释。 */
struct ImpedanceSetpoint {
    double position_rad;
    double velocity_rad_s;
    double effort_ff;
    double kp;
    double kd;

    ImpedanceSetpoint()
        : position_rad(0.0), velocity_rad_s(0.0), effort_ff(0.0), kp(0.0),
          kd(0.0) {}

    ImpedanceSetpoint(double pos_rad,
                      double vel_rad_s,
                      double effort_ff_value,
                      double kp_value,
                      double kd_value)
        : position_rad(pos_rad), velocity_rad_s(vel_rad_s),
          effort_ff(effort_ff_value), kp(kp_value), kd(kd_value) {}
};


enum class ControlCommandKind {
    DISCRETE,  // STOP/RESTART/SET_MODE 等需要闭环确认的离散命令
    SETPOINT   // 连续目标值命令
};

enum class SetpointCommandType {
    POSITION_TARGETS,
    VELOCITY_TARGETS,
    TORQUE_TARGETS,
    IMPEDANCE_TARGETS
};

enum class DiscreteCommandType {
    STOP,     // 停止电机
    RESTART,  // 启动电机
    SET_MODE  // 设置电机模式
};

enum class CommandSubmitResult {
    ACCEPTED,
    QUEUE_FULL,
    INVALID_COMMAND,
    INVALID_PAYLOAD
};

/* 跨电机类型的公共控制模式语义。 */
enum class MotorControlMode : int8_t {
    NONE = 0,
    POSITION,
    VELOCITY,
    TORQUE,
    IMPEDANCE
};

/* 控制命令 */
struct ControlCommand {
    static constexpr int kAllMotors = -1;

    ControlCommandKind  kind;            // 命令大类
    DiscreteCommandType discrete_type;   // 离散命令类型
    SetpointCommandType setpoint_type;   // 连续目标值类型

    int motor_index;                     // 电机索引，kAllMotors 表示全部电机

    std::vector<double> setpoints;
    std::vector<ImpedanceSetpoint> impedance_setpoints;

    MotorControlMode mode; // 目标电机模式，仅 SET_MODE 使用

    bool payload_valid;

    ControlCommand()
        : kind(ControlCommandKind::DISCRETE),
          discrete_type(DiscreteCommandType::STOP),
          setpoint_type(SetpointCommandType::POSITION_TARGETS),
          motor_index(kAllMotors),
          setpoints(),
          impedance_setpoints(),
          mode(MotorControlMode::NONE),
          payload_valid(true) {}

    static ControlCommand stop(int motor_index = kAllMotors) {
        ControlCommand cmd;
        cmd.kind = ControlCommandKind::DISCRETE;
        cmd.discrete_type = DiscreteCommandType::STOP;
        cmd.motor_index = motor_index;
        return cmd;
    }

    static ControlCommand restart(int motor_index = kAllMotors) {
        ControlCommand cmd;
        cmd.kind = ControlCommandKind::DISCRETE;
        cmd.discrete_type = DiscreteCommandType::RESTART;
        cmd.motor_index = motor_index;
        return cmd;
    }

    static ControlCommand set_mode(MotorControlMode mode, int motor_index = kAllMotors) {
        ControlCommand cmd;
        cmd.kind = ControlCommandKind::DISCRETE;
        cmd.discrete_type = DiscreteCommandType::SET_MODE;
        cmd.motor_index = motor_index;
        cmd.mode = mode;
        return cmd;
    }

    static ControlCommand set_position_targets_rad(std::vector<double> values) {
        ControlCommand cmd;
        cmd.kind = ControlCommandKind::SETPOINT;
        cmd.setpoint_type = SetpointCommandType::POSITION_TARGETS;
        cmd.motor_index = kAllMotors;
        if (values.size() > kMaxMotorCommandSetpoints) {
            cmd.payload_valid = false;
            return cmd;
        }
        cmd.setpoints = std::move(values);
        return cmd;
    }

    static ControlCommand set_velocity_targets_rad_s(std::vector<double> values) {
        ControlCommand cmd;
        cmd.kind = ControlCommandKind::SETPOINT;
        cmd.setpoint_type = SetpointCommandType::VELOCITY_TARGETS;
        cmd.motor_index = kAllMotors;
        if (values.size() > kMaxMotorCommandSetpoints) {
            cmd.payload_valid = false;
            return cmd;
        }
        cmd.setpoints = std::move(values);
        return cmd;
    }

    static ControlCommand set_torque_targets(std::vector<double> torque) {
        ControlCommand cmd;
        cmd.kind = ControlCommandKind::SETPOINT;
        cmd.setpoint_type = SetpointCommandType::TORQUE_TARGETS;
        cmd.motor_index = kAllMotors;
        if (torque.size() > kMaxMotorCommandSetpoints) {
            cmd.payload_valid = false;
            return cmd;
        }
        cmd.setpoints = std::move(torque);
        return cmd;
    }

    static ControlCommand set_impedance_targets(std::vector<ImpedanceSetpoint> values) {
        ControlCommand cmd;
        cmd.kind = ControlCommandKind::SETPOINT;
        cmd.setpoint_type = SetpointCommandType::IMPEDANCE_TARGETS;
        cmd.motor_index = kAllMotors;
        if (values.size() > kMaxMotorCommandSetpoints) {
            cmd.payload_valid = false;
            return cmd;
        }
        cmd.impedance_setpoints = std::move(values);
        return cmd;
    }
};

/* 离散队列状态机 */
enum class DiscretePhase {
    QUEUED,         // 入队
    APPLY_PENDING,  // 已发送
    VERIFYING,      // 等待验证
    DONE,           // 验证成功
    FAILED          // 验证失败
};

enum class DiscreteCommandEvaluation {
    PENDING,
    SATISFIED,
    FAILED
};

/* 离散命令失败原因 */
enum class DiscreteFailReason : int {
    NONE = 0,
    FAULT = 1,
    TIMEOUT = 2,
    MAX_RETRY = 3
};


/* 离散队列命令 */
struct DiscreteCommand {
    DiscreteCommandType type;  // 离散命令类型
    MotorControlMode    mode;  // 目标模式（切换模式命令使用）

    DiscretePhase phase;        // 当前状态机阶段

    uint64_t enqueue_tick;      // 入队 tick
    uint64_t next_retry_tick;   // 下次重发 tick
    uint64_t next_verify_tick;  // 下次验证 tick
    uint64_t deadline_tick;     // 超时 tick

    int max_retries;    // 最大重试次数
    int cur_retry;      // 当前重试次数

    int stable_success_cycles;
    DiscreteFailReason fail_reason;    // 失败原因

    DiscreteCommand(DiscreteCommandType t = DiscreteCommandType::STOP,
                    MotorControlMode    m = MotorControlMode::NONE)

        : type(t), mode(m), phase(DiscretePhase::QUEUED),
          enqueue_tick(0), next_retry_tick(0), next_verify_tick(0), deadline_tick(0),
          max_retries(0), cur_retry(0), stable_success_cycles(0),
          fail_reason(DiscreteFailReason::NONE) {}
};


/* 离散命令重试与超时参数（以 1kHz 控制周期为基准） */
constexpr uint64_t kDiscreteRetryTicks = 300;            // 下次重发命令间隔：300 ms
constexpr uint64_t kDiscreteVerifyIntervalTicks = 20;    // 验证间隔：20 ms
constexpr int kDiscreteSuccessStableTicks = 2;           // 连续满足判据 tick 数
constexpr int kDiscreteMaxRetries = 10;                  // 最多重试次数
constexpr uint64_t kDiscreteTimeoutTicks = 4000;         // 命令超时：4 s

} // namespace motor_base
