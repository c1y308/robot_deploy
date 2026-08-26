# MYACTUA 异步 Mode-Switch 与 `control_ready` 修复记录

## 背景

本次问题来自 MYACTUA setpoint 提交路径中的一个语义与线程安全冲突：

- 旧版 `MYACTUA::validate_command()` 在非 RT 线程读取 `_motors[i].desired.mode`，用于同步判断 setpoint type 是否匹配当前目标模式。
- `_motors` 由 RT 线程拥有并更新，非 RT 路径读取会形成 C++ 数据竞争。
- 将 mode 校验移入 RT 线程后，`send_command()==ACCEPTED` 的语义必须收敛为“命令已提交”，不能再表示“命令一定会被执行”。

同时，`SET_MODE` 是异步离散命令，驱动器真实模式切换需要等待 `0x6061` 回读、状态机进入 RUNNING、Operation Enabled 等条件。应用层调用：

```cpp
send_command(SET_MODE(CSV));
send_command(VELOCITY_SETPOINT);
```

不能假设第二条 setpoint 一定在 CSV 已确认后执行。

## 风险

如果仅在 `RobotMotorSession` 层根据 `comm_ok / faulted / mode / target_mode / enabled` 拼接一个近似 ready 条件，会产生新的语义不一致：

- Session 判断 READY；
- 但 RT 侧由于 `step != RUNNING`、`operation_enabled == false` 或 `mode_switch_step != IDLE` 仍拒绝 setpoint；
- 上层会误以为 motion 已启用，实际目标未进入 tx shadow。

这会让 `RobotMotorSession::restart()`、`apply_*()` 和 policy command worker 对“命令已提交”和“命令已执行”的理解继续混在一起。

## 修复原则

采用方案 4：明确异步 mode-switch contract。

- `MotorControllerBase::send_command()` 的 `ACCEPTED` 仅表示 payload、基础校验和入队成功，不保证最终执行。
- Driver/status 直接暴露 `control_ready`。
- `control_ready` 与 RT setpoint acceptance 使用同一个 driver-side predicate。
- `RobotMotorSession` 不重新推导 driver 内部状态，只消费 `status.control_ready`。
- RT event callback 只报告和锁存，不调用 `send_command()`、`stop()`、`shutdown()` 或任何 recovery API。

目标契约：

```text
Session says READY
    =>
RT would accept the corresponding setpoint
```

## 核心判据

MYACTUA 内部应使用同一 helper 同时服务于：

- `apply_setpoint_command_callback()` 的 setpoint 接收判断；
- `update_status_snapshot()` 的 `MotorStatusSnapshot::control_ready` 填充。

目标模式下的控制 ready 判据：

```cpp
control_ready_for_mode(motor, expected_mode) =
    motor.comm_ok &&
    !motor.observed.fault &&
    motor.step == MyactMotorStep::RUNNING &&
    motor.observed.operation_enabled &&
    motor.mode_switch_step == MyactModeSwitchStep::IDLE &&
    motor.observed.mode == expected_mode &&
    motor.desired.mode == expected_mode;
```

`RobotMotorSession::all_control_ready(expected_mode)` 只判断：

```cpp
status.control_ready &&
status.mode == expected_mode &&
status.target_mode == expected_mode
```

## 行为变化

- `SET_MODE accepted != MODE_READY`。
- `RESTART accepted != motors ready for setpoint`。
- `RobotMotorSession::restart(-1)` 应阻塞等待全部电机 `control_ready` 后才返回 `true`，并且只有此时才能设置 `motion_enabled_=true`。
- 若等待超时，`restart()` 返回 `false`，保持 `motion_enabled_=false`。
- 若 RT 拒绝 setpoint 并发出 `SETPOINT_COMMAND_REJECTED`，Session 只锁存 `runtime_motion_fault_` 并记录首个拒绝信息。
- 真正的 STOP 由现有 non-RT failure path 触发，例如 policy worker 发现 `apply_*()==false` 后进入失败处理并调用 `stop(-1)`。

## Safe Target 约束

为了避免普通 setpoint 必须等 `control_ready` 后才能发送时，驱动进入 RUNNING 的第一帧使用旧目标或默认 0：

- `RESTART` 首次使能前，将 desired targets 重置为当前反馈位置、零速度、零力矩、零 Kp/Kd。
- `SET_MODE` 改变 target mode 时，也重置 setpoint 到当前反馈，避免旧模式目标在切换完成后残留生效。

## 验证点

至少覆盖以下回归：

- `control_ready=false` 时，setpoint 提交返回 `ACCEPTED`，RT 拒绝并上报 `SETPOINT_COMMAND_REJECTED`。
- `status.control_ready && mode==target_mode==expected` 后，对应 setpoint 被 RT 接收并写入 tx。
- `RobotMotorSession::restart(-1)` 在 ready 前不置 `motion_enabled_=true`。
- restart 等待超时后返回 `false`，并保持 motion disabled。
- RT reject event callback 只锁存 session fault，不提交 STOP。
- `apply_*()` 在 runtime motion fault 下返回 `false`，由 policy worker failure path 触发 STOP。
- `motor_response_latency_test` 需要在 restart/control_ready 后再发送第一帧 impedance target。

建议回归命令：

```bash
cmake --build src/motors/build --target motor_realtime_channel_test
./src/motors/build/motor_realtime_channel_test
cmake --build src/inference/build
```
