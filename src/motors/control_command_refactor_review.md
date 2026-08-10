# ControlCommand 重构复盘

记录日期：2026-06-01

## 1. 修改背景与现象

原来的 `ControlCommand` 使用一个 `CommandType` 枚举配合多个可选字段表达所有控制命令：

```cpp
struct ControlCommand {
    CommandType type;
    int motor_index;
    std::vector<double> values;
    std::vector<ImpedanceSetpoint> impedance_setpoints;
    ControlMode mode;
};
```

这种设计简单直接，但在项目继续扩展到 CSP 标量目标、阻抗式目标、STOP、RESTART、SET_MODE 等多类命令后，逐渐暴露出几个问题：

- 同一个结构体同时承载“连续目标值命令”和“离散状态命令”，语义混在一起。
- 哪些字段有效完全依赖 `type` 的约定，编译器无法阻止非法组合。
- `STOP/RESTART/SET_MODE` 使用 `motor_index < 0` 表示全部电机，而 setpoint 命令的 `motor_index` 语义不统一。
- `process_commands()` 中需要对所有 `CommandType` 分支做人工分发，离散命令状态机里还会出现 setpoint 相关的无意义 case。
- 调用点可读性不够好，例如 `ControlCommand(CommandType::SET_MODE, i, {}, mode)` 需要读构造函数参数顺序才能理解含义。

典型非法但可编译的调用包括：

```cpp
ControlCommand(CommandType::STOP, -1, values, ControlMode::PVT);
ControlCommand(CommandType::SET_MODE, -1, impedance_setpoints);
```

这类代码不会在编译期报错，只能依赖运行时分支忽略无效字段，长期维护风险较高。

## 2. 为什么需要这样修改

这次修改的目标不是单纯换命名，而是把“命令的合法状态”前移到接口层：

- 用类型区分命令大类，降低误用概率。
- 用工厂函数表达意图，让调用点更接近业务语言。
- 让离散命令队列只处理真正需要闭环确认的命令。
- 保持 `send_command(const ControlCommand&)` 不变，减少对控制器外层 API 的冲击。
- 不引入 `std::variant`，控制改动规模，属于“中改”而不是大规模架构替换。

从控制系统角度看，这个改动也更符合真实语义：

- `STOP`、`RESTART`、`SET_MODE` 是离散状态命令，需要进入离散命令队列，等待状态字或模式回读确认。
- `set_scalar_setpoints` 和 `set_impedance_setpoints` 是连续目标值更新，主要写入 `DesiredState`，不应该混入离散状态机。

## 3. 核心设计变化

新增三个枚举：

```cpp
enum class ControlCommandKind {
    DISCRETE,
    SETPOINT
};

enum class DiscreteCommandType {
    STOP,
    RESTART,
    SET_MODE
};

enum class SetpointCommandType {
    SCALAR_SETPOINTS,
    IMPEDANCE_SETPOINTS
};
```

`ControlCommand` 不再暴露旧构造方式，只允许通过静态工厂函数创建：

```cpp
ControlCommand::stop();
ControlCommand::restart();
ControlCommand::set_mode(ControlMode::CSP, i);
ControlCommand::set_scalar_setpoints(target_deg);
ControlCommand::set_scalar_setpoint(i, target);
ControlCommand::set_impedance_setpoints(impedance_setpoints);
ControlCommand::set_impedance_setpoint(i, impedance_setpoint);
```

同时保留 `ControlCommand::kAllMotors = -1`，统一表达“全部电机”的含义。

## 4. 主要实现点

### motor_base/command_types.hpp

- 删除旧的 `CommandType`。
- 删除 `ControlCommand(CommandType, ...)` 系列构造函数。
- 新增 `ControlCommandKind`、`DiscreteCommandType`、`SetpointCommandType`。
- 新增工厂函数，强制调用者使用语义明确的新接口。
- 将 `DiscreteCommand::type` 从旧 `CommandType` 改为 `DiscreteCommandType`。

### motor_control.cpp

`process_commands()` 从原来的单层 `switch (cmd.type)` 改为两段式处理：

```cpp
if (cmd.kind == ControlCommandKind::DISCRETE) {
    enqueue_discrete_command(cmd);
    continue;
}

switch (cmd.setpoint_type) {
    case SetpointCommandType::SCALAR_SETPOINTS:
        ...
    case SetpointCommandType::IMPEDANCE_SETPOINTS:
        ...
}
```

这样以后读代码时可以直接看到：

- 离散命令进入 `enqueue_discrete_command()`。
- 连续目标值命令只更新对应的 desired setpoint。

离散状态机中的 `apply_discrete_command_to_motor()` 和 `is_discrete_command_satisfied()` 也只处理：

- `DiscreteCommandType::STOP`
- `DiscreteCommandType::RESTART`
- `DiscreteCommandType::SET_MODE`

不再出现 setpoint 相关的空分支。

### 调用点迁移

旧写法：

```cpp
controller.send_command(
    ControlCommand(CommandType::SET_MODE, i, {}, ControlMode::CSP));
```

新写法：

```cpp
controller.send_command(
    ControlCommand::set_mode(ControlMode::CSP, i));
```

旧写法：

```cpp
controller.send_command(
    ControlCommand(CommandType::SET_SETPOINTS, -1, target_deg));
```

新写法：

```cpp
controller.send_command(
    ControlCommand::set_scalar_setpoints(target_deg));
```

旧写法：

```cpp
controller.send_command(
    ControlCommand(CommandType::SET_IMPEDANCE_SETPOINTS, -1, sp));
```

新写法：

```cpp
controller.send_command(
    ControlCommand::set_impedance_setpoints(sp));
```

## 5. 修改后的收益

### 类型安全更好

旧接口允许把任意字段组合到一起，新接口通过工厂函数收敛创建路径，调用者不能再直接写 `ControlCommand(CommandType, ...)`。

### 可读性更强

调用点从“看参数猜语义”变成“函数名表达语义”：

- `stop()`
- `restart()`
- `set_mode(...)`
- `set_scalar_setpoints(...)`
- `set_impedance_setpoints(...)`

这对控制代码很重要，因为读代码的人需要快速判断当前命令是否会改变电机状态、是否会下发连续目标值。

### 状态机职责更清晰

离散命令队列只处理需要确认完成的命令，不再承载 setpoint 分支。这样后续要扩展重试、超时、状态监控时，边界更明确。

### 兼容控制器外层调用形式

`MYACTUA::send_command(const ControlCommand&)` 没有改变，因此线程安全队列和实时控制线程的整体结构没有大改。

## 6. 风险与取舍

这次选择的是“强制新接口”，没有保留旧构造函数兼容层。收益是旧错误写法会直接编译失败，风险是所有调用点必须同步迁移。

没有使用 `std::variant`，原因是：

- 当前目标是中等规模重构，不希望一次性引入更大的模板/访问器改造。
- 现有 `ThreadSafeQueue<ControlCommand>` 和 `send_command()` 路径可以复用。
- 工厂函数已经能解决主要的误用问题。

setpoint 的单位语义保持不变：

- `set_scalar_setpoints` 的单位仍由当前 `ControlMode` 决定。
- `set_impedance_setpoints` 使用 `ImpedanceSetpoint` 字段中的单位定义。

这是为了避免在同一次重构中混入“单位系统重构”，控制改动风险。

## 7. 验证方式

已完成验证：

```bash
cmake --build src/motors/build
cmake --build src/inference/build
git diff --check
grep -R "enum class CommandType\|myactua::CommandType\|ControlCommand(myactua::" -n src --exclude-dir=build --exclude-dir=third_party
```

验证结果：

- motors 编译通过。
- inference 编译通过。
- 没有空白格式问题。
- 旧 `CommandType` 和旧构造调用无残留。

未做事项：

- 没有运行 EtherCAT 实机测试。
- 没有改变底层 PDO、状态字处理、实时线程调度策略。

## 8. 面试复盘表达

可以按下面这段组织语言：

> 我在电机控制接口里发现 `ControlCommand` 是一个典型的手写 tagged union：一个 `CommandType` 配多个可选字段。短期看很方便，但随着命令类型变多，它会允许很多非法组合，比如 STOP 命令携带 setpoint、SET_MODE 命令携带 MIT 参数。编译器无法约束这些状态，后续维护时容易把错误藏到运行时分支里。
>
> 所以我把命令拆成两类：离散命令和连续 setpoint 命令。离散命令包括 STOP、RESTART、SET_MODE，需要进入状态机并等待状态确认；setpoint 命令只负责更新 desired target。然后我移除了旧构造函数，改成 `ControlCommand::stop()`、`set_mode()`、`set_scalar_setpoints()`、`set_impedance_setpoints()` 这类静态工厂函数，让调用点直接表达意图。
>
> 这个方案没有大改 `send_command()` 和线程安全队列，因此对实时控制框架影响较小，但显著提升了类型安全和可读性。最后我同步迁移了所有示例和 inference 层调用，并通过 motors、inference 两套 CMake build 和旧接口残留检查验证。

如果被追问“为什么不用 `std::variant`”，可以回答：

> `std::variant` 确实可以进一步增强类型表达力，但这次定位是中等规模重构。项目里已经有稳定的 `ThreadSafeQueue<ControlCommand>` 和 `send_command()` 路径，我优先选择工厂函数加命令分类，既解决主要误用问题，又避免引入过大的改造面。后续如果命令数量继续扩展，再考虑把 payload 改成 variant。

如果被追问“这次改动有没有风险”，可以回答：

> 最大风险是强制新接口会破坏旧调用，所以我没有保留兼容构造函数，而是一次性迁移仓库内所有调用点，并用编译和 grep 做闭环检查。这样能保证旧接口不会继续被使用。

## 9. 后续可继续优化

- 给 `set_scalar_setpoints` 进一步拆出 `SetPositionDeg`、`SetVelocityRpm`、`SetTorqueRaw`，把单位也放进类型或函数名。
- 给命令增加轻量校验函数，例如指定单轴时要求 payload 至少有一个元素。
- 如果命令 payload 继续复杂化，可以考虑 `std::variant` 版本，彻底避免无效字段常驻结构体。
- 为 `process_commands()` 增加单元测试或仿真测试，覆盖批量 setpoint、单轴 setpoint、STOP/RESTART/SET_MODE 入队逻辑。
