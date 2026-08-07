# MYACTUA EtherCAT Motor Control

基于 IGH EtherCAT Master 的电机控制模块，提供 MYACT 电机的模式切换、周期控制与状态读取能力。

## 项目结构

```text
src/motors/
├── CMakeLists.txt
├── README.md
├── include/
│   ├── motor_base/
│       ├── CommandTypes.hpp
│       ├── MotorControllerBase.hpp
│       ├── MotorStatusMonitor.hpp
│       ├── StatusChannel.hpp
│       └── RtEventDispatcher.hpp
│   └── driver/
│       └── myact/
│           ├── motor_control.hpp
│           ├── motor_state.hpp
│           ├── motor_units.hpp
│           ├── myact_debug_printers.hpp
│           └── realtime_queue.hpp
├── src/
│   ├── motor_base/
│   │   ├── motor_base.cpp
│   │   ├── motor_status_monitor.cpp
│   │   └── rt_event_dispatcher.cpp
│   ├── drivers/
│   │   └── myact/
│   │       ├── motor_control.cpp
│   │       └── myact_debug_printers.cpp
│   └── protocol/
│       └── ethercat/
│           ├── EthercatAdapter.hpp
│           ├── EthercatAdapterIGH.hpp
│           ├── EthercatAdapterIGH.cpp
│           └── EthercatTypes.hpp
├── examples/
│   ├── simple_test.cpp
│   ├── debug_tool.cpp
│   └── ID_test.cpp
├── tests/
│   └── motor_realtime_channel_test.cpp
└── datasheet/
    └── myact/
```

## 编译方法

```bash
cd src/motors
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

可执行文件与库产物：
- `libMYACTUA_EtherCat.a`
- `simple_test`
- `debug_tool`
- `ID_test`

## 使用方法

示例程序当前在代码中默认使用网卡 `enp8s0`，运行前请先按现场环境修改对应源码或网卡名。

```bash
cd src/motors/build

# 基础控制流程示例（连接、设模式、发送速度）
./simple_test

# 交互式调试工具（键盘输入模式/目标值）
./debug_tool

# 设备识别/连通性测试
./ID_test
```

## 控制模式与命令

### 公共控制模式（`MotorControlMode`）
- `POSITION` 位置控制，位置目标单位为 `rad`
- `VELOCITY` 速度控制，速度目标单位为 `rad/s`
- `TORQUE` 力矩/出力控制，目标值单位由具体控制器解释
- `IMPEDANCE` 阻抗控制，位置/速度使用 `rad`/`rad/s`，`effort_ff` 由具体控制器解释

MYACTUA 驱动内部会把公共模式映射到 CiA402/PDO 模式：
`POSITION -> CSP`，`VELOCITY -> CSV`，`TORQUE -> CST`，`IMPEDANCE -> PVT`。

### 控制命令（`ControlCommand`）
- `ControlCommand::set_position_targets_rad(...)` 设置位置目标
- `ControlCommand::set_velocity_targets_rad_s(...)` 设置速度目标
- `ControlCommand::set_torque_targets(...)` 设置中性力矩/出力目标
- `ControlCommand::set_impedance_targets(...)` 设置阻抗目标
- `ControlCommand::stop(...)` 停止电机
- `ControlCommand::restart(...)` 重新启动
- `ControlCommand::set_mode(...)` 切换模式

连续目标值命令一次必须提供当前控制器电机数量个目标值，不支持只更新部分电机。

## 依赖与环境要求

1. Linux + 实时性较好的调度环境（建议按控制周期需求配置）。
2. 已安装并可用的 EtherCAT 主站库（例如 `ethercat`）。
3. `CMakeLists.txt` 中 `IGH_PATH` 路径有效（当前默认 `/home/cat/ethercat`）。
4. 具备访问 EtherCAT 网卡与主站设备的权限（必要时使用 root 或配置 udev/组权限）。

## 硬件配置（需要用户确认）

1. **网卡名称**
   - 示例默认：`enp8s0`
   - 请按实际 EtherCAT 主站网卡修改。
2. **从站数量**
   - 由示例中的 `MYACTUA controller(adapter, N)` 决定。
   - 当前 `EthercatAdapterIGH` 默认按 12 个从站配置，逻辑索引 `0-11` 对应 EtherCAT 物理位置 `1-6, 8-13`。
3. **PDO/对象字典一致性**
   - `src/protocol/ethercat/EthercatTypes.hpp` 中 PDO 偏移需与从站 ESI/固件一致。
4. **模式切换与使能流程**
   - 依赖状态字与控制字逻辑，建议先使用 `debug_tool` 验证单轴。

## 常见问题

1. **连接失败**
   - 检查网卡名、网线、从站上电、主站服务状态。
2. **可运行但无动作**
   - 检查模式是否切换成功、目标值单位是否匹配、使能流程是否完成。
3. **周期抖动或丢帧**
   - 检查 CPU 占用、线程优先级、系统实时配置与网卡负载。
