# MYACTUA EtherCAT Motor Control

基于 IGH EtherCAT Master 的电机控制模块，提供 MYACT 电机的模式切换、周期控制与状态读取能力。

## 项目结构

```text
src/motors/
├── CMakeLists.txt
├── README.md
├── include/
│   ├── ControlTypes.hpp
│   └── MotorTypes.hpp
├── src/
│   ├── drivers/
│   │   └── myact/
│   │       ├── motor_control.hpp
│   │       ├── motor_control.cpp
│   │       └── ThreadSafeQueue.hpp
│   └── protocol/
│       └── ethercat/
│           ├── EthercatAdapter.hpp
│           ├── EthercatAdapterIGH.hpp
│           └── EthercatAdapterIGH.cpp
├── examples/
│   ├── simple_test.cpp
│   ├── debug_tool.cpp
│   └── ID_test.cpp
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

### 控制模式（`ControlMode`）
- `CSP (0x08)` 周期同步位置
- `CSV (0x09)` 周期同步速度
- `CST (0x0A)` 周期同步扭矩

### 控制命令（`CommandType`）
- `SET_SETPOINTS` 设置目标值
- `STOP` 停止电机
- `RESTART` 重新启动
- `SET_MODE` 切换模式

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
   - `include/MotorTypes.hpp` 中 PDO 偏移需与从站 ESI/固件一致。
4. **模式切换与使能流程**
   - 依赖状态字与控制字逻辑，建议先使用 `debug_tool` 验证单轴。

## 常见问题

1. **连接失败**
   - 检查网卡名、网线、从站上电、主站服务状态。
2. **可运行但无动作**
   - 检查模式是否切换成功、目标值单位是否匹配、使能流程是否完成。
3. **周期抖动或丢帧**
   - 检查 CPU 占用、线程优先级、系统实时配置与网卡负载。
