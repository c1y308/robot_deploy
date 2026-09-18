# RK3588 IMU Reader

RK3588平台惯导模块数据读取程序，当前支持 WHEELTEC A100 串口 IMU 和 Xsens MTi CAN IMU。

## 项目结构

```
src/imu/
├── CMakeLists.txt          # CMake构建文件
├── README.md               # 说明文档
├── examples/               # 示例程序
│   ├── a100_test.cpp       # A100测试程序
│   └── xsens_mti_can_test.cpp
├── include/                # 头文件目录
│   ├── imu_base/
│   │   └── imu_base.hpp
│   ├── driver/
│   │   ├── serial_port.hpp
│   │   ├── socket_can_port.hpp
│   │   ├── a100/
│   │   │   └── a100_reader.hpp
│   │   └── xsens_mti/
│   │       └── xsens_reader.hpp
│   └── protocol/
│       ├── a100/
│       │   ├── types.hpp
│       │   └── imu_parser.hpp
│       └── xsens_mti/
│           └── can_parser.hpp
└── src/                    # 源文件目录
    ├── drivers/
    │   ├── serial_port.cpp
    │   ├── socket_can_port.cpp
    │   ├── a100/
    │   │   └── a100_reader.cpp
    │   └── xsens_mti/
    │       └── xsens_reader.cpp
    └── protocol/
        ├── a100/
        │   └── imu_parser.cpp
        └── xsens_mti/
            └── can_parser.cpp
```

## 编译方法

```bash
cd src/imu
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## IMU类型

- `imu_base::ReaderType::A100_SERIAL`：默认类型，`device` 为串口设备路径，`baudrate` 为串口波特率。
- `imu_base::ReaderType::XSENS_MTI_CAN`：`device` 为 SocketCAN 接口名，例如 `can0`；默认启动时自动配置接口 down、`type can bitrate 250000`、up。该操作需要 root 权限或 `CAP_NET_ADMIN`。

## 使用方法

```bash
# A100 串口 IMU
./a100_test

# Xsens MTi CAN IMU，默认读取 can0
./xsens_mti_can_test
```

## 数据格式

### A100 IMU数据
- 角速度 (gyroscope_x/y/z): rad/s
- 加速度 (accelerometer_x/y/z): m/s²
- 磁力计 (magnetometer_x/y/z): mG
- `receive_timestamp_ns`: Linux 主机接收时间，`CLOCK_MONOTONIC` ns
- `sample_timestamp_ns`: 设备原始采样时间转换为 ns

### A100 AHRS数据
- 角速度 (roll_speed/pitch_speed/heading_speed): rad/s
- 欧拉角 (roll/pitch/heading): rad
- 四元数 (qw/qx/qy/qz)
- `receive_timestamp_ns`: Linux 主机接收时间，`CLOCK_MONOTONIC` ns
- `sample_timestamp_ns`: 设备原始采样时间转换为 ns

### Xsens MTi CAN AHRS数据
- `0x005 XCDI_SampleTime`: big-endian uint32，10 kHz tick，保存为 `sample_timestamp_ns`。
- `0x021 XCDI_Quaternion`: big-endian int16[4]，缩放 `raw / 32767.0`。
- `0x032 XCDI_RateOfTurn`: big-endian int16[3]，缩放 `raw * 2^-9`，单位 rad/s。
- `projected_gravity` 由四元数计算。

Quaternion 和 RateOfTurn 使用最新值配对：自上次发布以来，两路都更新过才发布，连续同类帧覆盖该字段的待组装值。例如 `Q Q Q R` 发布最后一个 Q 与 R。待组装数据与发布快照分离，`get_ahrs_data()` 和同步回调只读取完整快照；半更新、SampleTime 更新和非法四元数不会改变已发布快照。

两路分别保留各自的内核 RX 时间；AHRS 的 `receive_timestamp_ns` 取两者较早值，因此主机年龄代表组成字段中最老字段的年龄。SocketCAN 在绑定前启用 `SO_TIMESTAMPNS_NEW`，通过 `recvmsg()` 取得内核 `CLOCK_REALTIME` 接收时间，再转换到现有 `CLOCK_MONOTONIC` 域，保留 socket 队列中的滞留时间。

转换在打开时建立 offset 基准，每帧通过 `M1 → R → M2` 的 midpoint 采样得到当前 offset。基准仅检测超过 1 ms 的时钟关系变化，实际转换使用当前 offset；采样半跨度达到 1 ms、读钟失败、时间戳缺失／截断／非法，或转换时间不可信时返回错误，不回退到用户态当前时间。映射失效保持错误，必须关闭并重新打开。现有 reader 沿接收错误路径停止，上一有效快照继续受既有年龄守卫约束。转换有有限采样误差，不是设备时钟同步或硬件采样时间。

`sample_timestamp_ns` 只保存发布时最近收到的 SampleTime（原始 uint32 tick × 100000 ns）。无 SampleTime、重复、零值和回绕均不阻止发布，也不清除待组装字段；该元数据不承诺对应 Q 和 Rate 的共同采样时刻。

计算姿态和重力前检查四元数 `norm_sq` 有限且大于 `0.25`。非法四元数计入 `error_frames`，清空两路待组装值和 fresh 状态，需要重新收到合法 Q 和新 Rate 才能发布；保留上一发布快照及其 ready 状态。此检查不归一化四元数，也不要求精确单位范数。

parser 的 `feed()`、同步回调和 `get_ahrs_data()` 应在同一线程执行；跨线程使用者应复制快照并通过已有同步通道传递。机器人策略线程使用现有 `SpscLatestChannel` 接收副本。

## 测试

IMU Debug 构建及全部无硬件测试：

```bash
cmake -S src/imu -B /tmp/robot-deploy-imu-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/robot-deploy-imu-debug -j4
ctest --test-dir /tmp/robot-deploy-imu-debug --output-on-failure
```

`socket_can_timestamp_test` 使用内部时间辅助函数和仅用于测试目标的链接包装，覆盖 ancillary 数据、时钟转换、`EINTR` 重试和错误路径，不修改主机墙钟或公开驱动接口。

真实设备测试需显式指定已启动且有输入的接口，单独执行，不加入普通 CTest：

```bash
/tmp/robot-deploy-imu-debug/socket_can_timestamp_integration_test --interface can0
```

该程序只被动接收，不配置接口、不发送数据、不初始化或使能电机。确认可读后暂停 100 ms，检查首帧年龄至少 95 ms，再连续接收 5 秒并报告输入与 AHRS 频率。持续输入在 drain 后可能产生新鲜快照，因此本测试不要求最终 AHRS 被年龄守卫拒绝。旧 AHRS 的 50 ms 年龄拒绝由 inference 的 `robot_policy_target_timing_test` 使用受控旧测量单独验证。

## A100 数据帧协议

```
帧结构: [帧头][类型][长度][数据域][帧尾]
        0xFC   类型   长度   N字节    0xFD
```

| 数据类型 | 帧类型 | 数据长度 | 帧总长度 |
|----------|--------|----------|----------|
| IMU | 0x40 | 56 | 64字节 |
| AHRS | 0x41 | 48 | 56字节 |

---

## 硬件配置 (需要用户确认)

### [TODO] 请确认以下硬件连接

1. **串口设备路径**
   - RK3588的串口设备路径通常为:
     - `/dev/ttyS0` - 调试串口
     - `/dev/ttyS1` - UART1
     - `/dev/ttyS2` - UART2
     - `/dev/ttyUSB0` - USB转串口
   - 请根据实际硬件连接确认设备路径

2. **波特率设置**
   - WHEELTEC惯导模块默认波特率: **921600**
   - 如果修改过波特率，请使用 `-b` 参数指定

3. **串口引脚连接**
   ```
   惯导模块        RK3588
   --------        -------
   TX      --->    RX (UARTx)
   RX      <---    TX (UARTx)
   GND     --->    GND
   VCC     --->    3.3V/5V (根据模块要求)
   ```

4. **权限设置**
   ```bash
   # 将用户加入dialout组以获取串口访问权限
   sudo usermod -aG dialout $USER
   # 注销后重新登录生效
   ```

5. **RS485模式 (如使用RS485接口)**
   - 需要配置RS485收发控制引脚
   - 代码中已预留RS485控制接口，需要根据实际硬件实现

---

## 扩展开发

### 添加数据回调函数

在 `main.c` 中可以添加自定义的数据处理回调:

```c
void my_imu_callback(const IMUData_Packet_t *imu) {
    // 自定义IMU数据处理
}

void my_ahrs_callback(const AHRSData_Packet_t *ahrs) {
    // 自定义AHRS数据处理
}
```

### ROS集成

可以将此程序作为ROS节点运行，发布IMU消息:

```c
// 发布 sensor_msgs/Imu 消息
// 发布 sensor_msgs/NavSatFix 消息
```

---

## 常见问题

1. **串口打开失败**
   - 检查设备路径是否正确
   - 检查用户权限 (dialout组)
   - 检查设备是否被其他程序占用

2. **无数据输出**
   - 检查波特率是否匹配
   - 检查串口线连接 (TX/RX是否接反)
   - 检查惯导模块是否正常工作

3. **数据解析错误**
   - 检查帧格式是否匹配
   - 检查字节序 (小端模式)

---

## 参考来源

本项目数据解析逻辑参考自:
- WHEELTEC惯导模块STM32例程
- 串口5连接惯导，波特率921600

## License

MIT License
