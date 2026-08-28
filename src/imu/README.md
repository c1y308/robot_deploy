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
- `imu_base::ReaderType::XSENS_MTI_CAN`：`device` 为 SocketCAN 接口名，例如 `can0`；CAN bitrate 和接口 up/down 由系统外部配置。

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
- 时间戳 (timestamp): us

### A100 AHRS数据
- 角速度 (roll_speed/pitch_speed/heading_speed): rad/s
- 欧拉角 (roll/pitch/heading): rad
- 四元数 (qw/qx/qy/qz)
- 时间戳 (timestamp): us

### Xsens MTi CAN AHRS数据
- `0x005 XCDI_SampleTime`: big-endian uint32，10 kHz tick，保存为 us。
- `0x021 XCDI_Quaternion`: big-endian int16[4]，缩放 `raw / 32767.0`。
- `0x032 XCDI_RateOfTurn`: big-endian int16[3]，缩放 `raw * 2^-9`，单位 rad/s。
- `projected_gravity` 由四元数计算。

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
