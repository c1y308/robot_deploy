# RK3588 IMU Reader

RK3588平台惯导模块数据读取程序，通过串口获取WHEELTEC惯导模块的IMU/AHRS/INSGPS数据。

## 项目结构

```
src/imu/a100/
├── CMakeLists.txt          # CMake构建文件
├── README.md               # 说明文档
├── examples/               # 示例程序
│   └── main.cpp            # 主程序入口
├── include/                # 头文件目录
│   ├── types.hpp
│   ├── serial_port.hpp
│   ├── imu_parser.hpp
│   └── imu_reader.hpp
└── src/                    # 源文件目录
    ├── drivers/
    │   ├── serial_port.cpp
    │   └── imu_reader.cpp
    └── protocol/
        └── frame/
            └── imu_parser.cpp
```

## 编译方法

```bash
cd src/imu/a100
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## 使用方法

```bash
# 基本用法
./imu_reader_cpp -d /dev/ttyUSB0 -b 921600

# 显示帮助
./imu_reader_cpp -h

# 只打印IMU数据
./imu_reader_cpp -d /dev/ttyUSB0 -b 921600 -p

# 打印所有数据并显示统计信息
./imu_reader_cpp -d /dev/ttyUSB0 -b 921600 -p -s
```

### 命令行参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `-d, --device` | 串口设备路径 | `/dev/ttyUSB0` |
| `-b, --baud` | 波特率 | `921600` |
| `-p, --print-imu` | 打印IMU数据 | 关闭 |
| `-s, --stats` | 打印统计信息 | 关闭 |
| `-h, --help` | 显示帮助 | - |

## 数据格式

### IMU数据 (TYPE_IMU, 0x01)
- 角速度 (gyroscope_x/y/z): rad/s
- 加速度 (accelerometer_x/y/z): m/s²
- 磁力计 (magnetometer_x/y/z): mG
- 时间戳 (timestamp): us

### AHRS数据 (TYPE_AHRS, 0x02)
- 角速度 (roll_speed/pitch_speed/heading_speed): rad/s
- 欧拉角 (roll/pitch/heading): rad
- 四元数 (qw/qx/qy/qz)
- 时间戳 (timestamp): us

### INSGPS数据 (TYPE_INSGPS, 0x03)
- 位置 (latitude/longitude/altitude): deg, deg, m
- 速度 (north_velocity/east_velocity/ground_velocity): m/s
- 姿态 (azimuth/pitch/roll): deg
- 时间戳 (timestamp): us

## 数据帧协议

```
帧结构: [帧头][类型][长度][数据域][帧尾]
        0xFC   类型   长度   N字节    0xFD
```

| 数据类型 | 帧类型 | 数据长度 | 帧总长度 |
|----------|--------|----------|----------|
| IMU | 0x01 | 0x3C (60) | 64字节 |
| AHRS | 0x02 | 0x38 (56) | 56字节 |
| INSGPS | 0x03 | 0x50 (80) | 80字节 |

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
