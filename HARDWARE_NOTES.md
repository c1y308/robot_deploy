# RK3588 IMU Reader - C++ 重构版本

## 项目概述

本项目是 RK3588 IMU Reader 的 C++ 重构版本，采用现代 C++17 标准，使用面向对象设计和 RAII 资源管理。

## 项目结构

```
rk3588_imu_reader/
├── CMakeLists.txt              # CMake 构建文件
├── HARDWARE_NOTES.md           # 硬件接口说明文档
├── include/                    # 头文件目录
│   ├── types.hpp               # 数据类型定义
│   ├── serial_port.hpp         # 串口类定义
│   ├── imu_parser.hpp          # 数据解析类定义
│   └── imu_reader.hpp          # 主应用类定义
└── src/                        # 源文件目录
    ├── main.cpp                # 主程序入口
    ├── serial_port.cpp         # 串口实现
    ├── imu_parser.cpp          # 数据解析实现
    └── imu_reader.cpp          # 主应用实现
```

## 架构设计

### 1. 命名空间 (Namespace)
所有代码都在 `imu` 命名空间下，避免命名冲突。

### 2. 核心类

#### SerialPort 类
- **职责**: 封装串口硬件操作
- **特性**: RAII 资源管理，禁止拷贝
- **硬件相关**: ⚠️ 标注为 `[HARDWARE]` 和 `[HARDWARE ERROR]`

#### IMUParser 类
- **职责**: 解析 IMU/AHRS/INSGPS 数据帧
- **特性**: 支持回调函数，统计信息
- **协议相关**: ⚠️ 标注为 `[PROTOCOL WARNING]` 和 `[PROTOCOL ERROR]`

#### IMUReader 类
- **职责**: 主应用程序控制
- **特性**: 配置管理，信号处理，运行控制

### 3. 数据结构
- `IMUData`: IMU 数据结构
- `AHRSData`: AHRS 数据结构
- `INSGPSData`: INSGPS 数据结构
- `ParserStats`: 解析统计信息
- `Config`: 配置结构

## 编译方法

```bash
cd rk3588_imu_reader
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

# 只打印 IMU 数据
./imu_reader_cpp -d /dev/ttyUSB0 -b 921600 -p

# 打印 IMU 和 AHRS 数据
./imu_reader_cpp -d /dev/ttyUSB0 -b 921600 -p -a

# 打印所有数据并显示统计信息
./imu_reader_cpp -d /dev/ttyUSB0 -b 921600 -p -a -g -s
```

## C++ 重构亮点

### 1. 现代 C++ 特性
- **智能指针**: 使用 `std::unique_ptr` 管理资源
- **RAII**: 自动资源管理，异常安全
- **移动语义**: 禁用不必要的拷贝
- **constexpr**: 编译期常量
- **enum class**: 强类型枚举

### 2. 面向对象设计
- **封装**: 数据和操作封装在类中
- **单一职责**: 每个类有明确的职责
- **接口清晰**: 公共接口简洁明了

### 3. 可扩展性
- **回调机制**: 支持自定义数据处理
- **配置灵活**: 通过 Config 结构配置
- **统计完善**: 详细的运行统计

## 硬件接口说明

### ⚠️ 需要用户确认的硬件配置

#### 1. 串口设备路径
**位置**: [serial_port.cpp:27-31](file:///home/cat/rk3588_imu_reader/src/serial_port.cpp#L27-L31)

RK3588 的串口设备路径通常为:
- `/dev/ttyS0` - 调试串口
- `/dev/ttyS1` - UART1 (默认)
- `/dev/ttyS2` - UART2
- `/dev/ttyUSB0` - USB 转串口

**不确定点**: 
- 实际使用的串口设备路径
- 串口是否已被其他程序占用

#### 2. 波特率设置
**位置**: [serial_port.cpp:17-26](file:///home/cat/rk3588_imu_reader/src/serial_port.cpp#L17-L26)

WHEELTEC 惯导模块默认波特率: **921600**

支持的波特率:
- 9600, 19200, 38400, 57600
- 115200, 230400, 460800, 921600

**不确定点**:
- 惯导模块的实际波特率配置
- 是否支持其他波特率

#### 3. 串口引脚连接
**位置**: [serial_port.cpp:42-86](file:///home/cat/rk3588_imu_reader/src/serial_port.cpp#L42-L86)

硬件连接:
```
惯导模块        RK3588
--------        -------
TX      --->    RX (UARTx)
RX      <---    TX (UARTx)
GND     --->    GND
VCC     --->    3.3V/5V (根据模块要求)
```

**不确定点**:
- 实际的引脚连接方式
- 是否需要额外的电平转换
- VCC 电压要求 (3.3V 或 5V)

#### 4. 串口权限
**位置**: [serial_port.cpp:30-31](file:///home/cat/rk3588_imu_reader/src/serial_port.cpp#L30-L31)

需要用户权限:
```bash
# 将用户加入 dialout 组以获取串口访问权限
sudo usermod -aG dialout $USER
# 注销后重新登录生效
```

**不确定点**:
- 用户是否已加入 dialout 组
- 是否需要 root 权限

### ⚠️ 数据协议相关

#### 1. 帧格式定义
**位置**: [types.hpp:9-22](file:///home/cat/rk3588_imu_reader/include/types.hpp#L9-L22)

帧结构:
```
[帧头][类型][长度][数据域][帧尾]
0xFC   类型   长度   N字节    0xFD
```

**不确定点**:
- 帧格式是否与实际硬件完全匹配
- 是否有其他未知的帧类型

#### 2. 数据解析
**位置**: [imu_parser.cpp:66-147](file:///home/cat/rk3588_imu_reader/src/imu_parser.cpp#L66-L147)

**不确定点**:
- 字节序是否正确 (当前为小端模式)
- 数据域的具体偏移量是否准确
- 是否有其他未解析的数据字段

#### 3. IMU 数据字段
**位置**: [imu_parser.cpp:66-90](file:///home/cat/rk3588_imu_reader/src/imu_parser.cpp#L66-L90)

解析字段:
- 陀螺仪 (gyroscope_x/y/z): rad/s
- 加速度计 (accelerometer_x/y/z): m/s²
- 磁力计 (magnetometer_x/y/z): mG
- 时间戳 (timestamp): us

**不确定点**:
- 数据单位是否正确
- 坐标系定义 (右手/左手坐标系)
- 数据范围和精度

#### 4. AHRS 数据字段
**位置**: [imu_parser.cpp:92-118](file:///home/cat/rk3588_imu_reader/src/imu_parser.cpp#L92-L118)

解析字段:
- 角速度 (roll_speed/pitch_speed/heading_speed): rad/s
- 欧拉角 (roll/pitch/heading): rad
- 四元数 (qw/qx/qy/qz)
- 时间戳 (timestamp): us

**不确定点**:
- 欧拉角的旋转顺序 (ZYX, ZXY 等)
- 四元数的顺序 (wxyz 或 xyzw)
- 角度范围 (是否归一化到 ±π)

#### 5. INSGPS 数据字段
**位置**: [imu_parser.cpp:120-147](file:///home/cat/rk3588_imu_reader/src/imu_parser.cpp#L120-L147)

解析字段:
- 位置 (latitude/longitude/altitude): deg, deg, m
- 速度 (north_velocity/east_velocity/ground_velocity): m/s
- 姿态 (azimuth/pitch/roll): deg
- 时间戳 (timestamp): us

**不确定点**:
- 经纬度的坐标系 (WGS84?)
- 高度的参考基准 (海拔/椭球高)
- 速度的坐标系定义

## 错误处理标注

代码中已标注的错误类型:

### 硬件错误 `[HARDWARE ERROR]`
- 串口打开失败
- 串口配置失败
- 读写操作失败

### 协议错误 `[PROTOCOL ERROR]`
- 帧尾标记不匹配
- 数据校验失败

### 协议警告 `[PROTOCOL WARNING]`
- 未知的帧类型

## 扩展开发

### 添加自定义数据处理

```cpp
#include "imu_reader.hpp"

int main() {
    imu::IMUReader reader;
    
    // 设置自定义回调
    reader.setIMUCallback([](const imu::IMUData& data) {
        // 自定义 IMU 数据处理
        std::cout << "Custom IMU processing: " << data.timestamp << std::endl;
    });
    
    reader.setAHRSCallback([](const imu::AHRSData& data) {
        // 自定义 AHRS 数据处理
    });
    
    reader.setINSGPSCallback([](const imu::INSGPSData& data) {
        // 自定义 INSGPS 数据处理
    });
    
    // ... 初始化和运行
}
```

### ROS 集成示例

```cpp
// 可以在回调函数中发布 ROS 消息
reader.setIMUCallback([&imu_pub](const imu::IMUData& data) {
    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();
    msg.angular_velocity.x = data.gyroscope_x;
    msg.angular_velocity.y = data.gyroscope_y;
    msg.angular_velocity.z = data.gyroscope_z;
    msg.linear_acceleration.x = data.accelerometer_x;
    msg.linear_acceleration.y = data.accelerometer_y;
    msg.linear_acceleration.z = data.accelerometer_z;
    imu_pub.publish(msg);
});
```

## 测试建议

### 1. 串口测试
```bash
# 检查串口设备是否存在
ls -l /dev/ttyS*

# 检查串口权限
groups $USER

# 使用 minicom 测试串口
minicom -D /dev/ttyS1
```

### 2. 数据验证
- 检查帧头和帧尾是否正确
- 验证数据字段的合理性
- 比对原始数据和解析结果

### 3. 性能测试
- 测试数据吞吐量
- 检查 CPU 和内存使用
- 验证长时间运行稳定性

## 常见问题

### 1. 串口打开失败
- 检查设备路径是否正确
- 检查用户权限 (dialout 组)
- 检查设备是否被其他程序占用

### 2. 无数据输出
- 检查波特率是否匹配
- 检查串口线连接 (TX/RX 是否接反)
- 检查惯导模块是否正常工作

### 3. 数据解析错误
- 检查帧格式是否匹配
- 检查字节序 (小端模式)
- 使用十六进制工具查看原始数据

## 参考来源

本项目数据解析逻辑参考自:
- WHEELTEC 惯导模块 STM32 例程
- 串口 5 连接惯导，波特率 921600

## License

MIT License
