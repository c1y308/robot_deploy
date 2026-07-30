#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

namespace xbox_control {

constexpr const char* kXboxDevicePath = "/dev/input/event7";
constexpr int kAxisMin = 0;
constexpr int kAxisCenter = 128;
constexpr int kAxisMax = 255;
constexpr int kDeadzone = 0;
constexpr double kMaxSpeedMps = 0.5;
constexpr double kYawRate = 0.0;

struct VelocityCommand {
    int raw_abs_x = kAxisCenter;
    int raw_abs_y = kAxisCenter;
    bool has_abs_x = false;
    bool has_abs_y = false;
    double vx = 0.0;
    double vy = 0.0;
    double yaw_rate = kYawRate;
};

class XboxController {
public:
    /* 构造 Xbox 手柄控制器对象，设备在 open_device() 中打开 */
    XboxController();
    /* 析构时关闭已打开的设备文件描述符 */
    ~XboxController();

    XboxController(const XboxController&) = delete;
    XboxController& operator=(const XboxController&) = delete;

    /* 打开手柄输入设备，并初始化当前摇杆轴状态 */
    bool open_device();
    /* 关闭手柄输入设备 */
    void close_device();
    /* 判断手柄输入设备是否已经打开 */
    bool is_open() const;

    /* 启动后台线程读取手柄事件 */
    bool start_polling(std::chrono::milliseconds wait_timeout = std::chrono::milliseconds(20));
    /* 停止后台读取线程 */
    void stop_polling();
    /* 判断后台读取线程是否仍在运行 */
    bool is_polling() const;
    /* 获取后台线程维护的最近一次速度指令 */
    bool latest_command(VelocityCommand& command) const;
    /* 获取最近一次计算出的速度指令 */
    VelocityCommand command() const;
    /* 获取当前使用的手柄设备路径 */
    const std::string& device_path() const { return device_path_; }
    /* 获取最近一次设备操作失败的错误信息 */
    std::string last_error() const;

    /* 将原始摇杆轴值映射为速度值 */
    static double axis_to_speed(int raw_value);
    /* 根据 X/Y 轴原始值生成速度控制指令 */
    static VelocityCommand map_axes(int abs_x, int abs_y);

private:
    /* 读取设备当前轴值，作为初始控制状态 */
    void initialize_axis_values();
    /* 读取当前已经就绪的设备事件 */
    bool read_available_events();
    /* 后台线程循环，等待并消费手柄事件 */
    void polling_loop(std::chrono::milliseconds wait_timeout);
    /* 处理单个绝对轴事件 */
    void process_abs_event(int code, int value);
    /* 写入最近一次错误 */
    void set_error(std::string error);
    /* 根据缓存的轴值刷新速度控制指令 */
    void refresh_command_locked();


    /* 文件描述符 */
    int fd_ = -1;
    /* 设备路径 */
    std::string device_path_ = kXboxDevicePath;
    /* 错误信息 */
    std::string last_error_;

    VelocityCommand command_;

    mutable std::mutex io_mutex_;
    mutable std::mutex state_mutex_;

    std::atomic<bool> polling_active_{false};
    std::atomic<bool> stop_polling_requested_{false};

    std::thread polling_thread_;
};

}  // namespace xbox_control
