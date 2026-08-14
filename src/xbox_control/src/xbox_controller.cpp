#include "xbox_controller.hpp"

#include <linux/input.h>
#include <poll.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <utility>
#include <unistd.h>

namespace xbox_control {
namespace {

/* 组合系统调用前缀和 errno 文本，生成可读错误信息 */
std::string errno_message(const std::string& prefix)
{
    std::ostringstream stream;
    stream << prefix << ": " << std::strerror(errno);
    return stream.str();
}

/* 将超过中心正向死区的原始轴值归一化到 0.0 到 1.0 */
double positive_axis_scale(int raw_value)
{
    const double usable_range =
        static_cast<double>(kAxisMax - kAxisCenter - kDeadzone);
    return static_cast<double>(raw_value - kAxisCenter - kDeadzone) /
           usable_range;
}

/* 将超过中心负向死区的原始轴值归一化到 -1.0 到 0.0 */
double negative_axis_scale(int raw_value)
{
    const double usable_range =
        static_cast<double>(kAxisCenter - kAxisMin - kDeadzone);
    return static_cast<double>(raw_value - kAxisCenter + kDeadzone) /
           usable_range;
}

}  // namespace

/* 构造控制器对象，实际设备打开由 open_device() 完成 */
XboxController::XboxController() = default;

/* 打开手柄事件设备，并读取当前轴值初始化速度指令 */
bool XboxController::open_device()
{
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    if (fd_ >= 0) {
        return true;
    }

    fd_ = ::open(device_path_.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
        set_error(errno_message("open " + device_path_ + " failed"));
        return false;
    }

    initialize_axis_values();
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        last_error_.clear();
        refresh_command_locked();
    }
    return true;
}


/* 启动后台线程，将设备事件读取从控制主循环中移出。 */
bool XboxController::start_polling(std::chrono::milliseconds wait_timeout)
{
    if (is_polling()) {
        return true;
    }
    /* 如果后台线程已经存在，退出后台线程 */
    if (polling_thread_.joinable()) {
        polling_thread_.join();
    }

    if (!is_open()) {
        set_error("controller device is not open");
        return false;
    }

    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        last_error_.clear();
    }

    /* 关闭退出信号，启动后台线程活跃信号 */
    stop_polling_requested_.store(false);
    polling_active_.store(true);

    /* 创建后台线程 */
    try {
        polling_thread_ = std::thread(&XboxController::polling_loop, this, wait_timeout);
    } catch (...) {
        polling_active_.store(false);
        stop_polling_requested_.store(true);
        set_error("start controller polling thread failed");
        return false;
    }

    return true;
}

/* 后台线程等待设备可读，再一次性消费当前积压的事件。 */
void XboxController::polling_loop(std::chrono::milliseconds wait_timeout)
{
    const auto requested_timeout_ms = wait_timeout.count();
    const int timeout_ms = static_cast<int>(std::clamp<long long>(
        requested_timeout_ms, 1, std::numeric_limits<int>::max()));

    while (!stop_polling_requested_.load()) {
        int fd = -1;
        {
            std::lock_guard<std::mutex> io_lock(io_mutex_);
            fd = fd_;
        }

        if (fd < 0) {
            set_error("controller device is not open");
            break;
        }

        pollfd poll_fd = {};
        poll_fd.fd = fd;
        poll_fd.events = POLLIN;

        const int ready = ::poll(&poll_fd, 1, timeout_ms);
        if (stop_polling_requested_.load()) {
            break;
        }

        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            set_error(errno_message("poll " + device_path_ + " failed"));
            break;
        }

        if (ready == 0) {
            continue;
        }

        if ((poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            std::ostringstream stream;
            stream << "controller device poll event error: revents=0x"
                   << std::hex << poll_fd.revents;
            set_error(stream.str());
            break;
        }

        if ((poll_fd.revents & POLLIN) != 0) {
            if (!read_available_events()) {
                break;
            }
        }
    }

    polling_active_.store(false);
}

/* 非阻塞读取当前已经就绪的手柄事件。 */
bool XboxController::read_available_events()
{
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    if (fd_ < 0) {
        set_error("controller device is not open");
        return false;
    }

    while (true) {
        input_event event = {};
        const ssize_t bytes = ::read(fd_, &event, sizeof(event));

        if (bytes == static_cast<ssize_t>(sizeof(event))) {
            if (event.type == EV_ABS) {
                process_abs_event(event.code, event.value);
            }
            continue;
        }

        if (bytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::lock_guard<std::mutex> state_lock(state_mutex_);
                last_error_.clear();
                return true;
            }
            if (errno == EINTR) {
                continue;
            }
            set_error(errno_message("read " + device_path_ + " failed"));
            return false;
        }

        if (bytes == 0) {
            set_error("controller device returned EOF");
            return false;
        }

        set_error("short read from controller device");
        return false;
    }
}


/* 析构时确保设备文件描述符被释放 */
XboxController::~XboxController()
{
    close_device();
}

/* 关闭已经打开的设备文件描述符 */
void XboxController::close_device()
{
    stop_polling();

    std::lock_guard<std::mutex> io_lock(io_mutex_);
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}


/* 请求后台读取线程退出，并等待当前 poll 等待周期结束。 */
void XboxController::stop_polling()
{
    stop_polling_requested_.store(true);
    if (polling_thread_.joinable()) {
        polling_thread_.join();
    }
    polling_active_.store(false);
}


/* 判断当前是否持有有效的设备文件描述符 */
bool XboxController::is_open() const
{
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    return fd_ >= 0;
}



/* 判断后台读取线程是否仍在运行。 */
bool XboxController::is_polling() const
{
    return polling_active_.load();
}

/* 获取后台线程维护的最近一次速度指令。 */
bool XboxController::latest_command(VelocityCommand& command) const
{
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    command = command_;
    return last_error_.empty();
}

/* 获取最近一次计算出的速度指令。 */
VelocityCommand XboxController::command() const
{
    VelocityCommand latest;
    latest_command(latest);
    return latest;
}

/* 获取最近一次设备操作失败的错误信息。 */
std::string XboxController::last_error() const
{
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    return last_error_;
}


/* 按死区和最大速度限制，将原始摇杆轴值转换为线速度 */
double XboxController::axis_to_speed(int raw_value)
{
    double normalized = 0.0;
    if (raw_value > kAxisCenter + kDeadzone) {
        normalized = positive_axis_scale(raw_value);
    } else if (raw_value < kAxisCenter - kDeadzone) {
        normalized = negative_axis_scale(raw_value);
    }

    normalized = std::clamp(normalized, -1.0, 1.0);
    return normalized * kMaxSpeedMps;
}

/* 使用给定的 X/Y 轴原始值生成机器人平面速度控制指令 */
VelocityCommand XboxController::map_axes(int abs_x, int abs_y)
{
    VelocityCommand command;
    command.raw_abs_x = abs_x;
    command.raw_abs_y = abs_y;
    command.has_abs_x = true;
    command.has_abs_y = true;
    command.vx = std::clamp(-axis_to_speed(abs_y), -kMaxSpeedMps, kMaxSpeedMps);
    command.vy = 0.0;
    command.yaw_rate = kYawRate;
    return command;
}

/* 通过 ioctl 读取设备当前 ABS_X/ABS_Y 轴值 */
void XboxController::initialize_axis_values()
{
    std::lock_guard<std::mutex> state_lock(state_mutex_);

    input_absinfo abs_info = {};
    if (::ioctl(fd_, EVIOCGABS(ABS_X), &abs_info) == 0) {
        command_.raw_abs_x = abs_info.value;
        command_.has_abs_x = true;
    }

    abs_info = {};
    if (::ioctl(fd_, EVIOCGABS(ABS_Y), &abs_info) == 0) {
        command_.raw_abs_y = abs_info.value;
        command_.has_abs_y = true;
    }
}

/* 根据 ABS_X/ABS_Y 事件更新缓存轴值。 */
void XboxController::process_abs_event(int code, int value)
{
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (code == ABS_X) {
        command_.raw_abs_x = value;
        command_.has_abs_x = true;
        refresh_command_locked();
    } else if (code == ABS_Y) {
        command_.raw_abs_y = value;
        command_.has_abs_y = true;
        refresh_command_locked();
    }
}

/* 记录最近一次错误信息。 */
void XboxController::set_error(std::string error)
{
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    last_error_ = std::move(error);
}

/* 根据缓存的轴值刷新 vx/vy/yaw_rate 控制量 */
void XboxController::refresh_command_locked()
{
    command_.vx = std::clamp(-axis_to_speed(command_.raw_abs_y),
                             -kMaxSpeedMps,
                             kMaxSpeedMps);
    command_.vy = 0.0;
    command_.yaw_rate = kYawRate;
}

}  // namespace xbox_control
