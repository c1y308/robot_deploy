#include "driver/serial_port.hpp"
#include "tool/tool.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <poll.h>
#include <iostream>

namespace imu {

SerialPort::SerialPort() 
    : fd_(-1), baudrate_(0) {
}


SerialPort::~SerialPort() {
    close();
}


int SerialPort::baudrate_to_constant(int baudrate) {
    switch (baudrate) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B921600;
    }
}


bool SerialPort::open(const std::string& device, int baudrate) {
    /* 当前实例已有对应的串口设备 */
    if (fd_ >= 0) {
        close();
    }
    
    device_path_ = device;
    baudrate_ = baudrate;
    
    /* 调用全局的open函数打开设备 */
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "[HARDWARE ERROR] Cannot open serial port " << device 
                  << ": " << strerror(errno) << std::endl;
        std::cerr << "[TODO] 请确认以下硬件配置:" << std::endl;
        std::cerr << "  1. 惯导模块已正确连接到RK3588的串口" << std::endl;
        std::cerr << "  2. 确认串口设备路径 (如 /dev/ttyS1, /dev/ttyUSB0 等)" << std::endl;
        std::cerr << "  3. 检查用户是否有串口访问权限 (可能需要加入 dialout 组)" << std::endl;
        return false;
    }
    
    /* 保存旧的设置属性 */
    if (tcgetattr(fd_, &oldtio_) != 0) {
        std::cerr << "[HARDWARE ERROR] tcgetattr failed: " << strerror(errno) << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    
    /* 设置新的属性 */
    memset(&newtio_, 0, sizeof(newtio_));
    
    newtio_.c_cflag |= CLOCAL | CREAD;
    newtio_.c_cflag |= baudrate_to_constant(baudrate);
    
    newtio_.c_cflag &= ~CSIZE;
    newtio_.c_cflag |= CS8;
    
    /* 禁止奇偶校验 */
    newtio_.c_cflag &= ~PARENB;
    /* 禁用 2 停止位 */
    newtio_.c_cflag &= ~CSTOPB;
    /* 禁用硬件流控 */
    newtio_.c_cflag &= ~CRTSCTS;
    /* 规范模式、回显输入字符、回显擦除字符、启用信号 */
    newtio_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    newtio_.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | INLCR | ICRNL);
    /* 启用输出处理 */
    newtio_.c_oflag &= ~OPOST;
    /* 决定read的行为 */
    newtio_.c_cc[VTIME] = 0;
    newtio_.c_cc[VMIN] = 0;
    
    tcflush(fd_, TCIOFLUSH);
    
    if (tcsetattr(fd_, TCSANOW, &newtio_) != 0) {
        std::cerr << "[HARDWARE ERROR] tcsetattr failed: " << strerror(errno) << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    
    std::cout << "[HARDWARE] Serial port opened: " << device 
              << " @ " << baudrate << " bps" << std::endl;
    return true;
}


/* 关闭并恢复旧的配置 */
void SerialPort::close() {
    if (fd_ >= 0) {
        tcsetattr(fd_, TCSANOW, &oldtio_);
        ::close(fd_);
        fd_ = -1;
        std::cout << "[HARDWARE] Serial port closed" << std::endl;
    }
}


/* 串口是否打开 */
bool SerialPort::is_open() const {
    return fd_ >= 0;
}


/* 读取数据 */
int SerialPort::wait_readable(int timeout_ms) {
    if (fd_ < 0) {
        return -1;
    }

    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;

    const int ret = poll(&pfd, 1, timeout_ms);
    if (ret < 0) {
        if (errno == EINTR) {
            return 0;
        }
        std::cerr << "[HARDWARE ERROR] poll failed: " << strerror(errno) << std::endl;
        return -1;
    }
    if (ret == 0) {
        return 0;
    }

    if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        std::cerr << "[HARDWARE ERROR] serial poll revents=0x"
                  << std::hex << pfd.revents << std::dec << std::endl;
        return -1;
    }

    return (pfd.revents & POLLIN) != 0 ? 1 : 0;
}


int SerialPort::read_nonblocking(uint8_t* buffer,
                                 int max_len,
                                 std::int64_t* host_receive_timestamp_ns) {
    if (host_receive_timestamp_ns != nullptr) {
        *host_receive_timestamp_ns = 0;
    }
    if (fd_ < 0 || buffer == nullptr) {
        return -1;
    }

    int bytes_read = ::read(fd_, buffer, max_len);
    if (bytes_read < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        std::cerr << "[HARDWARE ERROR] read failed: " << strerror(errno) << std::endl;
        return -1;
    }

    if (bytes_read > 0 && host_receive_timestamp_ns != nullptr) {
        *host_receive_timestamp_ns = robot_base::monotonic_now_ns();
    }
    return bytes_read;
}


int SerialPort::read(uint8_t* buffer, int max_len) {
    const int ready = wait_readable(10);
    if (ready <= 0) {
        return ready;
    }
    return read_nonblocking(buffer, max_len, nullptr);
}


/* 写入数据 */
int SerialPort::write(const uint8_t* data, int len) {
    if (fd_ < 0 || data == nullptr) {
        return -1;
    }
    
    int bytes_written = ::write(fd_, data, len);
    if (bytes_written < 0) {
        std::cerr << "[HARDWARE ERROR] write failed: " << strerror(errno) << std::endl;
        return -1;
    }
    
    tcdrain(fd_);
    return bytes_written;
}


/* 设置波特率 */
bool SerialPort::set_baudrate(int baudrate) {
    if (fd_ < 0) {
        return false;
    }
    
    cfsetispeed(&newtio_, baudrate_to_constant(baudrate));
    cfsetospeed(&newtio_, baudrate_to_constant(baudrate));
    
    if (tcsetattr(fd_, TCSANOW, &newtio_) != 0) {
        std::cerr << "[HARDWARE ERROR] tcsetattr failed: " << strerror(errno) << std::endl;
        return false;
    }
    
    baudrate_ = baudrate;
    return true;
}


/* 刷新缓冲区 */
void SerialPort::flush() {
    if (fd_ >= 0) {
        tcflush(fd_, TCIOFLUSH);
    }
}


}
