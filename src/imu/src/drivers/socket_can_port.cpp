#include "driver/socket_can_port.hpp"
#include "tool/tool.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace imu {

SocketCanPort::SocketCanPort()
    : fd_(-1)
{
}

SocketCanPort::~SocketCanPort()
{
    close();
}

bool SocketCanPort::open(const std::string& interface_name)
{
    if (fd_ >= 0) {
        close();
    }
    if (interface_name.size() >= IFNAMSIZ) {
        std::cerr << "[HARDWARE ERROR] CAN interface name is too long: "
                  << interface_name << std::endl;
        return false;
    }

    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
        std::cerr << "[HARDWARE ERROR] Cannot open SocketCAN socket: "
                  << std::strerror(errno) << std::endl;
        return false;
    }

    const int flags = fcntl(fd_, F_GETFL, 0);
    if (flags < 0 || fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
        std::cerr << "[HARDWARE ERROR] Cannot set CAN socket nonblocking: "
                  << std::strerror(errno) << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    ifreq ifr = {};
    std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "[HARDWARE ERROR] Cannot find CAN interface "
                  << interface_name << ": " << std::strerror(errno)
                  << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "[HARDWARE ERROR] Cannot bind CAN interface "
                  << interface_name << ": " << std::strerror(errno)
                  << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    interface_name_ = interface_name;
    std::cout << "[HARDWARE] SocketCAN opened: " << interface_name_
              << std::endl;
    return true;
}

void SocketCanPort::close()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        std::cout << "[HARDWARE] SocketCAN closed" << std::endl;
    }
}

bool SocketCanPort::is_open() const
{
    return fd_ >= 0;
}

int SocketCanPort::wait_readable(int timeout_ms)
{
    if (fd_ < 0) {
        return -1;
    }

    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;

    const int ready = poll(&pfd, 1, timeout_ms);
    if (ready < 0) {
        if (errno == EINTR) {
            return 0;
        }
        std::cerr << "[HARDWARE ERROR] CAN poll failed: "
                  << std::strerror(errno) << std::endl;
        return -1;
    }
    if (ready == 0) {
        return 0;
    }

    if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        std::cerr << "[HARDWARE ERROR] CAN poll revents=0x"
                  << std::hex << pfd.revents << std::dec << std::endl;
        return -1;
    }

    return (pfd.revents & POLLIN) != 0 ? 1 : 0;
}

int SocketCanPort::read_nonblocking(
    can_frame& frame,
    std::int64_t* receive_timestamp_ns)
{
    if (receive_timestamp_ns != nullptr) {
        *receive_timestamp_ns = 0;
    }
    if (fd_ < 0) {
        return -1;
    }

    const ssize_t bytes_read = ::read(fd_, &frame, sizeof(frame));
    if (bytes_read < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        std::cerr << "[HARDWARE ERROR] CAN read failed: "
                  << std::strerror(errno) << std::endl;
        return -1;
    }
    if (bytes_read != CAN_MTU) {
        std::cerr << "[HARDWARE ERROR] CAN read returned " << bytes_read
                  << " bytes, expected " << CAN_MTU << std::endl;
        return -1;
    }

    if (receive_timestamp_ns != nullptr) {
        *receive_timestamp_ns = robot_base::monotonic_now_ns();
    }
    return 1;
}

int SocketCanPort::read(can_frame& frame)
{
    const int ready = wait_readable(10);
    if (ready <= 0) {
        return ready;
    }
    return read_nonblocking(frame, nullptr);
}

}  // namespace imu
