#include "driver/socket_can_port.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
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

int SocketCanPort::read(can_frame& frame)
{
    if (fd_ < 0) {
        return -1;
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);

    timeval timeout = {};
    timeout.tv_usec = 10000;

    const int ready = select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready < 0) {
        if (errno == EINTR) {
            return 0;
        }
        std::cerr << "[HARDWARE ERROR] CAN select failed: "
                  << std::strerror(errno) << std::endl;
        return -1;
    }
    if (ready == 0) {
        return 0;
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

    return 1;
}

}  // namespace imu
