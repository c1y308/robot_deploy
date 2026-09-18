#include "driver/socket_can_port.hpp"
#include "socket_can_timestamp.hpp"

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
    baseline_clock_offset_ns_ = 0;
    timestamp_mapping_valid_ = false;
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

    const int enable_timestamp = 1;
    if (::setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPNS_NEW,
                     &enable_timestamp, sizeof(enable_timestamp)) < 0) {
        std::cerr << "[HARDWARE ERROR] Cannot enable CAN RX timestamps: "
                  << std::strerror(errno) << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    detail::ClockOffsetSample baseline;
    if (!detail::sample_clock_offset(baseline)) {
        std::cerr << "[HARDWARE ERROR] Cannot establish CAN timestamp clock mapping"
                  << std::endl;
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
    baseline_clock_offset_ns_ = baseline.offset_ns;
    timestamp_mapping_valid_ = true;
    std::cout << "[HARDWARE] SocketCAN opened: " << interface_name_
              << std::endl;
    return true;
}

void SocketCanPort::close()
{
    baseline_clock_offset_ns_ = 0;
    timestamp_mapping_valid_ = false;
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
    if (fd_ < 0 || !timestamp_mapping_valid_) {
        return -1;
    }

    alignas(cmsghdr) unsigned char control[CMSG_SPACE(sizeof(__kernel_timespec))];
    iovec payload{&frame, sizeof(frame)};
    msghdr message{};
    ssize_t bytes_read;
    do {
        std::memset(control, 0, sizeof(control));
        message = {};
        message.msg_iov = &payload;
        message.msg_iovlen = 1;
        message.msg_control = control;
        message.msg_controllen = sizeof(control);
        bytes_read = ::recvmsg(fd_, &message, 0);
    } while (bytes_read < 0 && errno == EINTR);
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

    std::int64_t realtime_ns = 0, monotonic_ns = 0;
    detail::ClockOffsetSample current;
    if (!detail::receive_realtime_timestamp(message, realtime_ns) ||
        !detail::sample_clock_offset(current) ||
        !detail::receive_monotonic_timestamp(
            realtime_ns, baseline_clock_offset_ns_, current, monotonic_ns)) {
        timestamp_mapping_valid_ = false;
        std::cerr << "[HARDWARE ERROR] CAN RX timestamp missing, invalid, "
                     "or clock mapping changed; reopen required" << std::endl;
        return -1;
    }
    if (receive_timestamp_ns != nullptr) {
        *receive_timestamp_ns = monotonic_ns;
    }
    return 1;
}

int SocketCanPort::read(can_frame& frame)
{
    if (fd_ < 0 || !timestamp_mapping_valid_) {
        return -1;
    }
    const int ready = wait_readable(10);
    if (ready <= 0) {
        return ready;
    }
    return read_nonblocking(frame, nullptr);
}

}  // namespace imu
