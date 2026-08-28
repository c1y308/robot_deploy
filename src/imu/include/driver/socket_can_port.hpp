#pragma once

#include <linux/can.h>

#include <string>

namespace imu {

class SocketCanPort {
public:
    SocketCanPort();
    ~SocketCanPort();

    SocketCanPort(const SocketCanPort&) = delete;
    SocketCanPort& operator=(const SocketCanPort&) = delete;

    bool open(const std::string& interface_name);
    void close();
    bool is_open() const;

    int read(can_frame& frame);

    const std::string& get_interface_name() const { return interface_name_; }
    int get_file_descriptor() const { return fd_; }

private:
    int fd_;
    std::string interface_name_;
};

}  // namespace imu
