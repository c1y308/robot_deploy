#pragma once

#include <linux/can.h>

#include <cstdint>
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

    int wait_readable(int timeout_ms);
    int read_nonblocking(can_frame& frame,
                         std::int64_t* receive_timestamp_ns = nullptr);
    int read(can_frame& frame);

    const std::string& get_interface_name() const { return interface_name_; }
    int get_file_descriptor() const { return fd_; }

private:
    int fd_;
    std::int64_t baseline_clock_offset_ns_{0};
    bool timestamp_mapping_valid_{false};
    std::string interface_name_;
};

}  // namespace imu
