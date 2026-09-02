#include "driver/socket_can_config.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <linux/can/netlink.h>
#include <linux/if_link.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

namespace imu {
namespace {

struct LinkRequest {
    nlmsghdr header;
    ifinfomsg info;
    char attributes[1024];
};

class FileDescriptor {
public:
    explicit FileDescriptor(int fd)
        : fd_(fd)
    {
    }

    ~FileDescriptor()
    {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    FileDescriptor(const FileDescriptor&) = delete;
    FileDescriptor& operator=(const FileDescriptor&) = delete;

    int get() const { return fd_; }

private:
    int fd_;
};

void set_error(std::string* error, const std::string& message)
{
    if (error != nullptr) {
        *error = message;
    }
}

std::string errno_message(const std::string& operation, int error_number)
{
    std::string message = operation + ": " + std::strerror(error_number);
    if (error_number == EPERM || error_number == EACCES) {
        message += " (run as root or grant CAP_NET_ADMIN)";
    }
    return message;
}

bool add_attr(nlmsghdr* header,
              std::size_t max_length,
              int type,
              const void* data,
              std::size_t data_length,
              std::string* error)
{
    const std::size_t attr_length = RTA_LENGTH(data_length);
    const std::size_t next_length =
        NLMSG_ALIGN(header->nlmsg_len) + RTA_ALIGN(attr_length);
    if (next_length > max_length) {
        set_error(error, "netlink request is too large");
        return false;
    }

    auto* attr = reinterpret_cast<rtattr*>(
        reinterpret_cast<char*>(header) + NLMSG_ALIGN(header->nlmsg_len));
    attr->rta_type = type;
    attr->rta_len = attr_length;
    if (data_length > 0) {
        std::memcpy(RTA_DATA(attr), data, data_length);
    }

    header->nlmsg_len = next_length;
    return true;
}

rtattr* begin_nested_attr(nlmsghdr* header,
                          std::size_t max_length,
                          int type,
                          std::string* error)
{
    const std::size_t attr_length = RTA_LENGTH(0);
    const std::size_t next_length =
        NLMSG_ALIGN(header->nlmsg_len) + RTA_ALIGN(attr_length);
    if (next_length > max_length) {
        set_error(error, "netlink nested request is too large");
        return nullptr;
    }

    auto* attr = reinterpret_cast<rtattr*>(
        reinterpret_cast<char*>(header) + NLMSG_ALIGN(header->nlmsg_len));
    attr->rta_type = type;
    attr->rta_len = attr_length;

    header->nlmsg_len = next_length;
    return attr;
}

void end_nested_attr(nlmsghdr* header, rtattr* attr)
{
    attr->rta_len = static_cast<unsigned short>(
        reinterpret_cast<char*>(header) + header->nlmsg_len -
        reinterpret_cast<char*>(attr));
}

bool send_request_and_wait_ack(int fd,
                               nlmsghdr* request,
                               const std::string& operation,
                               std::string* error)
{
    sockaddr_nl kernel_address{};
    kernel_address.nl_family = AF_NETLINK;

    iovec iov{};
    iov.iov_base = request;
    iov.iov_len = request->nlmsg_len;

    msghdr message{};
    message.msg_name = &kernel_address;
    message.msg_namelen = sizeof(kernel_address);
    message.msg_iov = &iov;
    message.msg_iovlen = 1;

    if (::sendmsg(fd, &message, 0) < 0) {
        set_error(error, errno_message(operation + " send", errno));
        return false;
    }

    char buffer[8192];
    while (true) {
        const ssize_t received = ::recv(fd, buffer, sizeof(buffer), 0);
        if (received < 0) {
            if (errno == EINTR) {
                continue;
            }
            set_error(error, errno_message(operation + " receive ack", errno));
            return false;
        }

        int remaining = static_cast<int>(received);
        for (auto* header = reinterpret_cast<nlmsghdr*>(buffer);
             NLMSG_OK(header, remaining);
             header = NLMSG_NEXT(header, remaining)) {
            if (header->nlmsg_seq != request->nlmsg_seq) {
                continue;
            }

            if (header->nlmsg_type == NLMSG_ERROR) {
                if (header->nlmsg_len < NLMSG_LENGTH(sizeof(nlmsgerr))) {
                    set_error(error, operation + ": short netlink error ack");
                    return false;
                }

                const auto* ack =
                    reinterpret_cast<const nlmsgerr*>(NLMSG_DATA(header));
                if (ack->error == 0) {
                    return true;
                }

                set_error(error, errno_message(operation, -ack->error));
                return false;
            }

            if (header->nlmsg_type == NLMSG_DONE) {
                return true;
            }
        }
    }
}

bool set_link_up(int fd,
                 unsigned int sequence,
                 unsigned int ifindex,
                 bool up,
                 std::string* error)
{
    LinkRequest request{};
    request.header.nlmsg_len = NLMSG_LENGTH(sizeof(ifinfomsg));
    request.header.nlmsg_type = RTM_NEWLINK;
    request.header.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
    request.header.nlmsg_seq = sequence;
    request.info.ifi_family = AF_UNSPEC;
    request.info.ifi_index = static_cast<int>(ifindex);
    request.info.ifi_flags = up ? IFF_UP : 0;
    request.info.ifi_change = IFF_UP;

    return send_request_and_wait_ack(
        fd, &request.header, up ? "set CAN interface up" :
                                  "set CAN interface down", error);
}

bool set_can_bitrate(int fd,
                     unsigned int sequence,
                     unsigned int ifindex,
                     int bitrate,
                     std::string* error)
{
    LinkRequest request{};
    request.header.nlmsg_len = NLMSG_LENGTH(sizeof(ifinfomsg));
    request.header.nlmsg_type = RTM_NEWLINK;
    request.header.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
    request.header.nlmsg_seq = sequence;
    request.info.ifi_family = AF_UNSPEC;
    request.info.ifi_index = static_cast<int>(ifindex);

    rtattr* link_info =
        begin_nested_attr(&request.header, sizeof(request), IFLA_LINKINFO, error);
    if (link_info == nullptr) {
        return false;
    }
    const char can_kind[] = "can";
    if (!add_attr(&request.header,
                  sizeof(request),
                  IFLA_INFO_KIND,
                  can_kind,
                  sizeof(can_kind),
                  error)) {
        return false;
    }

    rtattr* info_data =
        begin_nested_attr(&request.header, sizeof(request), IFLA_INFO_DATA, error);
    if (info_data == nullptr) {
        return false;
    }

    can_bittiming bit_timing{};
    bit_timing.bitrate = static_cast<__u32>(bitrate);
    if (!add_attr(&request.header,
                  sizeof(request),
                  IFLA_CAN_BITTIMING,
                  &bit_timing,
                  sizeof(bit_timing),
                  error)) {
        return false;
    }

    end_nested_attr(&request.header, info_data);
    end_nested_attr(&request.header, link_info);

    return send_request_and_wait_ack(
        fd, &request.header, "set CAN bitrate", error);
}

}  // namespace

bool validate_socket_can_config(const std::string& interface_name,
                                int bitrate,
                                std::string* error)
{
    if (interface_name.empty()) {
        set_error(error, "CAN interface name is empty");
        return false;
    }
    if (interface_name.size() >= IFNAMSIZ) {
        set_error(error, "CAN interface name is too long: " + interface_name);
        return false;
    }
    if (bitrate <= 0) {
        set_error(error, "CAN bitrate must be positive");
        return false;
    }
    return true;
}

bool configure_socket_can_interface(const std::string& interface_name,
                                    int bitrate,
                                    std::string* error)
{
    std::string local_error;
    std::string* out_error = error != nullptr ? error : &local_error;
    if (!validate_socket_can_config(interface_name, bitrate, out_error)) {
        if (error == nullptr) {
            std::cerr << "[HARDWARE ERROR] Cannot configure CAN interface "
                      << interface_name << ": " << local_error << std::endl;
        }
        return false;
    }

    const unsigned int ifindex = if_nametoindex(interface_name.c_str());
    if (ifindex == 0) {
        set_error(out_error,
                  errno_message("find CAN interface " + interface_name, errno));
        if (error == nullptr) {
            std::cerr << "[HARDWARE ERROR] Cannot configure CAN interface "
                      << interface_name << ": " << local_error << std::endl;
        }
        return false;
    }

    FileDescriptor fd(::socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC,
                               NETLINK_ROUTE));
    if (fd.get() < 0) {
        set_error(out_error, errno_message("open rtnetlink socket", errno));
        if (error == nullptr) {
            std::cerr << "[HARDWARE ERROR] Cannot configure CAN interface "
                      << interface_name << ": " << local_error << std::endl;
        }
        return false;
    }

    unsigned int sequence = 1;
    if (!set_link_up(fd.get(), sequence++, ifindex, false, out_error) ||
        !set_can_bitrate(fd.get(), sequence++, ifindex, bitrate, out_error) ||
        !set_link_up(fd.get(), sequence++, ifindex, true, out_error)) {
        if (error == nullptr) {
            std::cerr << "[HARDWARE ERROR] Cannot configure CAN interface "
                      << interface_name << ": " << local_error << std::endl;
        }
        return false;
    }

    std::cout << "[HARDWARE] SocketCAN configured: " << interface_name
              << " bitrate=" << bitrate << std::endl;
    return true;
}

}  // namespace imu
