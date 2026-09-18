#include "driver/socket_can_port.hpp"
#include "drivers/socket_can_timestamp.hpp"

#include <cerrno>
#include <cstdarg>
#include <fcntl.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <poll.h>

#include <deque>
#include <iostream>
#include <stdexcept>

namespace {

void expect(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

constexpr std::int64_t kSecond = 1'000'000'000;
constexpr std::int64_t kRealtime = 1'700'000'000 * kSecond;
constexpr std::int64_t kBaseline = kRealtime - kSecond;
constexpr int kFd = 77;

struct ControlMessage {
    alignas(cmsghdr) unsigned char control[CMSG_SPACE(sizeof(__kernel_timespec))]{};
    msghdr message{};

    explicit ControlMessage(std::int64_t timestamp = kRealtime)
    {
        message.msg_control = control;
        message.msg_controllen = sizeof(control);
        auto* header = CMSG_FIRSTHDR(&message);
        header->cmsg_level = SOL_SOCKET;
        header->cmsg_type = SO_TIMESTAMPNS_NEW;
        header->cmsg_len = CMSG_LEN(sizeof(__kernel_timespec));
        const __kernel_timespec value{timestamp / kSecond, timestamp % kSecond};
        std::memcpy(CMSG_DATA(header), &value, sizeof(value));
    }
};

struct ClockCall {
    clockid_t id;
    std::int64_t ns;
    bool fail = false;
};

struct ReceiveCall {
    int error = 0;
    ssize_t bytes = CAN_MTU;
    int flags = 0;
    bool has_timestamp = true;
    std::int64_t realtime_ns = kRealtime;
};

struct FakeSystem {
    std::deque<ClockCall> clocks;
    std::deque<ReceiveCall> receives;
    bool fail_option = false;
    bool timestamp_enabled = false;
    int closed = 0;
    int received = 0;
    int bound = 0;
    int polls = 0;
    int poll_ready = 1;
} system_calls;

void enqueue_clock_sample(std::int64_t midpoint = kSecond,
                          std::int64_t offset = kBaseline,
                          std::int64_t span = 100)
{
    system_calls.clocks.push_back({CLOCK_MONOTONIC, midpoint - span / 2});
    system_calls.clocks.push_back({CLOCK_REALTIME, midpoint + offset});
    system_calls.clocks.push_back({CLOCK_MONOTONIC, midpoint + span / 2});
}

void open_port(imu::SocketCanPort& port)
{
    system_calls = {};
    enqueue_clock_sample();
    expect(port.open("can-test"), "fake timestamp-enabled socket must open");
    expect(system_calls.clocks.empty() && system_calls.bound == 1,
           "baseline must be established before bind");
}

void test_ancillary_data()
{
    std::int64_t timestamp = 0;
    ControlMessage valid(kRealtime + 123);
    expect(imu::detail::receive_realtime_timestamp(valid.message, timestamp) &&
               timestamp == kRealtime + 123, "NEW timestamp extraction failed");
    for (const int flag : {MSG_TRUNC, MSG_CTRUNC}) {
        ControlMessage truncated;
        truncated.message.msg_flags = flag;
        expect(!imu::detail::receive_realtime_timestamp(truncated.message, timestamp),
               "truncated message must fail closed");
    }
    for (const std::size_t length : {CMSG_LEN(0) - 1,
                                    CMSG_LEN(sizeof(__kernel_timespec)) - 1,
                                    CMSG_SPACE(sizeof(__kernel_timespec)) + 1}) {
        ControlMessage malformed;
        CMSG_FIRSTHDR(&malformed.message)->cmsg_len = length;
        expect(!imu::detail::receive_realtime_timestamp(malformed.message, timestamp),
               "malformed control length accepted");
    }
    ControlMessage wrong_type;
    CMSG_FIRSTHDR(&wrong_type.message)->cmsg_type = SO_TIMESTAMPNS_OLD;
    expect(!imu::detail::receive_realtime_timestamp(wrong_type.message, timestamp),
           "OLD timestamp must not satisfy NEW contract");
    ControlMessage wrong_level;
    CMSG_FIRSTHDR(&wrong_level.message)->cmsg_level = SOL_CAN_RAW;
    expect(!imu::detail::receive_realtime_timestamp(wrong_level.message, timestamp),
           "timestamp at wrong cmsg level accepted");
    msghdr missing{};
    expect(!imu::detail::receive_realtime_timestamp(missing, timestamp),
           "missing control data accepted");
    ControlMessage short_buffer;
    short_buffer.message.msg_controllen = sizeof(cmsghdr) - 1;
    expect(!imu::detail::receive_realtime_timestamp(short_buffer.message, timestamp),
           "short control buffer accepted");
    for (const __kernel_timespec invalid : {
             __kernel_timespec{0, 0}, __kernel_timespec{-1, 0},
             __kernel_timespec{1, -1}, __kernel_timespec{1, kSecond}}) {
        ControlMessage bad_time;
        std::memcpy(CMSG_DATA(CMSG_FIRSTHDR(&bad_time.message)), &invalid, sizeof(invalid));
        expect(!imu::detail::receive_realtime_timestamp(bad_time.message, timestamp),
               "illegal kernel timestamp accepted");
    }
}

void test_clock_mapping()
{
    using namespace imu::detail;
    ClockOffsetSample current;
    expect(clock_offset_from_samples(kSecond - 50, kRealtime, kSecond + 50, current) &&
               current.offset_ns == kBaseline && current.monotonic_end_ns == kSecond + 50,
           "sandwich midpoint mapping is incorrect");
    expect(!clock_offset_from_samples(100, kRealtime, 99, current),
           "reversed monotonic sample accepted");
    expect(!clock_offset_from_samples(-1, kRealtime, 1, current),
           "negative monotonic sample accepted");
    expect(!clock_offset_from_samples(kSecond, kRealtime, kSecond + 2'000'000, current),
           "1ms sampling half-span must be rejected");
    expect(clock_offset_from_samples(kSecond, kRealtime, kSecond + 1'999'999, current),
           "half-span just below 1ms must be accepted");
    std::int64_t converted = 0;
    current = {kBaseline + 500'000, kSecond};
    expect(receive_monotonic_timestamp(kRealtime - 100'000'000 + 500'000,
                                      kBaseline, current, converted) &&
               converted == kSecond - 100'000'000,
           "actual conversion must use current offset, not baseline");
    for (const std::int64_t delta : {-1'000'000, 1'000'000}) {
        current = {kBaseline + delta, kSecond};
        expect(receive_monotonic_timestamp(kRealtime + delta, kBaseline, current, converted) &&
                   converted == kSecond, "exact 1ms offset tolerance must be allowed");
    }
    for (const std::int64_t delta : {-1'000'001, 1'000'001}) {
        current = {kBaseline + delta, kSecond};
        expect(!receive_monotonic_timestamp(kRealtime + delta, kBaseline, current, converted),
               "positive or negative wall-clock step must fail closed");
    }
    current = {kBaseline, kSecond};
    expect(receive_monotonic_timestamp(kRealtime + 1'000'000, kBaseline, current, converted),
           "exact 1ms future tolerance must be allowed");
    expect(!receive_monotonic_timestamp(kRealtime + 1'000'001, kBaseline, current, converted),
           "timestamp too far in the future accepted");
    expect(!receive_monotonic_timestamp(kBaseline, kBaseline, current, converted),
           "zero converted timestamp accepted");
    expect(!receive_monotonic_timestamp(kBaseline - 1, kBaseline, current, converted),
           "negative converted timestamp accepted");

}

void test_receive_and_retry()
{
    imu::SocketCanPort port;
    open_port(port);
    system_calls.receives.push_back({EINTR});
    system_calls.receives.push_back({0, CAN_MTU, 0, true, kRealtime - 100'000'000 + 500'000});
    enqueue_clock_sample(kSecond, kBaseline + 500'000);
    can_frame frame{};
    std::int64_t timestamp = 999;
    expect(port.read_nonblocking(frame, &timestamp) == 1 &&
               system_calls.received == 2 && timestamp == kSecond - 100'000'000 &&
               frame.can_id == 0x021,
           "EINTR must retry with a fresh msghdr and preserve kernel queue age");
    system_calls.receives.push_back({EAGAIN});
    expect(port.read_nonblocking(frame, &timestamp) == 0 && timestamp == 0 &&
               system_calls.clocks.empty(), "EAGAIN must return zero without timestamp fallback");
    system_calls.receives.push_back({EIO});
    timestamp = 999;
    expect(port.read_nonblocking(frame, &timestamp) == -1 && timestamp == 0,
           "receive error must clear output timestamp");
    system_calls.receives.push_back({0, CAN_MTU - 1});
    expect(port.read_nonblocking(frame, &timestamp) == -1 && timestamp == 0,
           "short CAN frame accepted");
    system_calls.receives.push_back({0, CAN_MTU + 1});
    expect(port.read_nonblocking(frame, &timestamp) == -1,
           "oversized CAN frame accepted");
    system_calls.receives.push_back({});
    enqueue_clock_sample();
    expect(port.read(frame) == 1, "legacy read must delegate to timestamp-validated recvmsg");
}

void test_mapping_failure_latches()
{
    for (const ReceiveCall bad : {
             ReceiveCall{0, CAN_MTU, 0, false}, ReceiveCall{0, CAN_MTU, MSG_CTRUNC},
             ReceiveCall{0, CAN_MTU, MSG_TRUNC}, ReceiveCall{0, CAN_MTU, 0, true, -1}}) {
        imu::SocketCanPort port;
        open_port(port);
        system_calls.receives.push_back(bad);
        can_frame frame{};
        std::int64_t timestamp = 999;
        expect(port.read_nonblocking(frame, &timestamp) == -1 && timestamp == 0,
               "missing or invalid ancillary timestamp must fail closed");
        const auto calls = system_calls.received;
        expect(port.read_nonblocking(frame, &timestamp) == -1 &&
                   system_calls.received == calls,
               "invalid mapping must remain invalid without consuming more queued frames");
        system_calls.poll_ready = 0;
        expect(port.read(frame) == -1 && system_calls.polls == 0,
               "legacy read must return a latched mapping error even with an empty queue");
    }
    for (const std::int64_t delta : {-1'000'001, 1'000'001}) {
        imu::SocketCanPort port;
        open_port(port);
        system_calls.receives.push_back({});
        enqueue_clock_sample(kSecond, kBaseline + delta);
        can_frame frame{};
        std::int64_t timestamp = 999;
        expect(port.read_nonblocking(frame, &timestamp) == -1 && timestamp == 0 &&
                   port.read_nonblocking(frame, &timestamp) == -1 && system_calls.received == 1,
               "clock step must invalidate mapping until reopen");
        port.close();
        enqueue_clock_sample(kSecond, kBaseline + delta);
        expect(port.open("can-test"), "reopen must establish a new baseline");
        system_calls.receives.push_back({0, CAN_MTU, 0, true, kRealtime + delta});
        enqueue_clock_sample(kSecond, kBaseline + delta);
        expect(port.read_nonblocking(frame, &timestamp) == 1 && timestamp == kSecond,
               "explicit reopen must restore a new mapping");
    }
}

void test_clock_and_startup_failures()
{
    for (int failed_call = 0; failed_call < 3; ++failed_call) {
        imu::SocketCanPort port;
        open_port(port);
        system_calls.receives.push_back({});
        enqueue_clock_sample();
        system_calls.clocks[failed_call].fail = true;
        can_frame frame{};
        std::int64_t timestamp = 999;
        expect(port.read_nonblocking(frame, &timestamp) == -1 && timestamp == 0 &&
                   port.read_nonblocking(frame, nullptr) == -1,
               "every clock-read failure must latch a mapping error");
    }
    for (const std::int64_t span : {-100, 2'000'000}) {
        imu::SocketCanPort port;
        open_port(port);
        system_calls.receives.push_back({});
        enqueue_clock_sample(kSecond, kBaseline, span);
        can_frame frame{};
        expect(port.read_nonblocking(frame) == -1,
               "untrustworthy clock sampling must reject reception");
    }
    system_calls = {};
    system_calls.fail_option = true;
    imu::SocketCanPort port;
    expect(!port.open("can-test") && !port.is_open() && system_calls.closed == 1 &&
               system_calls.bound == 0, "timestamp option failure must close the socket before bind");
    system_calls = {};
    enqueue_clock_sample();
    system_calls.clocks[0].fail = true;
    expect(!port.open("can-test") && !port.is_open() && system_calls.closed == 1 &&
               system_calls.bound == 0, "baseline clock failure must close the socket before bind");
}

}  // namespace

// Link wrapping is confined to this test executable; production has no injection API.
extern "C" int __wrap_socket(int domain, int type, int protocol)
{
    expect(domain == PF_CAN && type == SOCK_RAW && protocol == CAN_RAW, "unexpected socket request");
    system_calls.timestamp_enabled = false;
    return kFd;
}

extern "C" int __wrap_fcntl(int fd, int command, ...)
{
    expect(fd == kFd, "unexpected fcntl fd");
    if (command == F_GETFL) return 0;
    va_list arguments;
    va_start(arguments, command);
    const int flags = va_arg(arguments, int);
    va_end(arguments);
    expect(command == F_SETFL && (flags & O_NONBLOCK), "socket must be nonblocking");
    return 0;
}

extern "C" int __wrap_setsockopt(int fd, int level, int option,
                                 const void* value, socklen_t size)
{
    expect(fd == kFd && level == SOL_SOCKET && option == SO_TIMESTAMPNS_NEW &&
               size == sizeof(int) && *static_cast<const int*>(value) == 1,
           "NEW RX timestamps must be enabled");
    if (system_calls.fail_option) { errno = ENOPROTOOPT; return -1; }
    system_calls.timestamp_enabled = true;
    return 0;
}

extern "C" int __wrap_ioctl(int fd, unsigned long request, ...)
{
    expect(fd == kFd && request == SIOCGIFINDEX, "unexpected ioctl");
    va_list arguments;
    va_start(arguments, request);
    auto* interface = va_arg(arguments, ifreq*);
    va_end(arguments);
    interface->ifr_ifindex = 3;
    return 0;
}

extern "C" int __wrap_bind(int fd, const sockaddr* address, socklen_t size)
{
    expect(fd == kFd && system_calls.timestamp_enabled && system_calls.clocks.empty() &&
               size == sizeof(sockaddr_can) && address->sa_family == AF_CAN,
           "bind must follow timestamp setup and baseline sampling");
    ++system_calls.bound;
    return 0;
}

extern "C" int __wrap_close(int fd)
{
    expect(fd == kFd, "unexpected close fd");
    ++system_calls.closed;
    return 0;
}

extern "C" int __wrap_clock_gettime(clockid_t id, timespec* time)
{
    expect(!system_calls.clocks.empty(), "unexpected clock sampling");
    const auto call = system_calls.clocks.front();
    system_calls.clocks.pop_front();
    expect(id == call.id, "clock sampling must follow M1 R M2 order");
    if (call.fail) { errno = EIO; return -1; }
    *time = {call.ns / kSecond, call.ns % kSecond};
    return 0;
}

extern "C" ssize_t __wrap_recvmsg(int fd, msghdr* message, int flags)
{
    expect(fd == kFd && flags == 0 && !system_calls.receives.empty() &&
               message->msg_iovlen == 1 && message->msg_iov[0].iov_len == CAN_MTU &&
               message->msg_controllen == CMSG_SPACE(sizeof(__kernel_timespec)) &&
               message->msg_flags == 0,
           "each recvmsg attempt must initialize its payload and control buffer");
    const auto call = system_calls.receives.front();
    system_calls.receives.pop_front();
    ++system_calls.received;
    if (call.error) {
        // Ensure the EINTR retry does not reuse mutated output fields.
        message->msg_controllen = 0;
        message->msg_flags = MSG_CTRUNC;
        errno = call.error;
        return -1;
    }
    auto* frame = static_cast<can_frame*>(message->msg_iov[0].iov_base);
    *frame = {};
    frame->can_id = 0x021;
    frame->can_dlc = 8;
    message->msg_flags = call.flags;
    if (call.has_timestamp) {
        auto* header = CMSG_FIRSTHDR(message);
        header->cmsg_level = SOL_SOCKET;
        header->cmsg_type = SO_TIMESTAMPNS_NEW;
        header->cmsg_len = CMSG_LEN(sizeof(__kernel_timespec));
        const __kernel_timespec value{call.realtime_ns / kSecond, call.realtime_ns % kSecond};
        std::memcpy(CMSG_DATA(header), &value, sizeof(value));
    } else {
        message->msg_controllen = 0;
    }
    return call.bytes;
}

extern "C" int __wrap_poll(pollfd* descriptors, nfds_t count, int)
{
    expect(count == 1 && descriptors[0].fd == kFd, "unexpected poll");
    ++system_calls.polls;
    descriptors[0].revents = system_calls.poll_ready ? POLLIN : 0;
    return system_calls.poll_ready;
}

int main()
{
    try {
        test_ancillary_data();
        test_clock_mapping();
        test_receive_and_retry();
        test_mapping_failure_latches();
        test_clock_and_startup_failures();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[SOCKET_CAN_TIMESTAMP_TEST] " << error.what() << '\n';
        return 1;
    }
}
