#pragma once

// Internal SocketCAN timestamp helpers; not part of the reader API.
#include <linux/time_types.h>
#include <sys/socket.h>
#include <time.h>

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace imu::detail {

constexpr std::int64_t kClockOffsetToleranceNs = 1'000'000;

struct ClockOffsetSample {
    std::int64_t offset_ns = 0;
    std::int64_t monotonic_end_ns = 0;
};

inline bool timestamp_to_ns(std::int64_t seconds, std::int64_t nanoseconds,
                            std::int64_t& result)
{
    constexpr std::int64_t kNsPerSecond = 1'000'000'000;
    if (seconds < 0 || nanoseconds < 0 || nanoseconds >= kNsPerSecond) {
        return false;
    }
    result = seconds * kNsPerSecond + nanoseconds;
    return true;
}

inline bool clock_offset_from_samples(std::int64_t begin_ns,
                                     std::int64_t realtime_ns,
                                     std::int64_t end_ns,
                                     ClockOffsetSample& sample)
{
    if (begin_ns < 0 || realtime_ns < 0 || end_ns < begin_ns ||
        end_ns - begin_ns >= 2 * kClockOffsetToleranceNs) {
        return false;
    }
    const auto midpoint_ns = begin_ns + (end_ns - begin_ns) / 2;
    sample = {realtime_ns - midpoint_ns, end_ns};
    return true;
}

inline bool sample_clock_offset(ClockOffsetSample& sample)
{
    timespec begin{}, realtime{}, end{};
    if (::clock_gettime(CLOCK_MONOTONIC, &begin) != 0 ||
        ::clock_gettime(CLOCK_REALTIME, &realtime) != 0 ||
        ::clock_gettime(CLOCK_MONOTONIC, &end) != 0) {
        return false;
    }
    std::int64_t begin_ns = 0, realtime_ns = 0, end_ns = 0;
    return timestamp_to_ns(begin.tv_sec, begin.tv_nsec, begin_ns) &&
           timestamp_to_ns(realtime.tv_sec, realtime.tv_nsec, realtime_ns) &&
           timestamp_to_ns(end.tv_sec, end.tv_nsec, end_ns) &&
           clock_offset_from_samples(begin_ns, realtime_ns, end_ns, sample);
}

inline bool receive_realtime_timestamp(const msghdr& message,
                                       std::int64_t& timestamp_ns)
{
    if ((message.msg_flags & (MSG_TRUNC | MSG_CTRUNC)) != 0 ||
        message.msg_control == nullptr) {
        return false;
    }
    msghdr headers = message;
    const auto* control_end = static_cast<const unsigned char*>(message.msg_control) +
                              message.msg_controllen;
    for (cmsghdr* header = CMSG_FIRSTHDR(&headers); header != nullptr;
         header = CMSG_NXTHDR(&headers, header)) {
        const auto remaining = control_end -
                               reinterpret_cast<const unsigned char*>(header);
        if (header->cmsg_len < CMSG_LEN(0) ||
            header->cmsg_len > static_cast<std::size_t>(remaining)) {
            return false;
        }
        if (header->cmsg_level == SOL_SOCKET &&
            header->cmsg_type == SO_TIMESTAMPNS_NEW) {
            if (header->cmsg_len != CMSG_LEN(sizeof(__kernel_timespec))) {
                return false;
            }
            __kernel_timespec timestamp{};
            std::memcpy(&timestamp, CMSG_DATA(header), sizeof(timestamp));
            return timestamp_to_ns(timestamp.tv_sec, timestamp.tv_nsec,
                                   timestamp_ns) && timestamp_ns > 0;
        }
    }
    return false;
}

inline bool receive_monotonic_timestamp(std::int64_t realtime_ns,
                                        std::int64_t baseline_offset_ns,
                                        const ClockOffsetSample& current,
                                        std::int64_t& timestamp_ns)
{
    // 内核时钟及当前系统年份处于正常 int64 纳秒时间域。
    const std::int64_t offset_delta_ns = current.offset_ns - baseline_offset_ns;
    const std::int64_t converted_ns = realtime_ns - current.offset_ns;
    if (offset_delta_ns < -kClockOffsetToleranceNs ||
        offset_delta_ns > kClockOffsetToleranceNs ||
        converted_ns <= 0 ||
        converted_ns - current.monotonic_end_ns > kClockOffsetToleranceNs) {
        return false;
    }
    timestamp_ns = converted_ns;
    return true;
}

}  // namespace imu::detail
