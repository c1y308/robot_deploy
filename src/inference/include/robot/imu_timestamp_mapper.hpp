#pragma once

#include "imu_base/imu_base.hpp"

#include <cstdint>
#include <limits>

namespace inference {

class ImuTimestampMapper {
public:
    explicit ImuTimestampMapper(imu_base::ReaderType type)
        : type_(type)
    {
    }

    void reset(imu_base::ReaderType type)
    {
        type_ = type;
        has_last_device_timestamp_ = false;
        last_raw_device_timestamp_us_ = 0;
        last_unwrapped_device_timestamp_us_ = 0;
        xsens_wrap_base_us_ = 0;
        min_offset_initialized_ = false;
        min_offset_ns_ = 0;
        last_host_sample_timestamp_ns_ = 0;
    }

    bool map_device_timestamp_us(std::uint64_t device_timestamp_us,
                                 std::int64_t host_receive_timestamp_ns,
                                 std::int64_t& host_sample_timestamp_ns)
    {
        host_sample_timestamp_ns = 0;
        if (host_receive_timestamp_ns <= 0) {
            return false;
        }

        std::uint64_t unwrapped_device_timestamp_us = 0;
        if (!unwrap_device_timestamp_us(device_timestamp_us,
                                        unwrapped_device_timestamp_us)) {
            reset(type_);
            return false;
        }
        if (unwrapped_device_timestamp_us >
            static_cast<std::uint64_t>(
                std::numeric_limits<std::int64_t>::max() / 1000)) {
            reset(type_);
            return false;
        }

        const std::int64_t device_timestamp_ns =
            static_cast<std::int64_t>(unwrapped_device_timestamp_us * 1000ULL);
        const std::int64_t offset_ns =
            host_receive_timestamp_ns - device_timestamp_ns;
        if (!min_offset_initialized_ || offset_ns < min_offset_ns_) {
            min_offset_ns_ = offset_ns;
            min_offset_initialized_ = true;
        }

        std::int64_t mapped_sample_timestamp_ns =
            device_timestamp_ns + min_offset_ns_;
        if (last_host_sample_timestamp_ns_ > 0 &&
            mapped_sample_timestamp_ns <= last_host_sample_timestamp_ns_) {
            mapped_sample_timestamp_ns = last_host_sample_timestamp_ns_ + 1;
        }

        last_host_sample_timestamp_ns_ = mapped_sample_timestamp_ns;
        host_sample_timestamp_ns = mapped_sample_timestamp_ns;
        return true;
    }

private:
    static constexpr std::uint64_t kXsensTimestampWrapUs =
        (1ULL << 32) * 100ULL;
    static constexpr std::uint64_t kXsensTimestampHalfWrapUs =
        kXsensTimestampWrapUs / 2ULL;

    bool unwrap_device_timestamp_us(std::uint64_t raw_timestamp_us,
                                    std::uint64_t& unwrapped_timestamp_us)
    {
        unwrapped_timestamp_us = 0;
        if (type_ != imu_base::ReaderType::XSENS_MTI_CAN) {
            if (has_last_device_timestamp_ &&
                raw_timestamp_us < last_raw_device_timestamp_us_) {
                return false;
            }
            has_last_device_timestamp_ = true;
            last_raw_device_timestamp_us_ = raw_timestamp_us;
            last_unwrapped_device_timestamp_us_ = raw_timestamp_us;
            unwrapped_timestamp_us = raw_timestamp_us;
            return true;
        }

        if (raw_timestamp_us >= kXsensTimestampWrapUs) {
            return false;
        }
        if (!has_last_device_timestamp_) {
            has_last_device_timestamp_ = true;
            last_raw_device_timestamp_us_ = raw_timestamp_us;
            last_unwrapped_device_timestamp_us_ = raw_timestamp_us;
            unwrapped_timestamp_us = raw_timestamp_us;
            return true;
        }

        const std::uint64_t previous_raw_us =
            last_unwrapped_device_timestamp_us_ % kXsensTimestampWrapUs;
        if (raw_timestamp_us < previous_raw_us) {
            const std::uint64_t backward_us = previous_raw_us - raw_timestamp_us;
            if (backward_us > kXsensTimestampHalfWrapUs) {
                xsens_wrap_base_us_ += kXsensTimestampWrapUs;
            } else {
                return false;
            }
        } else if (raw_timestamp_us - previous_raw_us >
                   kXsensTimestampHalfWrapUs) {
            return false;
        }

        const std::uint64_t candidate_unwrapped_us =
            xsens_wrap_base_us_ + raw_timestamp_us;
        if (candidate_unwrapped_us < last_unwrapped_device_timestamp_us_) {
            return false;
        }

        last_raw_device_timestamp_us_ = raw_timestamp_us;
        last_unwrapped_device_timestamp_us_ = candidate_unwrapped_us;
        unwrapped_timestamp_us = candidate_unwrapped_us;
        return true;
    }

    imu_base::ReaderType type_;
    bool has_last_device_timestamp_{false};
    std::uint64_t last_raw_device_timestamp_us_{0};
    std::uint64_t last_unwrapped_device_timestamp_us_{0};
    std::uint64_t xsens_wrap_base_us_{0};
    bool min_offset_initialized_{false};
    std::int64_t min_offset_ns_{0};
    std::int64_t last_host_sample_timestamp_ns_{0};
};

}  // namespace inference
