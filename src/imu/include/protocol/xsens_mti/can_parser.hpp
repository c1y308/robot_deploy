#pragma once

#include "imu_base/imu_base.hpp"

#include <cstdint>
#include <functional>

namespace imu {

constexpr std::uint32_t XCDI_SAMPLE_TIME_ID = 0x005;
constexpr std::uint32_t XCDI_QUATERNION_ID = 0x021;
constexpr std::uint32_t XCDI_RATE_OF_TURN_ID = 0x032;

class XsensMtiCanParser {
public:
    using IMUCallback_t  = std::function<void(const imu_base::IMUData&)>;
    using AHRSCallback_t = std::function<void(const imu_base::AHRSData&)>;

    XsensMtiCanParser();

    void feed(std::uint32_t can_id,
              const std::uint8_t* data,
              std::uint8_t len,
              std::int64_t receive_timestamp_ns = 0);
    void reset();

    void set_imu_callback(IMUCallback_t callback) { imu_callback_ = callback; }
    void set_ahrs_callback(AHRSCallback_t callback) { ahrs_callback_ = callback; }

    bool get_ahrs_data(imu_base::AHRSData& ahrs);

    const imu_base::ParserInfo& get_info() const { return stats_; }
    void reset_info();

    static void print_ahrs_data(const imu_base::AHRSData& ahrs);

private:
    void parse_sample_time(const std::uint8_t* data);
    void parse_quaternion(const std::uint8_t* data,
                          std::int64_t receive_timestamp_ns);
    void parse_rate_of_turn(const std::uint8_t* data,
                            std::int64_t receive_timestamp_ns);
    void update_orientation_from_quaternion();
    void publish_ahrs_if_ready();

    imu_base::AHRSData ahrs_data_;
    bool ahrs_ready_;
    bool quaternion_fresh_;
    bool rate_of_turn_fresh_;
    std::uint64_t sample_timestamp_ns_;

    imu_base::ParserInfo stats_;
    IMUCallback_t imu_callback_;
    AHRSCallback_t ahrs_callback_;
};

}  // namespace imu
