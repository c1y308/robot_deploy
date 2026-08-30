#include "protocol/xsens_mti/can_parser.hpp"
#include "tool/tool.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <iomanip>
#include <iostream>

namespace imu {
namespace {

constexpr double kQuaternionScale = 1.0 / 32767.0;
constexpr double kRateOfTurnScale = 1.0 / 512.0;
constexpr std::uint64_t kSampleTimeTickUs = 100;

std::int16_t read_i16_be(const std::uint8_t* data, std::size_t offset)
{
    const int value = (static_cast<int>(data[offset]) << 8) |
                      static_cast<int>(data[offset + 1]);
    return static_cast<std::int16_t>(
        value >= 0x8000 ? value - 0x10000 : value);
}

std::uint32_t read_u32_be(const std::uint8_t* data)
{
    return (static_cast<std::uint32_t>(data[0]) << 24) |
           (static_cast<std::uint32_t>(data[1]) << 16) |
           (static_cast<std::uint32_t>(data[2]) << 8) |
           static_cast<std::uint32_t>(data[3]);
}

double clamp_unit(double value)
{
    return std::max(-1.0, std::min(1.0, value));
}

}  // namespace

XsensMtiCanParser::XsensMtiCanParser()
{
    reset();
}

void XsensMtiCanParser::reset_info()
{
    stats_ = imu_base::ParserInfo();
}

void XsensMtiCanParser::reset()
{
    ahrs_data_ = imu_base::AHRSData();
    ahrs_ready_ = false;
    sample_time_fresh_ = false;
    quaternion_fresh_ = false;
    rate_of_turn_fresh_ = false;
    sample_timestamp_us_ = 0;
    sample_receive_timestamp_ns_ = 0;
    stats_ = imu_base::ParserInfo();
}

void XsensMtiCanParser::feed(std::uint32_t can_id,
                             const std::uint8_t* data,
                             std::uint8_t len,
                             std::int64_t host_receive_timestamp_ns)
{
    if (data == nullptr) {
        stats_.error_frames++;
        return;
    }

    stats_.total_bytes += len;

    switch (can_id) {
        case XCDI_SAMPLE_TIME_ID:
            if (len != 4) {
                stats_.error_frames++;
                return;
            }
            parse_sample_time(data, host_receive_timestamp_ns);
            stats_.total_frames++;
            return;
        case XCDI_QUATERNION_ID:
            if (len != 8) {
                stats_.error_frames++;
                return;
            }
            parse_quaternion(data, host_receive_timestamp_ns);
            stats_.total_frames++;
            publish_ahrs_if_ready();
            return;
        case XCDI_RATE_OF_TURN_ID:
            if (len != 6) {
                stats_.error_frames++;
                return;
            }
            parse_rate_of_turn(data, host_receive_timestamp_ns);
            stats_.total_frames++;
            publish_ahrs_if_ready();
            return;
        default:
            return;
    }
}

void XsensMtiCanParser::parse_sample_time(
    const std::uint8_t* data,
    std::int64_t host_receive_timestamp_ns)
{
    sample_timestamp_us_ =
        static_cast<std::uint64_t>(read_u32_be(data)) * kSampleTimeTickUs;
    sample_receive_timestamp_ns_ = host_receive_timestamp_ns;
    sample_time_fresh_ = true;
}

void XsensMtiCanParser::parse_quaternion(
    const std::uint8_t* data,
    std::int64_t)
{
    ahrs_data_.qw =
        static_cast<float>(static_cast<double>(read_i16_be(data, 0)) * kQuaternionScale);
    ahrs_data_.qx =
        static_cast<float>(static_cast<double>(read_i16_be(data, 2)) * kQuaternionScale);
    ahrs_data_.qy =
        static_cast<float>(static_cast<double>(read_i16_be(data, 4)) * kQuaternionScale);
    ahrs_data_.qz =
        static_cast<float>(static_cast<double>(read_i16_be(data, 6)) * kQuaternionScale);
    quaternion_fresh_ = true;
    update_orientation_from_quaternion();
}

void XsensMtiCanParser::parse_rate_of_turn(
    const std::uint8_t* data,
    std::int64_t)
{
    ahrs_data_.roll_speed =
        static_cast<float>(static_cast<double>(read_i16_be(data, 0)) * kRateOfTurnScale);
    ahrs_data_.pitch_speed =
        static_cast<float>(static_cast<double>(read_i16_be(data, 2)) * kRateOfTurnScale);
    ahrs_data_.heading_speed =
        static_cast<float>(static_cast<double>(read_i16_be(data, 4)) * kRateOfTurnScale);
    rate_of_turn_fresh_ = true;
}

void XsensMtiCanParser::update_orientation_from_quaternion()
{
    const double w = static_cast<double>(ahrs_data_.qw);
    const double x = static_cast<double>(ahrs_data_.qx);
    const double y = static_cast<double>(ahrs_data_.qy);
    const double z = static_cast<double>(ahrs_data_.qz);

    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    const double sinp = 2.0 * (w * y - z * x);
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);

    ahrs_data_.roll = static_cast<float>(std::atan2(sinr_cosp, cosr_cosp));
    ahrs_data_.pitch = static_cast<float>(std::asin(clamp_unit(sinp)));
    ahrs_data_.heading = static_cast<float>(std::atan2(siny_cosp, cosy_cosp));

    ahrs_data_.projected_gravity_x =
        static_cast<float>(2.0 * (w * y - x * z));
    ahrs_data_.projected_gravity_y =
        static_cast<float>(-2.0 * (y * z + w * x));
    ahrs_data_.projected_gravity_z =
        static_cast<float>(2.0 * (x * x + y * y) - 1.0);

    ahrs_data_.projected_gravity_valid =
        std::isfinite(ahrs_data_.projected_gravity_x) &&
        std::isfinite(ahrs_data_.projected_gravity_y) &&
        std::isfinite(ahrs_data_.projected_gravity_z);
}

void XsensMtiCanParser::publish_ahrs_if_ready()
{
    if (!quaternion_fresh_ || !rate_of_turn_fresh_) {
        return;
    }

    if (sample_time_fresh_) {
        ahrs_data_.timestamp = sample_timestamp_us_;
        ahrs_data_.timestamp_valid = true;
        ahrs_data_.host_receive_timestamp_ns = sample_receive_timestamp_ns_;
    } else {
        ahrs_data_.timestamp = 0;
        ahrs_data_.timestamp_valid = false;
        ahrs_data_.host_receive_timestamp_ns = 0;
    }
    ahrs_data_.host_publish_timestamp_ns = robot_base::monotonic_now_ns();

    ahrs_ready_ = true;
    stats_.ahrs_frames++;
    if (ahrs_callback_) {
        ahrs_callback_(ahrs_data_);
    }

    sample_time_fresh_ = false;
    quaternion_fresh_ = false;
    rate_of_turn_fresh_ = false;
}

bool XsensMtiCanParser::get_ahrs_data(imu_base::AHRSData& ahrs)
{
    if (ahrs_ready_) {
        ahrs = ahrs_data_;
        ahrs_ready_ = false;
        return true;
    }
    return false;
}

void XsensMtiCanParser::print_ahrs_data(const imu_base::AHRSData& ahrs)
{
    std::cout << "========= XSENS MTI AHRS Data =========" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Roll/Pitch/Heading Speed (rad/s): ["
              << ahrs.roll_speed << ", "
              << ahrs.pitch_speed << ", "
              << ahrs.heading_speed << "]" << std::endl;
    std::cout << "Roll/Pitch/Heading (rad): ["
              << ahrs.roll << ", "
              << ahrs.pitch << ", "
              << ahrs.heading << "]" << std::endl;
    std::cout << "Quaternion [wxyz]: ["
              << ahrs.qw << ", "
              << ahrs.qx << ", "
              << ahrs.qy << ", "
              << ahrs.qz << "]" << std::endl;
    std::cout << "Projected gravity: ["
              << ahrs.projected_gravity_x << ", "
              << ahrs.projected_gravity_y << ", "
              << ahrs.projected_gravity_z << "]"
              << " valid=" << (ahrs.projected_gravity_valid ? "true" : "false")
              << std::endl;
    std::cout << "Sample timestamp: " << ahrs.timestamp << " us" << std::endl;
    std::cout << "=======================================" << std::endl << std::endl;
}

}  // namespace imu
