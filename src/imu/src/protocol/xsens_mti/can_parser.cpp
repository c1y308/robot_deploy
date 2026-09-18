#include "protocol/xsens_mti/can_parser.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <iomanip>
#include <iostream>

namespace imu {
namespace {

constexpr double kQuaternionScale = 1.0 / 32767.0;
constexpr double kRateOfTurnScale = 1.0 / 512.0;
constexpr std::uint64_t kSampleTimeTickNs = 100000;

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
    clear_pending();
    published_ = imu_base::AHRSData();
    ahrs_ready_ = false;
    sample_timestamp_ns_ = 0;
    stats_ = imu_base::ParserInfo();
}

void XsensMtiCanParser::clear_pending()
{
    pending_ = imu_base::AHRSData();
    quaternion_rx_ns_ = 0;
    rate_rx_ns_ = 0;
    quaternion_fresh_ = false;
    rate_of_turn_fresh_ = false;
}

void XsensMtiCanParser::feed(std::uint32_t       can_id,
                             const std::uint8_t* data,
                             std::uint8_t        len,
                             std::int64_t        receive_timestamp_ns)
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
            parse_sample_time(data);
            stats_.total_frames++;
            return;
        case XCDI_QUATERNION_ID:
            if (len != 8) {
                stats_.error_frames++;
                return;
            }
            parse_quaternion(data, receive_timestamp_ns);
            stats_.total_frames++;
            publish_ahrs_if_ready();
            return;
        case XCDI_RATE_OF_TURN_ID:
            if (len != 6) {
                stats_.error_frames++;
                return;
            }
            parse_rate_of_turn(data, receive_timestamp_ns);
            stats_.total_frames++;
            publish_ahrs_if_ready();
            return;
        default:
            return;
    }
}

void XsensMtiCanParser::parse_sample_time(const std::uint8_t* data)
{
    sample_timestamp_ns_ =
        static_cast<std::uint64_t>(read_u32_be(data)) * kSampleTimeTickNs;
}

void XsensMtiCanParser::parse_quaternion(
    const std::uint8_t* data,
    std::int64_t receive_timestamp_ns)
{
    pending_.qw =
        static_cast<float>(static_cast<double>(read_i16_be(data, 0)) * kQuaternionScale);
    pending_.qx =
        static_cast<float>(static_cast<double>(read_i16_be(data, 2)) * kQuaternionScale);
    pending_.qy =
        static_cast<float>(static_cast<double>(read_i16_be(data, 4)) * kQuaternionScale);
    pending_.qz =
        static_cast<float>(static_cast<double>(read_i16_be(data, 6)) * kQuaternionScale);
    const double qw = pending_.qw, qx = pending_.qx;
    const double qy = pending_.qy, qz = pending_.qz;
    const double norm_sq = qw*qw + qx*qx + qy*qy + qz*qz;
    if (!std::isfinite(norm_sq) || norm_sq <= 0.25) {
        stats_.error_frames++;
        clear_pending();
        return;
    }
    quaternion_rx_ns_ = receive_timestamp_ns;
    quaternion_fresh_ = true;
    update_orientation_from_quaternion();
}

void XsensMtiCanParser::parse_rate_of_turn(
    const std::uint8_t* data,
    std::int64_t receive_timestamp_ns)
{
    pending_.roll_speed =
        static_cast<float>(static_cast<double>(read_i16_be(data, 0)) * kRateOfTurnScale);
    pending_.pitch_speed =
        static_cast<float>(static_cast<double>(read_i16_be(data, 2)) * kRateOfTurnScale);
    pending_.heading_speed =
        static_cast<float>(static_cast<double>(read_i16_be(data, 4)) * kRateOfTurnScale);
    rate_rx_ns_ = receive_timestamp_ns;
    rate_of_turn_fresh_ = true;
}

void XsensMtiCanParser::update_orientation_from_quaternion()
{
    const double w = static_cast<double>(pending_.qw);
    const double x = static_cast<double>(pending_.qx);
    const double y = static_cast<double>(pending_.qy);
    const double z = static_cast<double>(pending_.qz);

    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    const double sinp = 2.0 * (w * y - z * x);
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);

    pending_.roll = static_cast<float>(std::atan2(sinr_cosp, cosr_cosp));
    pending_.pitch = static_cast<float>(std::asin(clamp_unit(sinp)));
    pending_.heading = static_cast<float>(std::atan2(siny_cosp, cosy_cosp));

    pending_.projected_gravity_x =
        static_cast<float>(2.0 * (w * y - x * z));
    pending_.projected_gravity_y =
        static_cast<float>(-2.0 * (y * z + w * x));
    pending_.projected_gravity_z =
        static_cast<float>(2.0 * (x * x + y * y) - 1.0);

    pending_.projected_gravity_valid =
        std::isfinite(pending_.projected_gravity_x) &&
        std::isfinite(pending_.projected_gravity_y) &&
        std::isfinite(pending_.projected_gravity_z);
}

void XsensMtiCanParser::publish_ahrs_if_ready()
{
    if (!quaternion_fresh_ || !rate_of_turn_fresh_) {
        return;
    }

    pending_.sample_timestamp_ns = sample_timestamp_ns_;
    pending_.receive_timestamp_ns = std::min(quaternion_rx_ns_, rate_rx_ns_);
    published_ = pending_;

    ahrs_ready_ = true;
    stats_.ahrs_frames++;
    if (ahrs_callback_) {
        ahrs_callback_(published_);
    }

    quaternion_fresh_ = false;
    rate_of_turn_fresh_ = false;
}

bool XsensMtiCanParser::get_ahrs_data(imu_base::AHRSData& ahrs)
{
    if (ahrs_ready_) {
        ahrs = published_;
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
              << ahrs.roll_speed  << ", "
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
              << ahrs.projected_gravity_z << "] "
              << " valid=" << (ahrs.projected_gravity_valid ? "true" : "false")
              << std::endl;
    std::cout << "Receive timestamp: " << ahrs.receive_timestamp_ns << " ns" << std::endl;
    std::cout << "Sample timestamp: "  << ahrs.sample_timestamp_ns  << " ns" << std::endl;
    std::cout << "=======================================" << std::endl << std::endl;
}

}  // namespace imu
