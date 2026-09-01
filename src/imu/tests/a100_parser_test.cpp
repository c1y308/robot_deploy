#include "protocol/a100/imu_parser.hpp"

#include <cstdint>
#include <cstring>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

void expect(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::uint8_t crc8(const std::vector<std::uint8_t>& data)
{
    std::uint8_t crc = 0;
    for (std::uint8_t value : data) {
        crc = imu::CRC8Table[crc ^ value];
    }
    return crc;
}

std::uint16_t crc16(const std::vector<std::uint8_t>& data)
{
    std::uint16_t crc = 0;
    for (std::uint8_t value : data) {
        crc = static_cast<std::uint16_t>(
            (crc << 8) ^ imu::CRC16Table[((crc >> 8) ^ value) & 0xFF]);
    }
    return crc;
}

void write_u32_le(std::vector<std::uint8_t>& frame,
                  std::size_t offset,
                  std::uint32_t value)
{
    frame[offset] = static_cast<std::uint8_t>(value & 0xFFU);
    frame[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFFU);
    frame[offset + 2] = static_cast<std::uint8_t>((value >> 16) & 0xFFU);
    frame[offset + 3] = static_cast<std::uint8_t>((value >> 24) & 0xFFU);
}

void write_u64_le(std::vector<std::uint8_t>& frame,
                  std::size_t offset,
                  std::uint64_t value)
{
    for (std::size_t i = 0; i < 8; ++i) {
        frame[offset + i] =
            static_cast<std::uint8_t>((value >> (8U * i)) & 0xFFU);
    }
}

void finalize_frame(std::vector<std::uint8_t>& frame,
                    std::size_t payload_length)
{
    frame[0] = imu::FRAME_HEAD;
    frame[2] = static_cast<std::uint8_t>(payload_length);
    frame[3] = 0;
    frame[4] = crc8(std::vector<std::uint8_t>(frame.begin(),
                                              frame.begin() + 4));

    const std::vector<std::uint8_t> payload(frame.begin() + 7,
                                            frame.begin() + 7 + payload_length);
    const std::uint16_t frame_crc = crc16(payload);
    frame[5] = static_cast<std::uint8_t>((frame_crc >> 8) & 0xFFU);
    frame[6] = static_cast<std::uint8_t>(frame_crc & 0xFFU);
    frame.back() = imu::FRAME_END;
}

std::vector<std::uint8_t> make_ahrs_frame(std::uint64_t timestamp_us)
{
    std::vector<std::uint8_t> frame(imu::AHRS_FRAME_SIZE, 0);
    frame[1] = imu::TYPE_AHRS;
    write_u64_le(frame, 47, timestamp_us);
    finalize_frame(frame, imu::AHRS_LEN);
    return frame;
}

std::vector<std::uint8_t> make_imu_frame(std::uint32_t timestamp_us)
{
    std::vector<std::uint8_t> frame(imu::IMU_FRAME_SIZE, 0);
    frame[1] = imu::TYPE_IMU;
    write_u32_le(frame, 55, timestamp_us);
    finalize_frame(frame, imu::IMU_LEN);
    return frame;
}

void feed_frame(imu::IMUParser& parser,
                const std::vector<std::uint8_t>& frame,
                std::int64_t receive_timestamp_ns)
{
    const std::uint8_t previous_end = imu::FRAME_END;
    parser.feed(&previous_end, 1, receive_timestamp_ns - 1);
    parser.feed(frame.data(),
                static_cast<int>(frame.size()),
                receive_timestamp_ns);
}

}  // namespace

int main()
{
    try {
        imu::IMUParser parser;
        imu::AHRSData_t callback_ahrs;
        imu::IMUData_t callback_imu;
        int ahrs_callbacks = 0;
        int imu_callbacks = 0;
        parser.set_ahrs_callback([&](const imu::AHRSData_t& data) {
            ++ahrs_callbacks;
            callback_ahrs = data;
        });
        parser.set_imu_callback([&](const imu::IMUData_t& data) {
            ++imu_callbacks;
            callback_imu = data;
        });

        constexpr std::uint64_t kAhrsTimestampUs = 123456789ULL;
        constexpr std::int64_t kAhrsReceiveTimestampNs = 9876543210LL;
        feed_frame(parser,
                   make_ahrs_frame(kAhrsTimestampUs),
                   kAhrsReceiveTimestampNs);

        expect(ahrs_callbacks == 1, "A100 AHRS callback must fire once");
        expect(callback_ahrs.sample_timestamp_ns == kAhrsTimestampUs * 1000ULL,
               "A100 AHRS sample timestamp must be converted to ns");
        expect(callback_ahrs.receive_timestamp_ns ==
                   kAhrsReceiveTimestampNs,
               "A100 AHRS receive timestamp must use completing chunk");

        imu::AHRSData_t latest_ahrs;
        expect(parser.get_ahrs_data(latest_ahrs),
               "A100 AHRS data must be readable after parse");
        expect(latest_ahrs.sample_timestamp_ns == kAhrsTimestampUs * 1000ULL,
               "A100 AHRS readable data must retain sample timestamp");

        std::vector<std::uint8_t> marker_like_payload_frame =
            make_ahrs_frame(kAhrsTimestampUs + 1ULL);
        marker_like_payload_frame[20] = imu::FRAME_END;
        marker_like_payload_frame[21] = imu::FRAME_HEAD;
        finalize_frame(marker_like_payload_frame, imu::AHRS_LEN);
        feed_frame(parser,
                   marker_like_payload_frame,
                   kAhrsReceiveTimestampNs + 1000);
        expect(ahrs_callbacks == 2,
               "A100 parser must not resync on FD FC inside a valid payload");
        expect(callback_ahrs.sample_timestamp_ns ==
                   (kAhrsTimestampUs + 1ULL) * 1000ULL,
               "A100 AHRS frame with marker-like payload bytes must parse sample timestamp");

        constexpr std::uint32_t kImuTimestampUs = 424242U;
        constexpr std::int64_t kImuReceiveTimestampNs = 9876549999LL;
        feed_frame(parser,
                   make_imu_frame(kImuTimestampUs),
                   kImuReceiveTimestampNs);

        expect(imu_callbacks == 1, "A100 IMU callback must fire once");
        expect(callback_imu.sample_timestamp_ns ==
                   static_cast<std::uint64_t>(kImuTimestampUs) * 1000ULL,
               "A100 raw IMU sample timestamp must be converted to ns");
        expect(callback_imu.receive_timestamp_ns == kImuReceiveTimestampNs,
               "A100 raw IMU receive timestamp must use completing chunk");

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[A100_PARSER_TEST] " << error.what() << "\n";
        return 1;
    }
}
