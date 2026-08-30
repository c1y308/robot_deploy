#include "protocol/xsens_mti/can_parser.hpp"

#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {

void expect(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool near(double lhs, double rhs)
{
    return std::fabs(lhs - rhs) < 1.0e-6;
}

}  // namespace

int main()
{
    try {
        imu::XsensMtiCanParser parser;
        int callback_count = 0;
        imu_base::AHRSData callback_data;
        parser.set_ahrs_callback([&](const imu_base::AHRSData& data) {
            ++callback_count;
            callback_data = data;
        });

        const std::uint8_t sample_time[] = {0x00, 0x00, 0x00, 0x0A};
        constexpr std::int64_t kSampleReceiveTimestampNs = 123000;
        parser.feed(imu::XCDI_SAMPLE_TIME_ID,
                    sample_time,
                    sizeof(sample_time),
                    kSampleReceiveTimestampNs);

        const std::uint8_t identity_quaternion[] = {
            0x7F, 0xFF,
            0x00, 0x00,
            0x00, 0x00,
            0x00, 0x00,
        };
        parser.feed(imu::XCDI_QUATERNION_ID,
                    identity_quaternion,
                    sizeof(identity_quaternion),
                    124000);

        imu_base::AHRSData ahrs;
        expect(callback_count == 0, "quaternion alone must not publish AHRS");
        expect(!parser.get_ahrs_data(ahrs), "quaternion alone must not be ready");

        const std::uint8_t rate_of_turn[] = {
            0x02, 0x00,
            0x00, 0x00,
            0x00, 0x00,
        };
        parser.feed(imu::XCDI_RATE_OF_TURN_ID,
                    rate_of_turn,
                    sizeof(rate_of_turn),
                    125000);

        expect(callback_count == 1, "quaternion + rate must publish AHRS");
        expect(parser.get_ahrs_data(ahrs), "quaternion + rate must be ready");
        expect(callback_data.timestamp == 1000, "sample time must be ticks * 100 us");
        expect(callback_data.timestamp_valid, "fresh sample time must mark timestamp valid");
        expect(callback_data.host_receive_timestamp_ns == kSampleReceiveTimestampNs,
               "AHRS receive timestamp must come from SampleTime frame");
        expect(callback_data.host_publish_timestamp_ns != 0,
               "AHRS publish timestamp must be populated");
        expect(near(ahrs.roll_speed, 1.0), "0x0200 rate must be 1 rad/s");
        expect(near(ahrs.pitch_speed, 0.0), "pitch rate must be zero");
        expect(near(ahrs.heading_speed, 0.0), "heading rate must be zero");
        expect(near(ahrs.qw, 1.0), "identity quaternion qw must be 1");
        expect(near(ahrs.qx, 0.0), "identity quaternion qx must be 0");
        expect(near(ahrs.qy, 0.0), "identity quaternion qy must be 0");
        expect(near(ahrs.qz, 0.0), "identity quaternion qz must be 0");
        expect(near(ahrs.roll, 0.0), "identity quaternion roll must be 0");
        expect(near(ahrs.pitch, 0.0), "identity quaternion pitch must be 0");
        expect(near(ahrs.heading, 0.0), "identity quaternion heading must be 0");
        expect(ahrs.projected_gravity_valid, "projected gravity must be valid");
        expect(near(ahrs.projected_gravity_x, 0.0), "projected gravity x must be 0");
        expect(near(ahrs.projected_gravity_y, 0.0), "projected gravity y must be 0");
        expect(near(ahrs.projected_gravity_z, -1.0), "projected gravity z must be -1");

        parser.feed(imu::XCDI_QUATERNION_ID,
                    identity_quaternion,
                    sizeof(identity_quaternion),
                    126000);
        parser.feed(imu::XCDI_RATE_OF_TURN_ID,
                    rate_of_turn,
                    sizeof(rate_of_turn),
                    127000);
        expect(callback_count == 2,
               "fresh quaternion + rate without SampleTime must still publish AHRS");
        expect(parser.get_ahrs_data(ahrs),
               "AHRS without fresh SampleTime must be readable");
        expect(!ahrs.timestamp_valid,
               "old SampleTime must not be reused for the next AHRS group");
        expect(ahrs.host_receive_timestamp_ns == 0,
               "invalid device timestamp must not expose timestamp-associated RX time");
        expect(ahrs.host_publish_timestamp_ns != 0,
               "AHRS without SampleTime must still expose publish timestamp");

        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "[XSENS_MTI_CAN_PARSER_TEST] " << ex.what() << "\n";
        return 1;
    }
}
