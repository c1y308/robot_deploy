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

const std::uint8_t kIdentity[] = {0x7F, 0xFF, 0, 0, 0, 0, 0, 0};
const std::uint8_t kZRotation[] = {0, 0, 0, 0, 0, 0, 0x7F, 0xFF};
const std::uint8_t kRateOne[] = {0x02, 0, 0, 0, 0, 0};
const std::uint8_t kRateTwo[] = {0x04, 0, 0, 0, 0, 0};

void test_latest_pairing()
{
    imu::XsensMtiCanParser parser;
    int callbacks = 0;
    imu_base::AHRSData published, fetched;
    parser.set_ahrs_callback([&](const imu_base::AHRSData& data) {
        ++callbacks;
        published = data;
    });
    parser.feed(imu::XCDI_QUATERNION_ID, kZRotation, 8, 100);
    parser.feed(imu::XCDI_QUATERNION_ID, kZRotation, 8, 200);
    parser.feed(imu::XCDI_QUATERNION_ID, kIdentity, 8, 300);
    expect(callbacks == 0, "repeated Q alone must not publish");
    parser.feed(imu::XCDI_RATE_OF_TURN_ID, kRateOne, 6, 400);
    expect(callbacks == 1 && near(published.qw, 1) &&
               published.receive_timestamp_ns == 300,
           "Q Q Q R must publish the latest Q and its older timestamp");
    parser.feed(imu::XCDI_RATE_OF_TURN_ID, kRateOne, 6, 500);
    parser.feed(imu::XCDI_RATE_OF_TURN_ID, kRateTwo, 6, 600);
    expect(callbacks == 1, "each publication must require both fresh fields");
    parser.feed(imu::XCDI_QUATERNION_ID, kZRotation, 8, 700);
    expect(callbacks == 2 && near(published.qz, 1) &&
               near(published.roll_speed, 2) && published.receive_timestamp_ns == 600,
           "R R Q must publish latest R and its older timestamp");
    expect(parser.get_ahrs_data(fetched) && fetched.receive_timestamp_ns == 600 &&
               near(fetched.qz, published.qz) && !parser.get_ahrs_data(fetched),
           "getter must consume the same latest complete snapshot once");
    parser.feed(imu::XCDI_QUATERNION_ID, kIdentity, 8, 1'000'000);
    parser.feed(imu::XCDI_RATE_OF_TURN_ID, kRateOne, 6, 101'000'000);
    expect(published.receive_timestamp_ns == 1'000'000,
           "a new rate must not hide the age of an old quaternion");
    parser.feed(imu::XCDI_RATE_OF_TURN_ID, kRateOne, 6, 102'000'000);
    parser.feed(imu::XCDI_QUATERNION_ID, kIdentity, 8, 202'000'000);
    expect(published.receive_timestamp_ns == 102'000'000 && callbacks == 4 &&
               parser.get_info().ahrs_frames == 4,
           "a new quaternion must not hide the age of an old rate");
}

void test_snapshot_isolation_and_invalid_quaternion()
{
    imu::XsensMtiCanParser parser;
    int callbacks = 0;
    parser.set_ahrs_callback([&](const imu_base::AHRSData&) { ++callbacks; });
    const std::uint8_t sample[] = {0, 0, 0, 10};
    const std::uint8_t next_sample[] = {0, 0, 0, 20};
    const std::uint8_t zero[] = {0, 0, 0, 0, 0, 0, 0, 0};
    const std::uint8_t low_norm[] = {0x3F, 0xFF, 0, 0, 0, 0, 0, 0};
    parser.feed(imu::XCDI_SAMPLE_TIME_ID, sample, 4, 5);
    parser.feed(imu::XCDI_QUATERNION_ID, kIdentity, 8, 10);
    parser.feed(imu::XCDI_RATE_OF_TURN_ID, kRateOne, 6, 20);
    parser.feed(imu::XCDI_QUATERNION_ID, kZRotation, 8, 30);
    parser.feed(imu::XCDI_SAMPLE_TIME_ID, next_sample, 4, 40);
    imu_base::AHRSData data;
    expect(parser.get_ahrs_data(data) && near(data.qw, 1) &&
               near(data.roll_speed, 1) && data.receive_timestamp_ns == 10 &&
               data.sample_timestamp_ns == 1'000'000,
           "half updates and metadata must not alter an unread complete snapshot");
    parser.feed(imu::XCDI_RATE_OF_TURN_ID, kRateTwo, 6, 50);
    parser.feed(imu::XCDI_QUATERNION_ID, zero, 8, 60);
    expect(callbacks == 2 && parser.get_info().error_frames == 1 &&
               parser.get_info().ahrs_frames == 2 && parser.get_info().total_frames == 7,
           "zero quaternion must count an error without publication");
    expect(parser.get_ahrs_data(data) && near(data.qz, 1) &&
               near(data.roll_speed, 2) && data.receive_timestamp_ns == 30 &&
               data.sample_timestamp_ns == 2'000'000 && data.projected_gravity_valid,
           "invalid quaternion must preserve the previous snapshot and ready state");
    parser.feed(imu::XCDI_RATE_OF_TURN_ID, kRateOne, 6, 70);
    parser.feed(imu::XCDI_QUATERNION_ID, low_norm, 8, 80);
    parser.feed(imu::XCDI_QUATERNION_ID, kIdentity, 8, 90);
    expect(callbacks == 2 && !parser.get_ahrs_data(data) &&
               parser.get_info().error_frames == 2,
           "low norm must discard both fresh fields, including a pending rate");
    parser.feed(imu::XCDI_RATE_OF_TURN_ID, kRateTwo, 6, 100);
    expect(callbacks == 3 && parser.get_ahrs_data(data) && near(data.qw, 1) &&
               near(data.roll_speed, 2) && data.receive_timestamp_ns == 90,
           "valid Q and a new rate must restore publication");
    parser.feed(imu::XCDI_QUATERNION_ID, kIdentity, 8, 110);
    parser.reset();
    parser.feed(imu::XCDI_RATE_OF_TURN_ID, kRateOne, 6, 120);
    expect(!parser.get_ahrs_data(data) && parser.get_info().ahrs_frames == 0 &&
               parser.get_info().error_frames == 0,
           "reset must clear pending, ready, and statistics");
    parser.feed(imu::XCDI_QUATERNION_ID, kIdentity, 8, 130);
    expect(parser.get_ahrs_data(data) && data.receive_timestamp_ns == 120 &&
               data.sample_timestamp_ns == 0,
           "reset must clear per-field timestamps and metadata");
}

void test_sample_time_is_metadata()
{
    imu::XsensMtiCanParser parser;
    const std::uint8_t maximum[] = {0xFF, 0xFF, 0xFF, 0xFF};
    const std::uint8_t zero[] = {0, 0, 0, 0};
    parser.feed(imu::XCDI_RATE_OF_TURN_ID, kRateOne, 6, 10);
    parser.feed(imu::XCDI_SAMPLE_TIME_ID, maximum, 4, 15);
    parser.feed(imu::XCDI_QUATERNION_ID, kIdentity, 8, 20);
    imu_base::AHRSData data;
    expect(parser.get_ahrs_data(data) && data.receive_timestamp_ns == 10 &&
               data.sample_timestamp_ns == 0xFFFFFFFFULL * 100000,
           "SampleTime must not interrupt pending latest-value pairing");
    parser.feed(imu::XCDI_QUATERNION_ID, kIdentity, 8, 30);
    parser.feed(imu::XCDI_SAMPLE_TIME_ID, zero, 4, 35);
    parser.feed(imu::XCDI_SAMPLE_TIME_ID, zero, 4, 36);
    parser.feed(imu::XCDI_RATE_OF_TURN_ID, kRateOne, 6, 40);
    expect(parser.get_ahrs_data(data) && data.receive_timestamp_ns == 30 &&
               data.sample_timestamp_ns == 0,
           "zero, wraparound, and repeated SampleTime must not block publication");
}

void test_captured_interleaving()
{
    // Real can0 excerpt: 005, 032, 021, other outputs, 032, 005, 021.
    const std::uint8_t sample[] = {0x03, 0x32, 0xFF, 0xB5};
    const std::uint8_t next_sample[] = {0x03, 0x32, 0xFF, 0xCE};
    const std::uint8_t quaternion[] = {0x7D, 0xF4, 0xFF, 0x6F, 0xFD, 0x95, 0x16, 0xA4};
    const std::uint8_t next_quaternion[] = {0x7D, 0xF4, 0xFF, 0x6F, 0xFD, 0x95, 0x16, 0xA3};
    const std::uint8_t rate_before[] = {0, 1, 0xFF, 0xFF, 0, 1};
    const std::uint8_t rate_after[] = {0, 2, 0, 1, 0, 1};
    imu::XsensMtiCanParser parser;
    parser.feed(imu::XCDI_SAMPLE_TIME_ID, sample, 4, 771'059'000);
    parser.feed(imu::XCDI_RATE_OF_TURN_ID, rate_before, 6, 771'482'000);
    parser.feed(imu::XCDI_QUATERNION_ID, quaternion, 8, 771'951'000);
    imu_base::AHRSData data;
    expect(parser.get_ahrs_data(data) && data.receive_timestamp_ns == 771'482'000 &&
               near(data.roll_speed, 1.0 / 512.0),
           "captured Rate-before-Q sequence must publish");
    parser.feed(imu::XCDI_RATE_OF_TURN_ID, rate_after, 6, 773'219'000);
    parser.feed(imu::XCDI_SAMPLE_TIME_ID, next_sample, 4, 774'047'000);
    parser.feed(imu::XCDI_QUATERNION_ID, next_quaternion, 8, 774'519'000);
    expect(parser.get_ahrs_data(data) && data.receive_timestamp_ns == 773'219'000 &&
               near(data.roll_speed, 2.0 / 512.0) && parser.get_info().ahrs_frames == 2,
           "captured interleaving must not acquire a SampleTime publication gate");
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
        parser.feed(imu::XCDI_SAMPLE_TIME_ID,
                    sample_time,
                    sizeof(sample_time),
                    123000);

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
        expect(callback_data.sample_timestamp_ns == 1000000,
               "sample time must be ticks * 100000 ns");
        expect(callback_data.receive_timestamp_ns == 124000,
               "AHRS receive timestamp must come from the older field");
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
        expect(ahrs.sample_timestamp_ns == 1000000,
               "latest SampleTime must be reused for the next AHRS group");
        expect(ahrs.receive_timestamp_ns == 126000,
               "AHRS receive timestamp must come from the older field");

        imu::XsensMtiCanParser missing_sample_parser;
        missing_sample_parser.feed(imu::XCDI_QUATERNION_ID,
                                   identity_quaternion,
                                   sizeof(identity_quaternion),
                                   224000);
        missing_sample_parser.feed(imu::XCDI_RATE_OF_TURN_ID,
                                   rate_of_turn,
                                   sizeof(rate_of_turn),
                                   225000);
        expect(missing_sample_parser.get_ahrs_data(ahrs),
               "AHRS without any SampleTime must still be readable");
        expect(ahrs.sample_timestamp_ns == 0,
               "AHRS without any SampleTime must expose zero sample timestamp");
        expect(ahrs.receive_timestamp_ns == 224000,
               "AHRS without SampleTime must still expose receive timestamp");

        test_latest_pairing();
        test_snapshot_isolation_and_invalid_quaternion();
        test_sample_time_is_metadata();
        test_captured_interleaving();
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "[XSENS_MTI_CAN_PARSER_TEST] " << ex.what() << "\n";
        return 1;
    }
}
