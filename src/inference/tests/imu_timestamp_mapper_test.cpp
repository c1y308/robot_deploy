#include "robot/imu_timestamp_mapper.hpp"

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

}  // namespace

int main()
{
    try {
        inference::ImuTimestampMapper a100_mapper(
            imu_base::ReaderType::A100_SERIAL);

        std::int64_t sample_ns = 0;
        expect(a100_mapper.map_device_timestamp_us(1000, 1000000, sample_ns),
               "first A100 timestamp must map");
        expect(sample_ns == 1000000,
               "first mapped timestamp should establish receive offset");

        expect(a100_mapper.map_device_timestamp_us(2000, 2200000, sample_ns),
               "later A100 timestamp must map");
        expect(sample_ns == 2000000,
               "larger receive delay must not move min offset");

        expect(a100_mapper.map_device_timestamp_us(3000, 2900000, sample_ns),
               "lower receive delay must update min offset");
        expect(sample_ns == 2900000,
               "minimum receive offset must pull sample time earlier");

        expect(a100_mapper.map_device_timestamp_us(3000, 3500000, sample_ns),
               "duplicate A100 timestamp must still map");
        expect(sample_ns == 2900001,
               "duplicate timestamp must be monotonic-clamped");

        expect(!a100_mapper.map_device_timestamp_us(1000, 4000000, sample_ns),
               "backward A100 timestamp must be treated as reset");
        expect(sample_ns == 0,
               "reset frame must not produce a mapped timestamp");
        expect(a100_mapper.map_device_timestamp_us(1100, 4100000, sample_ns),
               "A100 mapper must recover after reset");
        expect(sample_ns == 4100000,
               "first frame after reset must rebuild offset");

        inference::ImuTimestampMapper xsens_mapper(
            imu_base::ReaderType::XSENS_MTI_CAN);
        constexpr std::uint64_t kXsensWrapUs = (1ULL << 32) * 100ULL;
        std::int64_t sample_before_wrap_ns = 0;
        expect(xsens_mapper.map_device_timestamp_us(kXsensWrapUs - 200,
                                                    500000000000000,
                                                    sample_before_wrap_ns),
               "Xsens timestamp before wrap must map");
        std::int64_t sample_after_wrap_ns = 0;
        expect(xsens_mapper.map_device_timestamp_us(100,
                                                    500000000300000,
                                                    sample_after_wrap_ns),
               "Xsens timestamp after wrap must map");
        expect(sample_after_wrap_ns > sample_before_wrap_ns,
               "Xsens wrap must preserve monotonic mapped time");

        inference::ImuTimestampMapper xsens_reset_mapper(
            imu_base::ReaderType::XSENS_MTI_CAN);
        expect(xsens_reset_mapper.map_device_timestamp_us(10000,
                                                         100000000,
                                                         sample_ns),
               "first Xsens timestamp must map");
        expect(!xsens_reset_mapper.map_device_timestamp_us(9000,
                                                          101000000,
                                                          sample_ns),
               "small backward Xsens timestamp must be treated as reset");

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[IMU_TIMESTAMP_MAPPER_TEST] " << error.what() << "\n";
        return 1;
    }
}
