#include "robot/robot_imu_session.hpp"

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
        inference::ImuConfig input;
        input.type = imu_base::ReaderType::XSENS_MTI_CAN;
        input.device = "can1";
        input.baudrate = 115200;
        input.configure_can = false;
        input.can_bitrate = 500000;
        input.print_imu = true;
        input.print_ahrs = true;

        const imu_base::ReaderConfig output =
            inference::make_reader_config(input);

        expect(output.type == input.type, "type must be forwarded");
        expect(output.device == input.device, "device must be forwarded");
        expect(output.baudrate == input.baudrate, "baudrate must be forwarded");
        expect(output.configure_can == input.configure_can,
               "configure_can must be forwarded");
        expect(output.can_bitrate == input.can_bitrate,
               "can_bitrate must be forwarded");
        expect(output.print_imu == input.print_imu,
               "print_imu must be forwarded");
        expect(output.print_ahrs == input.print_ahrs,
               "print_ahrs must be forwarded");

        inference::ImuConfig defaults;
        expect(defaults.configure_can, "ImuConfig configure_can default");
        expect(defaults.can_bitrate == 250000, "ImuConfig can_bitrate default");
    } catch (const std::exception& error) {
        std::cerr << "[ROBOT_IMU_SESSION_CONFIG_TEST] "
                  << error.what() << "\n";
        return 1;
    }

    std::cout << "robot_imu_session_config_test passed\n";
    return 0;
}
