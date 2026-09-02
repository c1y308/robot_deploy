#include "driver/socket_can_config.hpp"
#include "imu_base/imu_base.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void expect(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void test_reader_config_defaults()
{
    imu_base::ReaderConfig cfg;
    expect(cfg.configure_can, "ReaderConfig configure_can default");
    expect(cfg.can_bitrate == 250000, "ReaderConfig can_bitrate default");
}

void test_validate_socket_can_config()
{
    std::string error;
    expect(imu::validate_socket_can_config("can0", 250000, &error),
           "valid SocketCAN config");

    error.clear();
    expect(!imu::validate_socket_can_config("", 250000, &error),
           "empty interface name must fail");
    expect(error.find("empty") != std::string::npos,
           "empty interface error should be clear");

    error.clear();
    expect(!imu::validate_socket_can_config(std::string(128, 'x'),
                                            250000,
                                            &error),
           "long interface name must fail");
    expect(error.find("too long") != std::string::npos,
           "long interface error should be clear");

    error.clear();
    expect(!imu::validate_socket_can_config("can0", 0, &error),
           "zero bitrate must fail");
    expect(error.find("positive") != std::string::npos,
           "zero bitrate error should be clear");

    error.clear();
    expect(!imu::validate_socket_can_config("can0", -1, &error),
           "negative bitrate must fail");
    expect(error.find("positive") != std::string::npos,
           "negative bitrate error should be clear");
}

}  // namespace

int main()
{
    try {
        test_reader_config_defaults();
        test_validate_socket_can_config();
    } catch (const std::exception& error) {
        std::cerr << "[SOCKET_CAN_CONFIG_TEST] " << error.what() << "\n";
        return 1;
    }

    std::cout << "socket_can_config_test passed\n";
    return 0;
}
