#pragma once

#include <string>

namespace imu {

bool validate_socket_can_config(const std::string& interface_name,
                                int bitrate,
                                std::string* error = nullptr);

bool configure_socket_can_interface(const std::string& interface_name,
                                    int bitrate,
                                    std::string* error = nullptr);

}  // namespace imu
