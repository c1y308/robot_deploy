#pragma once

#include <string>

namespace xbox_control {

constexpr const char* kXboxDevicePath = "/dev/input/event7";
constexpr int kAxisMin = -32768;
constexpr int kAxisMax = 32767;
constexpr int kDeadzone = 3000;
constexpr double kMaxSpeedMps = 0.3;
constexpr double kYawRate = 0.0;

struct VelocityCommand {
    int raw_abs_x = 0;
    int raw_abs_y = 0;
    bool has_abs_x = false;
    bool has_abs_y = false;
    double vx = 0.0;
    double vy = 0.0;
    double yaw_rate = kYawRate;
};

class XboxController {
public:
    XboxController();
    ~XboxController();

    XboxController(const XboxController&) = delete;
    XboxController& operator=(const XboxController&) = delete;

    bool open_device();
    void close_device();
    bool is_open() const;

    bool poll(VelocityCommand& command);
    const VelocityCommand& command() const { return command_; }
    const std::string& device_path() const { return device_path_; }
    const std::string& last_error() const { return last_error_; }

    static double axis_to_speed(int raw_value);
    static VelocityCommand map_axes(int abs_x, int abs_y);

private:
    void initialize_axis_values();
    void refresh_command();

    int fd_ = -1;
    std::string device_path_ = kXboxDevicePath;
    std::string last_error_;
    VelocityCommand command_;
};

}  // namespace xbox_control
