#include "xbox_controller.hpp"

#include <linux/input.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

namespace xbox_control {
namespace {

std::string errno_message(const std::string& prefix)
{
    std::ostringstream stream;
    stream << prefix << ": " << std::strerror(errno);
    return stream.str();
}

double positive_axis_scale(int raw_value)
{
    const double usable_range =
        static_cast<double>(kAxisMax - kDeadzone);
    return static_cast<double>(raw_value - kDeadzone) / usable_range;
}

double negative_axis_scale(int raw_value)
{
    const double usable_range =
        static_cast<double>(std::abs(kAxisMin) - kDeadzone);
    return static_cast<double>(raw_value + kDeadzone) / usable_range;
}

}  // namespace

XboxController::XboxController() = default;

XboxController::~XboxController()
{
    close_device();
}

bool XboxController::open_device()
{
    if (is_open()) {
        return true;
    }

    fd_ = ::open(device_path_.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
        last_error_ = errno_message("open " + device_path_ + " failed");
        return false;
    }

    last_error_.clear();
    initialize_axis_values();
    refresh_command();
    return true;
}

void XboxController::close_device()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool XboxController::is_open() const
{
    return fd_ >= 0;
}

bool XboxController::poll(VelocityCommand& command)
{
    if (!is_open()) {
        last_error_ = "controller device is not open";
        return false;
    }

    while (true) {
        input_event event = {};
        const ssize_t bytes = ::read(fd_, &event, sizeof(event));

        if (bytes == static_cast<ssize_t>(sizeof(event))) {
            if (event.type == EV_ABS) {
                if (event.code == ABS_X) {
                    command_.raw_abs_x = event.value;
                    command_.has_abs_x = true;
                    refresh_command();
                } else if (event.code == ABS_Y) {
                    command_.raw_abs_y = event.value;
                    command_.has_abs_y = true;
                    refresh_command();
                }
            }
            continue;
        }

        if (bytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                command = command_;
                last_error_.clear();
                return true;
            }
            if (errno == EINTR) {
                continue;
            }
            last_error_ = errno_message("read " + device_path_ + " failed");
            return false;
        }

        if (bytes == 0) {
            last_error_ = "controller device returned EOF";
            return false;
        }

        last_error_ = "short read from controller device";
        return false;
    }
}

double XboxController::axis_to_speed(int raw_value)
{
    double normalized = 0.0;
    if (raw_value > kDeadzone) {
        normalized = positive_axis_scale(raw_value);
    } else if (raw_value < -kDeadzone) {
        normalized = negative_axis_scale(raw_value);
    }

    normalized = std::clamp(normalized, -1.0, 1.0);
    return normalized * kMaxSpeedMps;
}

VelocityCommand XboxController::map_axes(int abs_x, int abs_y)
{
    VelocityCommand command;
    command.raw_abs_x = abs_x;
    command.raw_abs_y = abs_y;
    command.has_abs_x = true;
    command.has_abs_y = true;
    command.vx = std::clamp(-axis_to_speed(abs_y), -kMaxSpeedMps, kMaxSpeedMps);
    command.vy = std::clamp(axis_to_speed(abs_x), -kMaxSpeedMps, kMaxSpeedMps);
    command.yaw_rate = kYawRate;
    return command;
}

void XboxController::initialize_axis_values()
{
    input_absinfo abs_info = {};
    if (::ioctl(fd_, EVIOCGABS(ABS_X), &abs_info) == 0) {
        command_.raw_abs_x = abs_info.value;
        command_.has_abs_x = true;
    }

    abs_info = {};
    if (::ioctl(fd_, EVIOCGABS(ABS_Y), &abs_info) == 0) {
        command_.raw_abs_y = abs_info.value;
        command_.has_abs_y = true;
    }
}

void XboxController::refresh_command()
{
    command_.vx = std::clamp(-axis_to_speed(command_.raw_abs_y),
                             -kMaxSpeedMps,
                             kMaxSpeedMps);
    command_.vy = std::clamp(axis_to_speed(command_.raw_abs_x),
                             -kMaxSpeedMps,
                             kMaxSpeedMps);
    command_.yaw_rate = kYawRate;
}

}  // namespace xbox_control
