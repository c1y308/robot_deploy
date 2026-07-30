#include "xbox_controller.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

namespace {

std::atomic<bool> g_stop_requested{false};

void signal_handler(int)
{
    g_stop_requested.store(true);
}

bool command_changed(const xbox_control::VelocityCommand& lhs,
                     const xbox_control::VelocityCommand& rhs)
{
    return lhs.raw_abs_x != rhs.raw_abs_x ||
           lhs.raw_abs_y != rhs.raw_abs_y ||
           lhs.vx != rhs.vx ||
           lhs.vy != rhs.vy ||
           lhs.yaw_rate != rhs.yaw_rate ||
           lhs.has_abs_x != rhs.has_abs_x ||
           lhs.has_abs_y != rhs.has_abs_y;
}

void print_command(const xbox_control::VelocityCommand& command)
{
    std::cout << std::fixed << std::setprecision(3)
              << "ABS_X=" << std::setw(7) << command.raw_abs_x
              << " ABS_Y=" << std::setw(7) << command.raw_abs_y
              << " vx=" << std::setw(7) << command.vx
              << " vy=" << std::setw(7) << command.vy
              << " yaw_rate=" << std::setw(7) << command.yaw_rate
              << '\n';
}

}  // namespace

int main()
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    xbox_control::XboxController controller;
    if (!controller.open_device()) {
        std::cerr << "[ERROR] " << controller.last_error() << "\n";
        return 1;
    }

    std::cout << "[INFO] Reading Xbox controller from "
              << controller.device_path() << ". Press Ctrl+C to stop.\n";

    if (!controller.start_polling(std::chrono::milliseconds(20))) {
        std::cerr << "[ERROR] " << controller.last_error() << "\n";
        return 1;
    }

    using Clock = std::chrono::steady_clock;
    auto last_print = Clock::now() - std::chrono::seconds(1);
    xbox_control::VelocityCommand last_printed;

    while (!g_stop_requested.load()) {
        xbox_control::VelocityCommand command;
        if (!controller.latest_command(command)) {
            std::cerr << "[ERROR] " << controller.last_error() << "\n";
            controller.stop_polling();
            return 1;
        }

        const auto now = Clock::now();
        if (command_changed(command, last_printed) ||
            now - last_print >= std::chrono::milliseconds(500)) {
            print_command(command);
            last_printed = command;
            last_print = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    controller.stop_polling();
    std::cout << "[INFO] xbox_input_test stopped.\n";
    return 0;
}
