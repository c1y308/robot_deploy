#include "xbox_controller.hpp"

#include <cmath>
#include <iostream>

namespace {

bool near(double lhs, double rhs)
{
    return std::fabs(lhs - rhs) < 1e-9;
}

bool expect_command(int abs_x,
                    int abs_y,
                    double expected_vx,
                    double expected_vy,
                    double expected_yaw_rate)
{
    const xbox_control::VelocityCommand command =
        xbox_control::XboxController::map_axes(abs_x, abs_y);

    if (!near(command.vx, expected_vx) ||
        !near(command.vy, expected_vy) ||
        !near(command.yaw_rate, expected_yaw_rate)) {
        std::cerr << "map_axes(" << abs_x << ", " << abs_y << ") got vx="
                  << command.vx << " vy=" << command.vy
                  << " yaw_rate=" << command.yaw_rate
                  << ", expected vx=" << expected_vx
                  << " vy=" << expected_vy
                  << " yaw_rate=" << expected_yaw_rate << "\n";
        return false;
    }

    return true;
}

}  // namespace

int main()
{
    bool ok = true;
    ok = expect_command(0, -32768, 0.3, 0.0, 0.0) && ok;
    ok = expect_command(0, -8000, 0.0, 0.0, 0.0) && ok;
    ok = expect_command(0, 0, 0.0, 0.0, 0.0) && ok;
    ok = expect_command(0, 8000, 0.0, 0.0, 0.0) && ok;
    ok = expect_command(0, 32767, -0.3, 0.0, 0.0) && ok;
    ok = expect_command(-32768, 0, 0.0, 0.0, 0.0) && ok;
    ok = expect_command(32767, 0, 0.0, 0.0, 0.0) && ok;

    if (!ok) {
        return 1;
    }

    std::cout << "xbox_controller_mapping_test passed\n";
    return 0;
}
