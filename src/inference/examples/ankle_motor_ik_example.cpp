#include "ankle_motor_ik.hpp"

#include <iomanip>
#include <iostream>

int main() {
    ankle_motor_ik::Solver solver;

    const double tests[][2] = {
        {0.0, 0.0},
        {5.0, 5.0},
        {10.0, 0.0},
        {0.0, 8.0},
    };

    std::cout << std::fixed << std::setprecision(3);
    for (const auto& test : tests) {
        const double roll = ankle_motor_ik::deg_to_rad(test[0]);
        const double pitch = ankle_motor_ik::deg_to_rad(test[1]);
        const auto motors = solver.solve(roll, pitch);

        if (!motors.reachable()) {
            std::cout << "roll=" << test[0] << " deg, pitch=" << test[1]
                      << " deg -> unreachable\n";
            continue;
        }

        std::cout << "roll=" << test[0] << " deg, pitch=" << test[1]
                  << " deg -> motor1=" << ankle_motor_ik::rad_to_deg(motors.motor1)
                  << " deg, motor2=" << ankle_motor_ik::rad_to_deg(motors.motor2)
                  << " deg\n";
    }

    return 0;
}
