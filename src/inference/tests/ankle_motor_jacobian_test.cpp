#include "kinematics/ankle_motor_ik.hpp"
#include "kinematics/ankle_motor_jacobian.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace {

void expect(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

void expect_near(double actual, double expected, double tolerance, const std::string& message)
{
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << "FAIL: " << message
                  << " expected=" << expected
                  << " actual=" << actual
                  << " tolerance=" << tolerance << "\n";
        std::exit(1);
    }
}

ankle_motor_ik::MotorAngles solve_motor_angles(double pitch,
                                               double roll,
                                               double reference_motor1 = 0.0,
                                               double reference_motor2 = 0.0)
{
    const ankle_motor_ik::MotorAngles result =
        ankle_motor_ik::solve(roll, pitch, reference_motor1, reference_motor2);
    expect(result.reachable(), "ankle IK should solve the test pose");
    return result;
}

void test_matches_finite_difference()
{
    constexpr double kPitch = -0.05;
    constexpr double kRoll = 0.04;
    constexpr double kStep = 1e-6;
    constexpr double kTolerance = 1e-5;

    const ankle_motor_ik::MotorAngles center =
        solve_motor_angles(kPitch, kRoll);

    ankle_motor_jacobian::Result jacobian;
    std::string error;
    expect(ankle_motor_jacobian::solve(kPitch,
                                       kRoll,
                                       center.motor1,
                                       center.motor2,
                                       jacobian,
                                       error),
           "jacobian should solve: " + error);

    const ankle_motor_ik::MotorAngles pitch_plus =
        solve_motor_angles(kPitch + kStep, kRoll, center.motor1, center.motor2);
    const ankle_motor_ik::MotorAngles pitch_minus =
        solve_motor_angles(kPitch - kStep, kRoll, center.motor1, center.motor2);
    const ankle_motor_ik::MotorAngles roll_plus =
        solve_motor_angles(kPitch, kRoll + kStep, center.motor1, center.motor2);
    const ankle_motor_ik::MotorAngles roll_minus =
        solve_motor_angles(kPitch, kRoll - kStep, center.motor1, center.motor2);

    const double numeric[2][2] = {
        {
            (pitch_plus.motor1 - pitch_minus.motor1) / (2.0 * kStep),
            (roll_plus.motor1 - roll_minus.motor1) / (2.0 * kStep),
        },
        {
            (pitch_plus.motor2 - pitch_minus.motor2) / (2.0 * kStep),
            (roll_plus.motor2 - roll_minus.motor2) / (2.0 * kStep),
        },
    };

    for (int row = 0; row < 2; ++row) {
        for (int col = 0; col < 2; ++col) {
            expect_near(jacobian.motor_from_virtual[row][col],
                        numeric[row][col],
                        kTolerance,
                        "analytic motor_from_virtual should match finite difference");
        }
    }
}

void test_inverse_product()
{
    const ankle_motor_ik::MotorAngles center =
        solve_motor_angles(0.02, -0.03);

    ankle_motor_jacobian::Result jacobian;
    std::string error;
    expect(ankle_motor_jacobian::solve(0.02,
                                       -0.03,
                                       center.motor1,
                                       center.motor2,
                                       jacobian,
                                       error),
           "jacobian should solve for inverse test: " + error);

    for (int row = 0; row < 2; ++row) {
        for (int col = 0; col < 2; ++col) {
            const double product =
                jacobian.virtual_from_motor[row][0] * jacobian.motor_from_virtual[0][col] +
                jacobian.virtual_from_motor[row][1] * jacobian.motor_from_virtual[1][col];
            expect_near(product,
                        row == col ? 1.0 : 0.0,
                        1e-9,
                        "virtual_from_motor should invert motor_from_virtual");
        }
    }
}

void test_rejects_non_finite_inputs()
{
    ankle_motor_jacobian::Result jacobian;
    std::string error;
    expect(!ankle_motor_jacobian::solve(std::numeric_limits<double>::quiet_NaN(),
                                        0.0,
                                        0.0,
                                        0.0,
                                        jacobian,
                                        error),
           "jacobian should reject non-finite inputs");
    expect(!error.empty(), "jacobian failure should explain the error");
}

void test_rejects_singular_branch()
{
    const ankle_motor_ik::MotorAngles center =
        solve_motor_angles(0.0, 0.0);

    ankle_motor_jacobian::Result jacobian;
    std::string error;
    expect(!ankle_motor_jacobian::solve(0.0,
                                        0.0,
                                        center.motor1,
                                        center.motor2,
                                        jacobian,
                                        error,
                                        1e9),
           "jacobian should reject a branch below the singular tolerance");
    expect(!error.empty(), "singular jacobian failure should explain the error");
}

}  // namespace

int main()
{
    test_matches_finite_difference();
    test_inverse_product();
    test_rejects_non_finite_inputs();
    test_rejects_singular_branch();

    std::cout << "ankle_motor_jacobian_test passed\n";
    return 0;
}
