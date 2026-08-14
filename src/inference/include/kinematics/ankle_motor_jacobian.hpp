#pragma once

#include "kinematics/ankle_motor_ik.hpp"

#include <array>
#include <cstddef>
#include <cmath>
#include <limits>
#include <string>

namespace ankle_motor_jacobian {

struct Result {
    std::array<double, 2> motor_angles{};
    std::array<std::array<double, 2>, 2> motor_from_virtual{};
    std::array<std::array<double, 2>, 2> virtual_from_motor{};
    double determinant = std::numeric_limits<double>::quiet_NaN();
};

namespace detail {

using ankle_motor_ik::Geometry;
using ankle_motor_ik::Vec3;

inline Vec3 foot_pitch_derivative(const Vec3& point, double roll, double pitch)
{
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);

    const double x1 = point.x;
    const double z1 = sr * point.y + cr * point.z;

    return {
        -sp * x1 + cp * z1,
        0.0,
        -cp * x1 - sp * z1,
    };
}

inline Vec3 foot_roll_derivative(const Vec3& point, double roll, double pitch)
{
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);

    const double dy1 = -sr * point.y - cr * point.z;
    const double dz1 =  cr * point.y - sr * point.z;

    return {
        sp * dz1,
        dy1,
        cp * dz1,
    };
}

inline Vec3 crank_motor_derivative(const Vec3& crank_vector, double motor_angle)
{
    const double c = std::cos(motor_angle);
    const double s = std::sin(motor_angle);

    return {
        0.0,
        -s * crank_vector.y - c * crank_vector.z,
         c * crank_vector.y - s * crank_vector.z,
    };
}

inline bool finite_vec3(const Vec3& value)
{
    return std::isfinite(value.x) &&
           std::isfinite(value.y) &&
           std::isfinite(value.z);
}

inline bool finite_matrix2(const std::array<std::array<double, 2>, 2>& matrix)
{
    return std::isfinite(matrix[0][0]) &&
           std::isfinite(matrix[0][1]) &&
           std::isfinite(matrix[1][0]) &&
           std::isfinite(matrix[1][1]);
}

}  // namespace detail

inline bool solve(double pitch,
                  double roll,
                  double motor1,
                  double motor2,
                  Result& result,
                  std::string& error,
                  double singular_tolerance = 1e-10)
{
    if (!std::isfinite(pitch) ||
        !std::isfinite(roll) ||
        !std::isfinite(motor1) ||
        !std::isfinite(motor2) ||
        !std::isfinite(singular_tolerance) ||
        singular_tolerance <= 0.0) {
        error = "ankle jacobian inputs must be finite";
        return false;
    }

    const std::array<double, 2> motors{motor1, motor2};
    const std::array<detail::Vec3, 2> a_points{detail::Geometry::kA1,
                                                detail::Geometry::kA2};
    const std::array<detail::Vec3, 2> crank_vectors{detail::Geometry::kV1,
                                                    detail::Geometry::kV2};
    const std::array<detail::Vec3, 2> c_locals{detail::Geometry::kC1Zero,
                                               detail::Geometry::kC2Zero};

    Result next;
    next.motor_angles = motors;

    for (std::size_t i = 0; i < 2; ++i) {
        const detail::Vec3 c_point =
            ankle_motor_ik::foot_rotate_point(c_locals[i], roll, pitch);
        const detail::Vec3 b_point =
            ankle_motor_ik::crank_rotate_point(a_points[i],
                                               crank_vectors[i],
                                               motors[i]);
        const detail::Vec3 residual = c_point - b_point;
        const detail::Vec3 dc_pitch =
            detail::foot_pitch_derivative(c_locals[i], roll, pitch);
        const detail::Vec3 dc_roll =
            detail::foot_roll_derivative(c_locals[i], roll, pitch);
        const detail::Vec3 db_motor =
            detail::crank_motor_derivative(crank_vectors[i], motors[i]);

        if (!detail::finite_vec3(residual) ||
            !detail::finite_vec3(dc_pitch) ||
            !detail::finite_vec3(dc_roll) ||
            !detail::finite_vec3(db_motor)) {
            error = "ankle jacobian intermediate value is not finite";
            return false;
        }

        const double d_f_motor = -2.0 * ankle_motor_ik::dot(residual, db_motor);
        if (!std::isfinite(d_f_motor) ||
            std::abs(d_f_motor) <= singular_tolerance) {
            error = "ankle jacobian branch is singular";
            return false;
        }

        next.motor_from_virtual[i][0] =
            -(2.0 * ankle_motor_ik::dot(residual, dc_pitch)) / d_f_motor;
        next.motor_from_virtual[i][1] =
            -(2.0 * ankle_motor_ik::dot(residual, dc_roll)) / d_f_motor;
    }

    if (!detail::finite_matrix2(next.motor_from_virtual)) {
        error = "ankle motor_from_virtual jacobian is not finite";
        return false;
    }

    const double a = next.motor_from_virtual[0][0];
    const double b = next.motor_from_virtual[0][1];
    const double c = next.motor_from_virtual[1][0];
    const double d = next.motor_from_virtual[1][1];
    const double determinant = a * d - b * c;
    if (!std::isfinite(determinant) ||
        std::abs(determinant) <= singular_tolerance) {
        error = "ankle jacobian matrix is singular";
        return false;
    }

    next.virtual_from_motor[0][0] =  d / determinant;
    next.virtual_from_motor[0][1] = -b / determinant;
    next.virtual_from_motor[1][0] = -c / determinant;
    next.virtual_from_motor[1][1] =  a / determinant;
    next.determinant = determinant;

    if (!detail::finite_matrix2(next.virtual_from_motor)) {
        error = "ankle virtual_from_motor jacobian is not finite";
        return false;
    }

    result = next;
    error.clear();
    return true;
}

inline bool solve(double pitch,
                  double roll,
                  double motor1,
                  double motor2,
                  Result& result,
                  double singular_tolerance = 1e-10)
{
    std::string error;
    return solve(pitch, roll, motor1, motor2, result, error, singular_tolerance);
}

}  // namespace ankle_motor_jacobian
