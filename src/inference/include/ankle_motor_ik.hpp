#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <limits>

namespace ankle_motor_ik {

constexpr double kPi = 3.141592653589793238462643383279502884;

struct Vec3 {
    double x;
    double y;
    double z;
};

struct CandidateSet {
    std::array<double, 2> values{};
    int count = 0;
};

struct MotorAngles {
    double motor1 = std::numeric_limits<double>::quiet_NaN();
    double motor2 = std::numeric_limits<double>::quiet_NaN();

    bool motor1_reachable = false;
    bool motor2_reachable = false;

    bool reachable() const {
        return motor1_reachable && motor2_reachable;
    }
};

inline Vec3 operator-(const Vec3& lhs, const Vec3& rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

inline double dot(const Vec3& lhs, const Vec3& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

inline double wrap_to_pi(double angle) {
    double wrapped = std::fmod(angle + kPi, 2.0 * kPi);
    if (wrapped < 0.0) {
        wrapped += 2.0 * kPi;
    }
    return wrapped - kPi;
}

inline Vec3 foot_rotate_point(const Vec3& point, double roll, double pitch) {
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);

    // Rfoot = Ry(pitch) * Rx(roll).
    const double x1 = point.x;
    const double y1 = cr * point.y - sr * point.z;
    const double z1 = sr * point.y + cr * point.z;

    return {
        cp * x1 + sp * z1,
        y1,
        -sp * x1 + cp * z1,
    };
}

inline CandidateSet motor_candidates(
    const Vec3& a_point,
    const Vec3& crank_vector,
    const Vec3& c_point,
    double rod_length,
    double tol = 1e-9
) {
    const Vec3 u = c_point - a_point;

    const double a = 2.0 * (u.y * crank_vector.z - u.z * crank_vector.y);
    const double b = -2.0 * (u.y * crank_vector.y + u.z * crank_vector.z);
    const double c =
        rod_length * rod_length
        - dot(u, u)
        - dot(crank_vector, crank_vector)
        + 2.0 * u.x * crank_vector.x;

    const double radius = std::hypot(a, b);
    CandidateSet result;

    if (radius <= tol) {
        if (std::abs(c) <= tol) {
            result.values[0] = 0.0;
            result.count = 1;
        }
        return result;
    }

    const double ratio = c / radius;
    if (std::abs(ratio) > 1.0 + tol) {
        return result;
    }

    const double clipped = std::max(-1.0, std::min(1.0, ratio));
    const double alpha = std::asin(clipped);
    const double phase = std::atan2(b, a);

    result.values[0] = wrap_to_pi(alpha - phase);
    result.values[1] = wrap_to_pi(kPi - alpha - phase);
    result.count = 2;
    return result;
}

inline double nearest_solution(const CandidateSet& candidates, double previous) {
    if (candidates.count <= 0) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    double best = previous + wrap_to_pi(candidates.values[0] - previous);
    double best_abs_delta = std::abs(best - previous);

    for (int i = 1; i < candidates.count; ++i) {
        const double candidate = previous + wrap_to_pi(candidates.values[i] - previous);
        const double abs_delta = std::abs(candidate - previous);
        if (abs_delta < best_abs_delta) {
            best = candidate;
            best_abs_delta = abs_delta;
        }
    }
    return best;
}

class Solver {
public:
    Solver() = default;

    Solver(double previous_motor1, double previous_motor2)
        : previous_motor1_(previous_motor1), previous_motor2_(previous_motor2) {}

    MotorAngles solve(double roll, double pitch) {
        const Vec3 c1 = foot_rotate_point(kC1Zero, roll, pitch);
        const Vec3 c2 = foot_rotate_point(kC2Zero, roll, pitch);

        const CandidateSet candidates1 = motor_candidates(kA1, kV1, c1, kL1);
        const CandidateSet candidates2 = motor_candidates(kA2, kV2, c2, kL2);

        MotorAngles result;
        result.motor1 = nearest_solution(candidates1, previous_motor1_);
        result.motor2 = nearest_solution(candidates2, previous_motor2_);
        result.motor1_reachable = !std::isnan(result.motor1);
        result.motor2_reachable = !std::isnan(result.motor2);

        if (result.motor1_reachable) {
            previous_motor1_ = result.motor1;
        }
        if (result.motor2_reachable) {
            previous_motor2_ = result.motor2;
        }

        return result;
    }

    double previous_motor1() const {
        return previous_motor1_;
    }

    double previous_motor2() const {
        return previous_motor2_;
    }

    void reset(double motor1 = 0.0, double motor2 = 0.0) {
        previous_motor1_ = motor1;
        previous_motor2_ = motor2;
    }

private:
    static constexpr Vec3 kA1{-13.45, -0.16, 198.0};
    static constexpr Vec3 kA2{-13.45, -0.16, 140.0};
    static constexpr Vec3 kB1Zero{-41.13, 23.53, 205.98};
    static constexpr Vec3 kB2Zero{-41.13, -23.50, 148.98};
    static constexpr Vec3 kC1Zero{-38.85, 25.0, 9.0};
    static constexpr Vec3 kC2Zero{-39.03, -25.0, 9.0};

    static constexpr Vec3 kV1{
        kB1Zero.x - kA1.x,
        kB1Zero.y - kA1.y,
        kB1Zero.z - kA1.z,
    };
    static constexpr Vec3 kV2{
        kB2Zero.x - kA2.x,
        kB2Zero.y - kA2.y,
        kB2Zero.z - kA2.z,
    };

    static constexpr double kL1 = 197.0;
    static constexpr double kL2 = 140.0;

    double previous_motor1_ = 0.0;
    double previous_motor2_ = 0.0;
};

inline MotorAngles solve(double roll, double pitch, double previous_motor1 = 0.0, double previous_motor2 = 0.0) {
    Solver solver(previous_motor1, previous_motor2);
    return solver.solve(roll, pitch);
}

inline double deg_to_rad(double degrees) {
    return degrees * kPi / 180.0;
}

inline double rad_to_deg(double radians) {
    return radians * 180.0 / kPi;
}

}  // namespace ankle_motor_ik
