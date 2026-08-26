#include "robot/target_interpolator.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

void expect(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

bool near(double actual, double expected)
{
    return std::abs(actual - expected) < 1e-9;
}

void expect_vector_near(const std::vector<double>& actual,
                        const std::vector<double>& expected,
                        const std::string& message)
{
    expect(actual.size() == expected.size(), message + ": size mismatch");
    for (std::size_t i = 0; i < expected.size(); ++i) {
        expect(near(actual[i], expected[i]),
               message + ": mismatch at index " + std::to_string(i));
    }
}

template <std::size_t N>
void expect_array_near(const std::array<double, N>& actual,
                       const std::array<double, N>& expected,
                       const std::string& message)
{
    for (std::size_t i = 0; i < N; ++i) {
        expect(near(actual[i], expected[i]),
               message + ": mismatch at index " + std::to_string(i));
    }
}

void test_zero_duration_reaches_target_immediately()
{
    inference::robot_detail::TargetInterpolator interpolator(0.0);
    const auto t0 = inference::robot_detail::TargetInterpolator::TimePoint{};

    interpolator.reset({0.0, 10.0});
    interpolator.set_target({10.0, 20.0}, t0);

    expect_vector_near(interpolator.sample(t0),
                       {10.0, 20.0},
                       "zero duration should reach target at t0");
    expect_vector_near(interpolator.sample(t0 + std::chrono::milliseconds(5)),
                       {10.0, 20.0},
                       "zero duration should hold target");
}

void test_ten_ms_interpolation()
{
    inference::robot_detail::TargetInterpolator interpolator(0.010);
    const auto t0 = inference::robot_detail::TargetInterpolator::TimePoint{};

    interpolator.reset({0.0, 0.0});
    interpolator.set_target({10.0, -10.0}, t0);

    expect_vector_near(interpolator.sample(t0 + std::chrono::milliseconds(5)),
                       {5.0, -5.0},
                       "10ms interpolation should be halfway at 5ms");
    expect_vector_near(interpolator.sample(t0 + std::chrono::milliseconds(10)),
                       {10.0, -10.0},
                       "10ms interpolation should finish at 10ms");
    expect_vector_near(interpolator.sample(t0 + std::chrono::milliseconds(20)),
                       {10.0, -10.0},
                       "finished interpolation should hold target");
}

void test_retarget_starts_from_current_smoothed_value()
{
    inference::robot_detail::TargetInterpolator interpolator(0.010);
    const auto t0 = inference::robot_detail::TargetInterpolator::TimePoint{};

    interpolator.reset({0.0});
    interpolator.set_target({10.0}, t0);
    expect_vector_near(interpolator.sample(t0 + std::chrono::milliseconds(5)),
                       {5.0},
                       "first target should be halfway at 5ms");

    interpolator.set_target({20.0}, t0 + std::chrono::milliseconds(5));
    expect_vector_near(interpolator.sample(t0 + std::chrono::milliseconds(10)),
                       {12.5},
                       "retarget should interpolate from current smoothed value");
    expect_vector_near(interpolator.sample(t0 + std::chrono::milliseconds(15)),
                       {20.0},
                       "retarget should finish after another 10ms");
}

void test_fixed_interpolator_matches_vector_behavior()
{
    inference::robot_detail::FixedTargetInterpolator<2> interpolator(0.010);
    const auto t0 =
        inference::robot_detail::FixedTargetInterpolator<2>::TimePoint{};

    interpolator.reset({0.0, 0.0});
    interpolator.set_target({10.0, -10.0}, t0);

    expect_array_near(interpolator.sample(t0 + std::chrono::milliseconds(5)),
                      std::array<double, 2>{5.0, -5.0},
                      "fixed 10ms interpolation should be halfway at 5ms");
    expect_array_near(interpolator.sample(t0 + std::chrono::milliseconds(10)),
                      std::array<double, 2>{10.0, -10.0},
                      "fixed 10ms interpolation should finish at 10ms");
}

}  // namespace

int main()
{
    test_zero_duration_reaches_target_immediately();
    test_ten_ms_interpolation();
    test_retarget_starts_from_current_smoothed_value();
    test_fixed_interpolator_matches_vector_behavior();

    std::cout << "target_interpolator_test passed\n";
    return 0;
}
