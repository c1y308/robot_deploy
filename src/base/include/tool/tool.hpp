#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <limits>
#include <vector>

namespace robot_base {

/// @brief 判断索引是否在 [0, count) 范围内
inline bool index_in_range(int index, int count)
{
    return index >= 0 && index < count;
}

/// @brief 检查数组中的数值是否全部为有限值
template <std::size_t N>
bool finite_array(const std::array<double, N>& values)
{
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    });
}

/// @brief 检查动态数组中的数值是否全部为有限值
inline bool finite_vector(const std::vector<double>& values)
{
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    });
}

/// @brief CLOCK_MONOTONIC timestamp in nanoseconds.
inline std::int64_t monotonic_now_ns() noexcept
{
    timespec time{};
    clock_gettime(CLOCK_MONOTONIC, &time);
    return static_cast<std::int64_t>(time.tv_sec) * 1000000000LL +
           static_cast<std::int64_t>(time.tv_nsec);
}

/// @brief double → int32_t（四舍五入）
inline int32_t double_to_i32(double value)
{
    return static_cast<int32_t>(std::llround(value));
}

/// @brief double → int16_t（四舍五入）
inline int16_t double_to_i16(double value)
{
    return static_cast<int16_t>(std::llround(value));
}

/// @brief 判断 double 值能否无损存入 int32_t
inline bool fits_i32(double value)
{
    const double lo = static_cast<double>(std::numeric_limits<int32_t>::min());
    const double hi = static_cast<double>(std::numeric_limits<int32_t>::max());
    return std::isfinite(value) && value >= lo && value <= hi;
}

/// @brief 判断 double 值能否无损存入 int16_t
inline bool fits_i16(double value)
{
    const double lo = static_cast<double>(std::numeric_limits<int16_t>::min());
    const double hi = static_cast<double>(std::numeric_limits<int16_t>::max());
    return std::isfinite(value) && value >= lo && value <= hi;
}

}  // namespace robot_base
