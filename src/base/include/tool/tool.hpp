#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
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

}  // namespace robot_base
