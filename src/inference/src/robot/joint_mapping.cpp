#include "robot/joint_mapping.hpp"

#include "tool/tool.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>

namespace inference {

using robot_base::index_in_range;

namespace {

// 检查 AnkleMap 中的索引是否都在 [0, count) 范围内
bool ankle_map_index_in_range(const inference::AnkleMap& ankle_map,
                              int count)
{
    return index_in_range(ankle_map.model_pitch_dof, count)   &&
           index_in_range(ankle_map.model_roll_dof,  count)   &&
           index_in_range(ankle_map.upper_motor_index, count) &&
           index_in_range(ankle_map.lower_motor_index, count);
}

}  // namespace

JointMapping::JointMapping(int dof_count, const JointMappingConfig& config)
{
    if (dof_count <= 0) {
        throw std::invalid_argument("dof_count must be positive");
    }

    if (static_cast<int>(config.motor_direction.size()) != dof_count) {
        throw std::invalid_argument("motor_direction must have one value per motor");
    }

    // 检查 左右脚踝 的索引是否在范围内
    if (!ankle_map_index_in_range(config.left_ankle,
                                  dof_count) ||
        !ankle_map_index_in_range(config.right_ankle,
                                  dof_count)) {
        throw std::invalid_argument("ankle parallel maps contain an out-of-range index");
    }

    std::vector<unsigned char> is_ankle_dof(dof_count, 0U);
    const auto& left  = config.left_ankle;
    const auto& right = config.right_ankle;
    is_ankle_dof[left.model_pitch_dof]  = 1U;
    is_ankle_dof[left.model_roll_dof]   = 1U;
    is_ankle_dof[right.model_pitch_dof] = 1U;
    is_ankle_dof[right.model_roll_dof]  = 1U;

    const int direct_model_dof_count = static_cast<int>(
        std::count(is_ankle_dof.begin(),
                   is_ankle_dof.end(),
                   0U));
    if (static_cast<int>(config.model_to_motor_index.size()) !=
        direct_model_dof_count) {
        throw std::invalid_argument("model_to_motor_index must have one value per direct model DOF");
    }

    motor_id_map_.assign(static_cast<std::size_t>(dof_count), -1);
    int mapping_slot = 0;
    for (int model_index = 0; model_index < dof_count; ++model_index) {
        if (is_ankle_dof[static_cast<std::size_t>(model_index)] != 0U) {
            continue;
        }

        const int motor_index =
            config.model_to_motor_index[static_cast<std::size_t>(mapping_slot)];
        ++mapping_slot;

        if (!index_in_range(motor_index, dof_count)) {
            throw std::invalid_argument("model_to_motor_index contains an out-of-range motor index");
        }
        motor_id_map_[static_cast<std::size_t>(model_index)] =
            motor_index;
    }

    motor_direction_ = config.motor_direction;
    left_ankle_ = config.left_ankle;
    right_ankle_ = config.right_ankle;
    dof_count_ = dof_count;
}

bool JointMapping::is_model_dof(int model_index) const
{
    return motor_id_map_[static_cast<std::size_t>(model_index)] < 0;
}

int JointMapping::get_motor_id(int model_index) const
{
    return motor_id_map_[static_cast<std::size_t>(model_index)];
}

int JointMapping::get_motor_direction(int motor_index) const
{
    return motor_direction_[static_cast<std::size_t>(motor_index)];
}

}  // namespace inference
