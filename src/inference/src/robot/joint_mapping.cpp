#include "robot/joint_mapping.hpp"

#include "tool/tool.hpp"

#include <algorithm>
#include <cstddef>

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

std::shared_ptr<const JointMapping> JointMapping::create(
    int dof_count,
    const inference::JointMappingConfig& config,
    std::string& error)
{
    std::shared_ptr<JointMapping> mapping(new JointMapping());
    mapping->config_ = config;
    if (dof_count <= 0) {
        error = "dof_count must be positive";
        return {};
    }

    if (static_cast<int>(mapping->config_.motor_direction.size()) != dof_count) {
        error = "motor_direction must have one value per motor";
        return {};
    }

    // 检查 左右脚踝 的索引是否在范围内
    if (!ankle_map_index_in_range(mapping->config_.left_ankle,
                                  dof_count) ||
        !ankle_map_index_in_range(mapping->config_.right_ankle,
                                  dof_count)) {
        error = "ankle parallel maps contain an out-of-range index";
        return {};
    }

    std::vector<unsigned char> is_ankle_dof(dof_count, 0U);
    const auto& left  = mapping->config_.left_ankle;
    const auto& right = mapping->config_.right_ankle;
    is_ankle_dof[left.model_pitch_dof]  = 1U;
    is_ankle_dof[left.model_roll_dof]   = 1U;
    is_ankle_dof[right.model_pitch_dof] = 1U;
    is_ankle_dof[right.model_roll_dof]  = 1U;

    const int direct_model_dof_count = static_cast<int>(
        std::count(is_ankle_dof.begin(),
                   is_ankle_dof.end(),
                   0U));
    if (static_cast<int>(mapping->config_.model_to_motor_index.size()) !=
        direct_model_dof_count) {
        error = "model_to_motor_index must have one value per direct model DOF";
        return {};
    }

    mapping->motor_id_map.assign(static_cast<std::size_t>(dof_count), -1);
    int mapping_slot = 0;
    for (int model_index = 0; model_index < dof_count; ++model_index) {
        if (is_ankle_dof[static_cast<std::size_t>(model_index)] != 0U) {
            continue;
        }

        const int motor_index =
            mapping->config_.model_to_motor_index[static_cast<std::size_t>(mapping_slot)];
        ++mapping_slot;

        if (!index_in_range(motor_index, dof_count)) {
            error = "model_to_motor_index contains an out-of-range motor index";
            return {};
        }
        mapping->motor_id_map[static_cast<std::size_t>(model_index)] =
            motor_index;
    }

    mapping->dof_count_ = dof_count;
    error.clear();
    return mapping;
}

bool JointMapping::is_model_dof(int model_index) const
{
    return motor_id_map[static_cast<std::size_t>(model_index)] < 0;
}

int JointMapping::get_motor_id(int model_index) const
{
    return motor_id_map[static_cast<std::size_t>(model_index)];
}

int JointMapping::get_motor_direction(int motor_index) const
{
    return config_.motor_direction[static_cast<std::size_t>(motor_index)];
}

}  // namespace inference
