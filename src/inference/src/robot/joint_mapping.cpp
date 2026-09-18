#include "robot/joint_mapping.hpp"

#include "tool/tool.hpp"

#include <algorithm>
#include <cstddef>

namespace inference::robot_detail {

using robot_base::index_in_range;

namespace {

// 检查 AnkleParallelMap 中的索引是否都在 [0, count) 范围内
bool ankle_parallel_map_indices_in_range(const inference::AnkleParallelMap& ankle_map,
                                         int count)
{
    return index_in_range(ankle_map.model_pitch_dof, count) &&
           index_in_range(ankle_map.model_roll_dof,  count) &&
           index_in_range(ankle_map.upper_motor_index, count) &&
           index_in_range(ankle_map.lower_motor_index, count);
}

// 检查方向数组值是否为 1 或 -1
template <typename DirectionContainer>
bool valid_motor_direction_values(const DirectionContainer& directions)
{
    return std::all_of(directions.begin(), directions.end(), [](int direction) {
        return direction == 1 || direction == -1;
    });
}

}  // namespace

std::shared_ptr<const JointMapping> JointMapping::create(
    int dof_count,
    const inference::JointMappingConfig& config,
    std::string& error)
{
    std::shared_ptr<JointMapping> mapping(new JointMapping());
    mapping->config_ = config;
    if (!mapping->configure(dof_count)) {
        error = mapping->last_error();
        return {};
    }
    error.clear();
    return mapping;
}

bool JointMapping::configure(int dof_count)
{
    if (dof_count <= 0) {
        last_error_ = "dof_count must be positive";
        return false;
    }

    if (static_cast<int>(config_.motor_to_model_direction.size()) != dof_count) {
        last_error_ = "motor_to_model_direction must have one value per motor";
        return false;
    }
    if (!valid_motor_direction_values(config_.motor_to_model_direction)) {
        last_error_ = "motor_to_model_direction values must be 1 or -1";
        return false;
    }

    if (!ankle_parallel_map_indices_in_range(config_.left_ankle_parallel,
                                             dof_count) ||
        !ankle_parallel_map_indices_in_range(config_.right_ankle_parallel,
                                             dof_count)) {
        last_error_ = "ankle parallel maps contain an out-of-range index";
        return false;
    }

    parallel_model_dof_.assign(static_cast<std::size_t>(dof_count), 0U);
    std::vector<unsigned char> seen_motor(
        static_cast<std::size_t>(dof_count), 0U);

    auto mark_model_dof = [&](int model_dof) {
        auto& seen = parallel_model_dof_[static_cast<std::size_t>(model_dof)];
        if (seen != 0U) {
            return false;
        }
        seen = 1U;
        return true;
    };

    auto mark_motor = [&](int motor_index) {
        auto& seen = seen_motor[static_cast<std::size_t>(motor_index)];
        if (seen != 0U) {
            return false;
        }
        seen = 1U;
        return true;
    };

    const auto& left  = config_.left_ankle_parallel;
    const auto& right = config_.right_ankle_parallel;
    if (!mark_model_dof(left.model_pitch_dof)  ||
        !mark_model_dof(left.model_roll_dof)   ||
        !mark_model_dof(right.model_pitch_dof) ||
        !mark_model_dof(right.model_roll_dof)) {
        last_error_ = "ankle parallel model DOFs must be distinct";
        return false;
    }

    if (!mark_motor(left.upper_motor_index) ||
        !mark_motor(left.lower_motor_index) ||
        !mark_motor(right.upper_motor_index) ||
        !mark_motor(right.lower_motor_index)) {
        last_error_ = "ankle parallel motor indices must be distinct";
        return false;
    }

    const int direct_model_dof_count = static_cast<int>(
        std::count(parallel_model_dof_.begin(),
                   parallel_model_dof_.end(),
                   0U));
    if (static_cast<int>(config_.model_to_motor_index.size()) !=
        direct_model_dof_count) {
        last_error_ = "model_to_motor_index must have one value per direct model DOF";
        return false;
    }

    direct_motor_for_model_dof_.assign(static_cast<std::size_t>(dof_count), -1);
    int mapping_slot = 0;
    for (int model_index = 0; model_index < dof_count; ++model_index) {
        if (parallel_model_dof_[static_cast<std::size_t>(model_index)] != 0U) {
            continue;
        }

        const int motor_index =
            config_.model_to_motor_index[static_cast<std::size_t>(mapping_slot)];
        ++mapping_slot;

        if (!index_in_range(motor_index, dof_count)) {
            last_error_ = "model_to_motor_index contains an out-of-range motor index";
            return false;
        }
        if (!mark_motor(motor_index)) {
            last_error_ = "direct and ankle motor mappings must be a permutation without duplicates";
            return false;
        }

        direct_motor_for_model_dof_[static_cast<std::size_t>(model_index)] =
            motor_index;
    }

    if (std::any_of(seen_motor.begin(), seen_motor.end(), [](unsigned char seen) {
            return seen == 0U;
        })) {
        last_error_ = "direct and ankle motor mappings must cover all motors";
        return false;
    }

    dof_count_ = dof_count;
    return true;
}

bool JointMapping::is_parallel_model_dof(int model_index) const
{
    return parallel_model_dof_[static_cast<std::size_t>(model_index)] != 0U;
}

int JointMapping::direct_motor_for_model_dof(int model_index) const
{
    return direct_motor_for_model_dof_[static_cast<std::size_t>(model_index)];
}

int JointMapping::direction_for_motor(int motor_index) const
{
    return config_.motor_to_model_direction[static_cast<std::size_t>(motor_index)];
}

}  // namespace inference::robot_detail
