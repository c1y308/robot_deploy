#include "robot/joint_mapping.hpp"

#include <algorithm>
#include <cstddef>
#include <utility>

namespace inference::robot_detail {
namespace {

bool index_in_range(int index, int count)
{
    return index >= 0 && index < count;
}

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
bool valid_motor_direction_values(const std::vector<int>& directions)
{
    return std::all_of(directions.begin(), directions.end(), [](int direction) {
        return direction == 1 || direction == -1;
    });
}


bool build_mapping(int config_dof_count,
                   const inference::JointMappingConfig& config,
                   int& dof_count,
                   std::vector<unsigned char>& parallel_model_dof,
                   std::vector<int>& direct_motor_for_model_dof,
                   std::vector<int>& motor_to_model_direction,  //电机方向
                   std::string& error)
{
    if (config_dof_count <= 0) {
        error = "dof_count must be positive";
        return false;
    }

    if (!config.motor_to_model_direction.empty() &&
        static_cast<int>(config.motor_to_model_direction.size()) != config_dof_count) {
        error = "motor_to_model_direction must have one value per motor or be empty";
        return false;
    }
    if (!valid_motor_direction_values(config.motor_to_model_direction)) {
        error = "motor_to_model_direction values must be 1 or -1";
        return false;
    }

    if (!ankle_parallel_map_indices_in_range(config.left_ankle_parallel,
                                             config_dof_count) ||
        !ankle_parallel_map_indices_in_range(config.right_ankle_parallel,
                                             config_dof_count)) {
        error = "ankle parallel maps contain an out-of-range index";
        return false;
    }

    std::vector<unsigned char> seen_ankle_model_dof(
        static_cast<std::size_t>(config_dof_count), 0U);
    std::vector<unsigned char> seen_motor(
        static_cast<std::size_t>(config_dof_count), 0U);

    auto mark_model_dof = [&](int model_dof) {
        auto& seen = seen_ankle_model_dof[static_cast<std::size_t>(model_dof)];
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

    const auto& left  = config.left_ankle_parallel;
    const auto& right = config.right_ankle_parallel;
    if (!mark_model_dof(left.model_pitch_dof)  ||
        !mark_model_dof(left.model_roll_dof)   ||
        !mark_model_dof(right.model_pitch_dof) ||
        !mark_model_dof(right.model_roll_dof)) {
        error = "ankle parallel model DOFs must be distinct";
        return false;
    }

    if (!mark_motor(left.upper_motor_index) ||
        !mark_motor(left.lower_motor_index) ||
        !mark_motor(right.upper_motor_index) ||
        !mark_motor(right.lower_motor_index)) {
        error = "ankle parallel motor indices must be distinct";
        return false;
    }

    const int direct_model_dof_count = static_cast<int>(
        std::count(seen_ankle_model_dof.begin(),
                   seen_ankle_model_dof.end(),
                   0U));
    if (static_cast<int>(config.model_to_motor_index.size()) !=
        direct_model_dof_count) {
        error = "model_to_motor_index must have one value per direct model DOF";
        return false;
    }

    std::vector<int> next_direct_motor_for_model_dof(
        static_cast<std::size_t>(config_dof_count), -1);
    int mapping_slot = 0;
    for (int model_index = 0; model_index < config_dof_count; ++model_index) {
        if (seen_ankle_model_dof[static_cast<std::size_t>(model_index)] != 0U) {
            continue;
        }

        const int motor_index =
            config.model_to_motor_index[static_cast<std::size_t>(mapping_slot)];
        ++mapping_slot;

        if (!index_in_range(motor_index, config_dof_count)) {
            error = "model_to_motor_index contains an out-of-range motor index";
            return false;
        }
        if (!mark_motor(motor_index)) {
            error = "direct and ankle motor mappings must be a permutation without duplicates";
            return false;
        }

        next_direct_motor_for_model_dof[static_cast<std::size_t>(model_index)] =
            motor_index;
    }

    if (std::any_of(seen_motor.begin(), seen_motor.end(), [](unsigned char seen) {
            return seen == 0U;
        })) {
        error = "direct and ankle motor mappings must cover all motors";
        return false;
    }

    std::vector<int> next_motor_to_model_direction(
        static_cast<std::size_t>(config_dof_count), 1);
    if (!config.motor_to_model_direction.empty()) {
        next_motor_to_model_direction = config.motor_to_model_direction;
    }

    dof_count = config_dof_count;
    parallel_model_dof = std::move(seen_ankle_model_dof);
    direct_motor_for_model_dof = std::move(next_direct_motor_for_model_dof);
    motor_to_model_direction = std::move(next_motor_to_model_direction);
    error.clear();
    return true;
}

}  // namespace

std::shared_ptr<const JointMapping> JointMapping::create(
    int dof_count,
    const inference::JointMappingConfig& config,
    std::string& error)
{
    std::shared_ptr<JointMapping> mapping(new JointMapping());
    if (!mapping->configure(dof_count, config)) {
        error = mapping->last_error();
        return {};
    }
    error.clear();
    return mapping;
}

bool JointMapping::configure(int dof_count, const inference::JointMappingConfig& config)
{
    int next_dof_count = 0;
    std::vector<unsigned char> next_parallel_model_dof;
    std::vector<int> next_direct_motor_for_model_dof;
    std::vector<int> next_motor_to_model_direction;
    std::string error;

    if (!build_mapping(dof_count,
                       config,
                       next_dof_count,
                       next_parallel_model_dof,
                       next_direct_motor_for_model_dof,
                       next_motor_to_model_direction,
                       error)) {
        configured_ = false;
        dof_count_ = 0;
        parallel_model_dof_.clear();
        direct_motor_for_model_dof_.clear();
        motor_to_model_direction_.clear();
        left_ankle_parallel_ = {};
        right_ankle_parallel_ = {};
        last_error_ = std::move(error);
        return false;
    }

    dof_count_ = next_dof_count;
    parallel_model_dof_ = std::move(next_parallel_model_dof);
    direct_motor_for_model_dof_ = std::move(next_direct_motor_for_model_dof);
    motor_to_model_direction_ = std::move(next_motor_to_model_direction);
    left_ankle_parallel_ = config.left_ankle_parallel;
    right_ankle_parallel_ = config.right_ankle_parallel;
    configured_ = true;
    last_error_.clear();
    return true;
}

bool JointMapping::validate(int dof_count,
                            const inference::JointMappingConfig& config,
                            std::string& error)
{
    int next_dof_count = 0;
    std::vector<unsigned char> parallel_model_dof;
    std::vector<int> direct_motor_for_model_dof;
    std::vector<int> motor_to_model_direction;
    return build_mapping(dof_count,
                         config,
                         next_dof_count,
                         parallel_model_dof,
                         direct_motor_for_model_dof,
                         motor_to_model_direction,
                         error);
}

bool JointMapping::is_parallel_model_dof(int model_index) const
{
    if (!configured_ || !index_in_range(model_index, dof_count_)) {
        return false;
    }
    return parallel_model_dof_[static_cast<std::size_t>(model_index)] != 0U;
}

int JointMapping::direct_motor_for_model_dof(int model_index) const
{
    if (!configured_ || !index_in_range(model_index, dof_count_)) {
        return -1;
    }
    return direct_motor_for_model_dof_[static_cast<std::size_t>(model_index)];
}

int JointMapping::direction_for_motor(int motor_index) const
{
    if (!configured_ || !index_in_range(motor_index, dof_count_)) {
        return 1;
    }
    return motor_to_model_direction_[static_cast<std::size_t>(motor_index)];
}

}  // namespace inference::robot_detail
