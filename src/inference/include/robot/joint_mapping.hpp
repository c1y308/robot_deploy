#pragma once

#include "robot/joint_mapping_config.hpp"

#include <memory>
#include <string>
#include <vector>

namespace inference::robot_detail {

class JointMapping {
public:
    static std::shared_ptr<const JointMapping> create(
        int dof_count,
        const inference::JointMappingConfig& config,
        std::string& error);

    int dof_count() const noexcept { return dof_count_; }

    const inference::AnkleParallelMap& left_ankle() const noexcept { return config_.left_ankle_parallel; }
    const inference::AnkleParallelMap& right_ankle() const noexcept { return config_.right_ankle_parallel; }

    bool is_parallel_model_dof(int model_index) const;
    int direct_motor_for_model_dof(int model_index) const;
    int direction_for_motor(int motor_index) const;

private:
    JointMapping() = default;

    bool configure(int dof_count, std::string& error);

    inference::JointMappingConfig config_;
    int dof_count_ = 0;
    std::vector<unsigned char> parallel_model_dof_;
    std::vector<int> direct_motor_for_model_dof_;
};

}  // namespace inference::robot_detail
