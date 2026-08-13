#pragma once

#include <memory>
#include <string>
#include <vector>

namespace inference {

struct AnkleParallelMap {
    int model_pitch_dof = -1;
    int model_roll_dof  = -1;

    int upper_motor_index = -1;
    int lower_motor_index = -1;
};

struct JointMappingConfig {
    std::vector<int> model_to_motor_index;
    std::vector<int> motor_to_model_direction;

    AnkleParallelMap left_ankle_parallel;
    AnkleParallelMap right_ankle_parallel;
};

}  // namespace inference

namespace inference::robot_detail {

class JointMapping {
public:
    static std::shared_ptr<const JointMapping> create(
        int dof_count,
        const inference::JointMappingConfig& config,
        std::string& error);
        
    static bool validate(int dof_count,
                         const inference::JointMappingConfig& config,
                         std::string& error);

    bool configured() const noexcept { return configured_; }
    int dof_count() const noexcept { return dof_count_; }

    const std::string& last_error() const noexcept { return last_error_; }
    const inference::AnkleParallelMap& left_ankle() const noexcept { return left_ankle_parallel_; }
    const inference::AnkleParallelMap& right_ankle() const noexcept { return right_ankle_parallel_; }

    bool is_parallel_model_dof(int model_index) const;
    int direct_motor_for_model_dof(int model_index) const;
    int direction_for_motor(int motor_index) const;

private:
    JointMapping() = default;

    bool configure(int dof_count, const inference::JointMappingConfig& config);

    int dof_count_ = 0;
    bool configured_ = false;
    std::string last_error_;
    std::vector<unsigned char> parallel_model_dof_;
    std::vector<int> direct_motor_for_model_dof_;
    std::vector<int> motor_to_model_direction_;
    
    inference::AnkleParallelMap left_ankle_parallel_;
    inference::AnkleParallelMap right_ankle_parallel_;
};

}  // namespace inference::robot_detail
