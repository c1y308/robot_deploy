#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace inference {

/* 脚踝并联机构：两个模型虚拟轴（pitch/roll）由上下两个物理电机并联驱动。 */
struct AnkleMap {
    int model_pitch_dof = -1;
    int model_roll_dof  = -1;

    int upper_motor_index = -1;
    int lower_motor_index = -1;
};

struct JointMappingConfig {
    /* 直驱模型 DOF（按模型顺序，跳过脚踝并联轴）→ 物理电机下标 */
    std::vector<int> model_to_motor_index = {0, 6, 1, 7, 2, 8, 3, 9};
    /* 物理电机顺序：电机方向与模型方向的关系，1 同向 / -1 反向 */
    std::array<int, 12> motor_direction = {
        -1, -1, 1, 1, -1, -1,
        -1, 1, 1, -1, -1, -1
    };

    AnkleMap left_ankle  = {8, 10, 4, 5};
    AnkleMap right_ankle = {9, 11, 10, 11};
};

class JointMapping {
public:
    static std::shared_ptr<const JointMapping> create(int                       dof_count,
                                                      const JointMappingConfig& config,
                                                      std::string&              error);

    int dof_count() const noexcept { return dof_count_; }

    const AnkleMap& left_ankle()  const noexcept { return config_.left_ankle; }
    const AnkleMap& right_ankle() const noexcept { return config_.right_ankle; }

    bool is_model_dof(int model_index) const;
    int  get_motor_id(int model_index) const;
    int  get_motor_direction(int motor_index) const;

private:
    JointMapping() = default;

    /*  */
    inference::JointMappingConfig config_;
    int dof_count_ = 0;
    std::vector<int> motor_id_map;
};

}  // namespace inference
