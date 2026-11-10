#pragma once

#include "motor_base/command_types.hpp"

#include <array>
#include <vector>

namespace inference {

struct AnkleMap {
    // 模型输出的 pitch/roll DOF 在模型关节数组中的下标
    int model_pitch_dof = -1;
    int model_roll_dof  = -1;

    // 这一组 roll/pitch DOF 算出后角度对应的物理电机下标
    int upper_motor_index = -1;
    int lower_motor_index = -1;
};

struct JointMappingConfig {
    /* 直驱模型 DOF（按模型顺序，跳过脚踝并联轴）→ 物理电机下标 */
    std::vector<int> model_to_motor_index = {0, 6, 1, 7, 2, 8, 3, 9};

    /* 物理电机顺序：电机方向与模型方向的关系，1 同向 / -1 反向 */
    std::array<int, motor_base::kMotorNum> motor_direction = {
        -1, -1, 1, 1, -1, -1,
        -1, 1, 1, -1, -1, -1
    };

    // 脚踝并联轴 用 AnkleMap 表示
    AnkleMap left_ankle  = {8, 10, 4, 5};
    AnkleMap right_ankle = {9, 11, 10, 11};
};

class JointMapping {
public:
    // 非法配置抛出 std::invalid_argument；映射表在构造时生成。
    JointMapping(int dof_count, const JointMappingConfig& config);

    int dof_count() const noexcept { return dof_count_; }

    const AnkleMap& left_ankle()  const noexcept { return left_ankle_; }
    const AnkleMap& right_ankle() const noexcept { return right_ankle_; }

    bool is_model_dof(int model_index) const;
    int  get_motor_id(int model_index) const;
    int  get_motor_direction(int motor_index) const;

private:
    std::array<int, motor_base::kMotorNum> motor_direction_{};
    AnkleMap left_ankle_;
    AnkleMap right_ankle_;
    int dof_count_ = 0;
    std::vector<int> motor_id_map_;
};

}  // namespace inference
