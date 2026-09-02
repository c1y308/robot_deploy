#pragma once

#include <array>
#include <vector>

namespace inference {

/* 脚踝并联机构：两个模型虚拟轴（pitch/roll）由上下两个物理电机并联驱动。 */
struct AnkleParallelMap {
    int model_pitch_dof = -1;
    int model_roll_dof  = -1;

    int upper_motor_index = -1;
    int lower_motor_index = -1;
};

struct JointMappingConfig {
    /* 直驱模型 DOF（按模型顺序，跳过脚踝并联轴）→ 物理电机下标 */
    std::vector<int> model_to_motor_index = {0, 6, 1, 7, 2, 8, 3, 9};
    /* 物理电机顺序：电机方向与模型方向的关系，1 同向 / -1 反向 */
    std::array<int, 12> motor_to_model_direction = {
        -1, -1, 1,  1, -1, -1,
        -1,  1, 1, -1, -1, -1
    };

    AnkleParallelMap left_ankle_parallel  = {8, 10, 4, 5};
    AnkleParallelMap right_ankle_parallel = {9, 11, 10, 11};
};

}  // namespace inference
