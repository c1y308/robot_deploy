#include "robot/joint_mapping.hpp"

#include <array>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void check(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

void verify(const inference::JointMappingConfig& config,
            const std::array<int, 12>& expected)
{
    const inference::JointMapping mapping(12, config);
    check(mapping.dof_count() == 12, "incorrect DOF count");
    for (int i = 0; i < 12; ++i) {
        check(mapping.get_motor_id(i) == expected[i], "incorrect motor index");
        check(mapping.is_model_dof(i) == (expected[i] < 0), "incorrect ankle classification");
        check(mapping.get_motor_direction(i) == config.motor_direction[i], "incorrect motor direction");
    }
    const auto check_ankle = [](const inference::AnkleMap& actual,
                                const inference::AnkleMap& expected_ankle) {
        check(actual.model_pitch_dof == expected_ankle.model_pitch_dof &&
              actual.model_roll_dof == expected_ankle.model_roll_dof &&
              actual.upper_motor_index == expected_ankle.upper_motor_index &&
              actual.lower_motor_index == expected_ankle.lower_motor_index,
              "incorrect ankle mapping");
    };
    check_ankle(mapping.left_ankle(), config.left_ankle);
    check_ankle(mapping.right_ankle(), config.right_ankle);
}

void rejects(int count, const inference::JointMappingConfig& config, const char* expected)
{
    try {
        const inference::JointMapping mapping(count, config);
        check(false, "invalid mapping accepted");
    } catch (const std::invalid_argument& error) {
        check(error.what() == std::string(expected), "incorrect mapping error");
    }
}
}

int main()
{
    inference::JointMappingConfig config;
    verify(config, {0, 6, 1, 7, 2, 8, 3, 9, -1, -1, -1, -1});
    config.left_ankle = {1, 5, 8, 10};
    config.right_ankle = {3, 7, 9, 11};
    config.model_to_motor_index = {7, 6, 5, 4, 3, 2, 1, 0};
    config.motor_direction = {1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1};
    verify(config, {7, -1, 6, -1, 5, -1, 4, -1, 3, 2, 1, 0});

    rejects(0, config, "dof_count must be positive");
    rejects(11, config, "motor_direction must have one value per motor");
    auto invalid = config;
    invalid.left_ankle.upper_motor_index = 12;
    rejects(12, invalid, "ankle parallel maps contain an out-of-range index");
    invalid = config;
    invalid.model_to_motor_index.pop_back();
    rejects(12, invalid, "model_to_motor_index must have one value per direct model DOF");
    invalid = config;
    invalid.model_to_motor_index[0] = 12;
    rejects(12, invalid, "model_to_motor_index contains an out-of-range motor index");
}
