#include "config/deploy_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void expect(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

}  // namespace

int main()
{
    inference::RobotInterfaceConfig config;
    std::string error;
    expect(inference::load_deploy_config(ROBOT_DEPLOY_CONFIG_PATH,
                                         config,
                                         error),
           "failed to load deploy config: " + error);

    const YAML::Node root = YAML::LoadFile(ROBOT_DEPLOY_CONFIG_PATH);
    const YAML::Node joint_ids_map = root["joint_ids_map"];
    const YAML::Node stiffness = root["stiffness"];
    const YAML::Node damping = root["damping"];
    expect(joint_ids_map.size() == inference::policy_observation::kDof,
           "joint_ids_map size mismatch");

    for (std::size_t model_index = 0;
         model_index < inference::policy_observation::kDof;
         ++model_index) {
        const auto motor_index = joint_ids_map[model_index].as<std::size_t>();
        expect(std::abs(config.motor.mit_kp[motor_index] -
                        stiffness[model_index].as<double>()) < 1e-12,
               "stiffness was not converted from model to motor order");
        expect(std::abs(config.motor.mit_kd[motor_index] -
                        damping[model_index].as<double>()) < 1e-12,
               "damping was not converted from model to motor order");
    }

    expect(!config.action.raw_action_clip.has_value(),
           "raw_action_clip: null must disable raw action clipping");
    expect(config.policy.gait.enabled,
           "gait_phase in deploy config must enable the 705-dimension layout");

    std::cout << "deploy_config_model_order_test passed\n";
    return 0;
}
