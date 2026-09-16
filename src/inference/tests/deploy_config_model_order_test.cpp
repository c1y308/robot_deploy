#include "config/deploy_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
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

    const YAML::Node raw_clip = root["actions"]["JointPositionAction"]["raw_action_clip"];
    if (raw_clip.IsNull()) {
        expect(!config.action.raw_action_clip.has_value(),
               "raw_action_clip: null must disable raw action clipping");
    } else {
        expect(config.action.raw_action_clip.has_value() &&
                   std::abs(*config.action.raw_action_clip - raw_clip.as<double>()) < 1e-12,
               "raw_action_clip must match the deployment config");
    }

    // Test explicit null independently of the current deployment's clip value.
    YAML::Node null_clip_config = YAML::Clone(root);
    null_clip_config["actions"]["JointPositionAction"]["raw_action_clip"] =
        YAML::Node(YAML::NodeType::Null);
    const auto null_clip_path = std::filesystem::temp_directory_path() /
        ("deploy-null-clip-" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
    {
        std::ofstream file(null_clip_path);
        file << null_clip_config;
        expect(file.good(), "failed to write null clip fixture");
    }
    inference::RobotInterfaceConfig null_clip_result;
    const bool null_loaded = inference::load_deploy_config(null_clip_path.string(), null_clip_result, error);
    std::filesystem::remove(null_clip_path);
    expect(null_loaded, "failed to load null clip fixture: " + error);
    expect(!null_clip_result.action.raw_action_clip.has_value(),
           "raw_action_clip: null must disable raw action clipping");
    expect(config.policy.gait.enabled,
           "gait_phase in deploy config must enable the 705-dimension layout");

    std::cout << "deploy_config_model_order_test passed\n";
    return 0;
}
