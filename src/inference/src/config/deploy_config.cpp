#include "config/deploy_config.hpp"
#include "policy/policy_observation_config.hpp"

#include "tool/tool.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <set>
#include <string>
#include <utility>
#include <vector>

#ifndef ROBOT_INFERENCE_ROOT_DIR
#define ROBOT_INFERENCE_ROOT_DIR ""
#endif

namespace inference {

namespace {

using robot_base::finite_array;
using robot_base::index_in_range;

constexpr std::size_t kDirectDriveDofCount = kDof - 4;
constexpr double kValueTolerance = 1e-12;

class DeployConfigLoader {
public:
    DeployConfigLoader(std::string root_dir, RobotInterfaceConfig& config)
        : root_dir_(std::move(root_dir)), config_(config)
    {
    }

    bool load(const std::string& yaml_path, std::string& error)
    {
        YAML::Node root;
        try {
            root = YAML::LoadFile(yaml_path);
        } catch (const YAML::Exception& exception) {
            error = "failed to parse deploy config " + yaml_path + ": " +
                    exception.what();
            return false;
        }
        if (!root.IsMap()) {
            error = "deploy config root must be a map: " + yaml_path;
            return false;
        }

        config_ = RobotInterfaceConfig{};
        config_.policy.model_path = resolve_path(config_.policy.model_path).string();
        config_.recorder.directory = resolve_path(config_.recorder.directory.string());

        if (!check_known_keys(root,
                              {"joint_ids_map", "step_dt", "stiffness",
                               "damping", "default_joint_pos", "commands",
                               "actions", "observations"},
                              "root", error)) {
            return false;
        }

        return load_joint_topology(root, error) &&
               read_value(root, "step_dt", "root", config_.policy.step_dt, error) &&
               load_motor_gains(root, error) &&
               load_default_joint_pos(root, error) &&
               load_commands(root, error) &&
               load_action(root, error) &&
               load_observations(root, error);
    }

private:
    static bool fail(const std::string& message, std::string& error)
    {
        error = message;
        return false;
    }

    // node 已由根节点检查或 read_map() 确认为 Map。
    static bool check_known_keys(const YAML::Node& node,
                                 const std::set<std::string>& allowed,
                                 const std::string& node_path,
                                 std::string& error)
    {
        for (const auto& kv : node) {
            std::string key;
            try {
                key = kv.first.as<std::string>();
            } catch (const YAML::BadConversion&) {
                return fail(node_path + " contains a non-string key", error);
            }
            if (allowed.find(key) == allowed.end()) {
                return fail(node_path + " contains unknown key: " + key, error);
            }
        }
        return true;
    }

    static bool read_map(const YAML::Node& parent_node,
                         const char* key,
                         const std::string& node_path,
                         YAML::Node& output,
                         std::string& error)
    {
        const YAML::Node node = parent_node[key];
        if (!node.IsDefined() || !node.IsMap()) {
            return fail(node_path + " missing required map key: " + key, error);
        }
        output = node;
        return true;
    }

    template <typename T>
    static bool read_value(const YAML::Node& parent_node,
                           const char* key,
                           const std::string& node_path,
                           T& output,
                           std::string& error)
    {
        const YAML::Node node = parent_node[key];
        if (!node.IsDefined() || !node.IsScalar()) {
            return fail(node_path + " missing required scalar key: " + key, error);
        }
        try {
            output = node.as<T>();
        } catch (const YAML::BadConversion&) {
            return fail(node_path + " key " + key + " has the wrong type", error);
        }
        return true;
    }

    template <std::size_t N>
    static bool read_double_array(const YAML::Node& parent_node,
                                  const char* key,
                                  const std::string& node_path,
                                  std::array<double, N>& output,
                                  std::string& error)
    {
        const YAML::Node node = parent_node[key];
        if (!node.IsDefined() || !node.IsSequence()) {
            return fail(node_path + " missing required sequence key: " + key, error);
        }
        if (node.size() != N) {
            return fail(node_path + " key " + key + " must have " +
                        std::to_string(N) + " elements, got " +
                        std::to_string(node.size()), error);
        }
        for (std::size_t i = 0; i < N; ++i) {
            try {
                output[i] = node[i].as<double>();
            } catch (const YAML::BadConversion&) {
                return fail(node_path + " key " + key + " element " +
                            std::to_string(i) + " is not a number", error);
            }
        }
        return true;
    }

    static bool read_int_vector(const YAML::Node& parent_node,
                                const char* key,
                                const std::string& node_path,
                                std::vector<int>& output,
                                std::string& error)
    {
        const YAML::Node node = parent_node[key];
        if (!node.IsDefined() || !node.IsSequence()) {
            return fail(node_path + " missing required sequence key: " + key, error);
        }
        output.clear();
        output.reserve(node.size());
        for (std::size_t i = 0; i < node.size(); ++i) {
            try {
                output.push_back(node[i].as<int>());
            } catch (const YAML::BadConversion&) {
                return fail(node_path + " key " + key + " element " +
                            std::to_string(i) + " is not an integer", error);
            }
        }
        return true;
    }

    static bool read_double_pair(const YAML::Node& parent_node,
                                 const char* key,
                                 const std::string& node_path,
                                 std::array<double, 2>& output,
                                 std::string& error)
    {
        const YAML::Node node = parent_node[key];
        if (!node.IsDefined() || !node.IsSequence()) {
            return fail(node_path + " missing required sequence key: " + key, error);
        }
        if (node.size() != 2) {
            return fail(node_path + " key " + key + " must be [min, max]", error);
        }
        try {
            output[0] = node[0].as<double>();
            output[1] = node[1].as<double>();
        } catch (const YAML::BadConversion&) {
            return fail(node_path + " key " + key + " must be two numbers", error);
        }
        if (!std::isfinite(output[0]) || !std::isfinite(output[1])) {
            return fail(node_path + " key " + key + " values must be finite", error);
        }
        if (output[0] > output[1]) {
            return fail(node_path + " key " + key + " min must be <= max", error);
        }
        return true;
    }

    std::filesystem::path resolve_path(const std::string& raw) const
    {
        std::filesystem::path path(raw);
        if (path.is_absolute() || root_dir_.empty()) {
            return path;
        }
        return std::filesystem::path(root_dir_) / path;
    }

    bool load_joint_topology(const YAML::Node& root, std::string& error)
    {
        std::vector<int> joint_ids_map;
        if (!read_int_vector(root, "joint_ids_map", "root",
                             joint_ids_map, error)) {
            return false;
        }
        if (joint_ids_map.size() != kDof) {
            return fail("joint_ids_map must have " + std::to_string(kDof) +
                        " elements, got " + std::to_string(joint_ids_map.size()),
                        error);
        }

        std::vector<unsigned char> seen(motor_base::kMotorNum, 0U);
        for (std::size_t i = 0; i < joint_ids_map.size(); ++i) {
            const int motor_index = joint_ids_map[i];
            if (!index_in_range(motor_index, static_cast<int>(motor_base::kMotorNum))) {
                return fail("joint_ids_map element " + std::to_string(i) +
                            " is out of motor range [0, " +
                            std::to_string(motor_base::kMotorNum) + "): " +
                            std::to_string(motor_index), error);
            }
            auto& slot = seen[static_cast<std::size_t>(motor_index)];
            if (slot != 0U) {
                return fail("joint_ids_map must be a permutation; duplicate motor " +
                            std::to_string(motor_index), error);
            }
            slot = 1U;
        }

        // 模型末四轴依次为左 pitch、右 pitch、左 roll、右 roll；
        // 对应的物理电机依次为左上、右上、左下、右下，由 YAML 指定。
        auto& left = config_.joint_mapping.left_ankle;
        auto& right = config_.joint_mapping.right_ankle;
        left.upper_motor_index = joint_ids_map[left.model_pitch_dof];
        left.lower_motor_index = joint_ids_map[left.model_roll_dof];
        right.upper_motor_index = joint_ids_map[right.model_pitch_dof];
        right.lower_motor_index = joint_ids_map[right.model_roll_dof];

        std::copy(joint_ids_map.begin(), joint_ids_map.end(),
                  model_to_motor_index_.begin());

        config_.joint_mapping.model_to_motor_index.assign(
            joint_ids_map.begin(),
            joint_ids_map.begin() + kDirectDriveDofCount);

        return true;
    }

    bool load_motor_gains(const YAML::Node& root, std::string& error)
    {
        std::array<double, kDof> model_kp{};
        std::array<double, kDof> model_kd{};
        if (!read_double_array(root, "stiffness", "root",
                               model_kp, error) ||
            !read_double_array(root, "damping", "root",
                               model_kd, error)) {
            return false;
        }
        if (!finite_array(model_kp) || !finite_array(model_kd)) {
            return fail("stiffness/damping values must be finite", error);
        }

        for (std::size_t model_index = 0; model_index < kDof; ++model_index) {
            if (model_kp[model_index] < 0.0 || model_kd[model_index] < 0.0) {
                return fail("stiffness/damping values must be non-negative",
                            error);
            }

            const auto motor_index = static_cast<std::size_t>(
                model_to_motor_index_[model_index]);
            config_.motor.mit_kp[motor_index] = model_kp[model_index];
            config_.motor.mit_kd[motor_index] = model_kd[model_index];
        }

        const double ankle_kp = model_kp[kDirectDriveDofCount];
        const double ankle_kd = model_kd[kDirectDriveDofCount];
        for (std::size_t model_index = kDirectDriveDofCount + 1;
             model_index < kDof; ++model_index) {
            if (std::abs(model_kp[model_index] - ankle_kp) > kValueTolerance ||
                std::abs(model_kd[model_index] - ankle_kd) > kValueTolerance) {
                return fail("stiffness/damping for ankle motors must be identical", error);
            }
        }

        config_.ankle.virtual_kp = {ankle_kp, ankle_kp};
        config_.ankle.virtual_kd = {ankle_kd, ankle_kd};
        return true;
    }

    bool load_default_joint_pos(const YAML::Node& root, std::string& error)
    {
        if (!read_double_array(root, "default_joint_pos",
                               "root", config_.default_joint_pos_rad, error)) {
            return false;
        }
        if (!finite_array(config_.default_joint_pos_rad)) {
            return fail("default_joint_pos values must be finite", error);
        }
        return true;
    }

    bool load_commands(const YAML::Node& root, std::string& error)
    {
        YAML::Node commands;
        if (!read_map(root, "commands", "root", commands, error)) {
            return false;
        }
        // commands 节点下只允许有 base_velocity 子节点
        if (!check_known_keys(commands, {"base_velocity"},
                              "commands", error)) {
            return false;
        }

        YAML::Node base_velocity;
        if (!read_map(commands, "base_velocity", "commands",
                      base_velocity, error)) {
            return false;
        }
        if (!check_known_keys(base_velocity, {"ranges"},
                              "commands.base_velocity", error)) {
            return false;
        }

        YAML::Node ranges;
        if (!read_map(base_velocity, "ranges", "commands.base_velocity",
                      ranges, error)) {
            return false;
        }
        // heading 仅保留为兼容键，不读取或校验其值。
        if (!check_known_keys(ranges, {"lin_vel_x", "lin_vel_y",
                                       "ang_vel_z", "heading"},
                              "commands.base_velocity.ranges", error)) {
            return false;
        }
        const auto read_range = [&](const char* key,
                                    std::optional<std::array<double, 2>>& output) {
            std::array<double, 2> range;
            if (!read_double_pair(ranges, key, "commands.base_velocity.ranges", range, error)) {
                return false;
            }
            output = range;
            return true;
        };
        return read_range("lin_vel_x", config_.command_ranges.lin_vel_x) &&
               read_range("lin_vel_y", config_.command_ranges.lin_vel_y) &&
               read_range("ang_vel_z", config_.command_ranges.ang_vel_z);
    }

    bool load_action(const YAML::Node& root, std::string& error)
    {
        YAML::Node actions;
        if (!read_map(root, "actions", "root", actions, error)) {
            return false;
        }
        if (!check_known_keys(actions, {"JointPositionAction"},
                              "actions", error)) {
            return false;
        }

        YAML::Node action;
        if (!read_map(actions, "JointPositionAction", "actions",
                      action, error)) {
            return false;
        }
        // joint_names、joint_ids、offset 和 delay 字段保留为兼容键，不读取或校验其值。
        if (!check_known_keys(action, {"clip", "joint_names", "scale",
                                       "offset", "raw_action_clip",
                                       "min_inference_delay",
                                       "max_inference_delay",
                                       "min_communication_delay",
                                       "max_communication_delay",
                                       "joint_ids"},
                              "actions.JointPositionAction", error)) {
            return false;
        }

        if (!load_action_clip(action, error) ||
            !read_double_array(action, "scale",
                               "actions.JointPositionAction",
                               config_.action.action_scale, error)) {
            return false;
        }
        if (!finite_array(config_.action.action_scale)) {
            return fail("actions.JointPositionAction scale values must be finite",
                        error);
        }
        for (double scale : config_.action.action_scale) {
            if (scale <= 0.0) {
                return fail("actions.JointPositionAction scale must be > 0 "
                            "for every model DOF", error);
            }
        }

        const YAML::Node raw_action_clip = action["raw_action_clip"];
        if (!raw_action_clip.IsDefined()) {
            return fail("actions.JointPositionAction missing required key: "
                        "raw_action_clip", error);
        }
        if (raw_action_clip.IsNull()) {
            config_.action.raw_action_clip.reset();
        } else {
            double clip = 0.0;
            if (!read_value(action, "raw_action_clip", "actions.JointPositionAction",
                            clip, error)) {
                return false;
            }
            config_.action.raw_action_clip = clip;
        }

        return true;
    }

    bool load_action_clip(const YAML::Node& action, std::string& error)
    {
        const YAML::Node clip = action["clip"];
        if (!clip.IsDefined() || !clip.IsSequence()) {
            return fail("actions.JointPositionAction missing required "
                        "sequence key: clip", error);
        }
        if (clip.size() != kDof) {
            return fail("actions.JointPositionAction.clip must have " +
                        std::to_string(kDof) + " ranges, got " +
                        std::to_string(clip.size()), error);
        }

        for (std::size_t i = 0; i < kDof; ++i) {
            const YAML::Node range = clip[i];
            if (!range.IsSequence() || range.size() != 2) {
                return fail("actions.JointPositionAction.clip[" +
                            std::to_string(i) + "] must be [lower, upper]",
                            error);
            }
            try {
                config_.action.action_clip[i][0] = range[0].as<double>();
                config_.action.action_clip[i][1] = range[1].as<double>();
            } catch (const YAML::BadConversion&) {
                return fail("actions.JointPositionAction.clip[" +
                            std::to_string(i) + "] must be two numbers",
                            error);
            }
            if (!std::isfinite(config_.action.action_clip[i][0]) ||
                !std::isfinite(config_.action.action_clip[i][1])) {
                return fail("actions.JointPositionAction.clip values must be finite",
                            error);
            }
            if (config_.action.action_clip[i][0] >
                config_.action.action_clip[i][1]) {
                return fail("actions.JointPositionAction.clip lower bound "
                            "must be <= upper bound for every model DOF",
                            error);
            }
        }
        return true;
    }

    bool load_observations(const YAML::Node& root, std::string& error)
    {
        YAML::Node observations;
        if (!read_map(root, "observations", "root", observations, error)) {
            return false;
        }
        if (!check_known_keys(observations, {"base_ang_vel",
                                             "projected_gravity",
                                             "velocity_commands",
                                             "gait_phase",
                                             "joint_pos_rel",
                                             "joint_vel_rel",
                                             "last_action"},
                              "observations", error)) {
            return false;
        }

        const YAML::Node gait_phase = observations["gait_phase"];
        config_.policy.gait.enabled = gait_phase.IsDefined();
        if (config_.policy.gait.enabled) {
            if (!load_observation_item(observations, "gait_phase",
                                       config_.observation_scales.gait_phase_scale, error)) {
                return false;
            }
            YAML::Node params;
            if (!read_map(gait_phase, "params", "observations.gait_phase", params, error) ||
                !read_value(params, "period", "observations.gait_phase.params",
                            config_.policy.gait.period, error)) {
                return false;
            }
        }

        if (!load_observation_item(observations, "base_ang_vel",
                                   config_.observation_scales.body_ang_vel_scale, error) ||
            !load_observation_item(observations, "projected_gravity",
                                   config_.observation_scales.projected_gravity_scale, error) ||
            !load_observation_item(observations, "velocity_commands",
                                   config_.observation_scales.command_scale, error) ||
            !load_observation_item(observations, "joint_pos_rel",
                                   config_.observation_scales.dof_pos_scale, error) ||
            !load_observation_item(observations, "joint_vel_rel",
                                   config_.observation_scales.dof_vel_scale, error) ||
            !load_observation_item(observations, "last_action",
                                   config_.observation_scales.last_action_scale, error)) {
            return false;
        }
        return true;
    }

    template <std::size_t N>
    bool load_observation_item(const YAML::Node& observations,
                               const char* name,
                               std::array<double, N>& output,
                               std::string& error)
    {
        const std::string node_path = std::string("observations.") + name;
        YAML::Node item;
        if (!read_map(observations, name, "observations", item, error)) {
            return false;
        }
        // params 由对应观测分支按需读取；clip 仅保留为兼容键。
        if (!check_known_keys(item, {"params", "clip", "scale",
                                     "history_length"},
                              node_path, error)) {
            return false;
        }

        if (!read_double_array(item, "scale", node_path, output, error)) {
            return false;
        }
        if (!finite_array(output)) {
            return fail(node_path + " scale values must be finite", error);
        }

        int history_length = 0;
        if (!read_value(item, "history_length", node_path, history_length, error)) {
            return false;
        }
        if (history_length != static_cast<int>(policy_observation::kFrameStack)) {
            return fail(node_path + " history_length must equal the compiled "
                        "frame stack (" +
                        std::to_string(policy_observation::kFrameStack) +
                        "), got " + std::to_string(history_length), error);
        }
        return true;
    }

    std::string root_dir_;
    RobotInterfaceConfig& config_;
    std::array<int, kDof> model_to_motor_index_{};
};

}  // namespace

bool load_deploy_config(const std::string& yaml_path,
                        RobotInterfaceConfig& config,
                        std::string& error)
{
    DeployConfigLoader loader(ROBOT_INFERENCE_ROOT_DIR, config);
    return loader.load(yaml_path, error);
}

}  // namespace inference
