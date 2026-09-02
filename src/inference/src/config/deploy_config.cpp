#include "config/deploy_config.hpp"

#include "robot/joint_mapping.hpp"
#include "tool/tool.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <memory>
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

constexpr std::size_t kDof = policy_observation::kDof;
constexpr std::size_t kMotorCount = motor_base::kMaxMotors;
constexpr std::size_t kDirectDriveDofCount = kDof - 4;
constexpr double kValueTolerance = 1e-12;

bool finite_positive(double value)
{
    return std::isfinite(value) && value > 0.0;
}

template <std::size_t N>
bool all_near(const std::array<double, N>& values, double expected)
{
    return std::all_of(values.begin(), values.end(), [expected](double value) {
        return std::isfinite(value) &&
               std::abs(value - expected) <= kValueTolerance;
    });
}

template <std::size_t N>
bool arrays_near(const std::array<double, N>& lhs,
                 const std::array<double, N>& rhs)
{
    for (std::size_t i = 0; i < N; ++i) {
        if (std::abs(lhs[i] - rhs[i]) > kValueTolerance) {
            return false;
        }
    }
    return true;
}

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
        resolve_runtime_default_paths();

        if (!check_known_keys(root,
                              {"joint_ids_map", "step_dt", "stiffness",
                               "damping", "default_joint_pos", "commands",
                               "actions", "observations"},
                              "root", error)) {
            return false;
        }

        return load_joint_topology(root, error) &&
               load_step_dt(root, error) &&
               load_motor_gains(root, error) &&
               load_default_joint_pos(root, error) &&
               load_commands(root, error) &&
               load_action(root, error) &&
               load_observations(root, error);
    }

private:
    enum class ParamsRule {
        Empty,
        BaseVelocityCommand,
    };

    static bool fail(const std::string& message, std::string& error)
    {
        error = message;
        return false;
    }

    static bool check_known_keys(const YAML::Node& node,
                                 const std::set<std::string>& allowed,
                                 const std::string& where,
                                 std::string& error)
    {
        if (!node.IsMap()) {
            return fail(where + " must be a map", error);
        }
        for (const auto& kv : node) {
            std::string key;
            try {
                key = kv.first.as<std::string>();
            } catch (const YAML::BadConversion&) {
                return fail(where + " contains a non-string key", error);
            }
            if (allowed.find(key) == allowed.end()) {
                return fail(where + " contains unknown key: " + key, error);
            }
        }
        return true;
    }

    static bool require_map(const YAML::Node& parent,
                            const char* key,
                            YAML::Node& out,
                            const std::string& where,
                            std::string& error)
    {
        const YAML::Node node = parent[key];
        if (!node.IsDefined() || !node.IsMap()) {
            return fail(where + " missing required map key: " + key, error);
        }
        out = node;
        return true;
    }

    template <typename T>
    static bool require_value(const YAML::Node& parent,
                              const char* key,
                              T& out,
                              const std::string& where,
                              std::string& error)
    {
        const YAML::Node node = parent[key];
        if (!node.IsDefined() || !node.IsScalar()) {
            return fail(where + " missing required scalar key: " + key, error);
        }
        try {
            out = node.as<T>();
        } catch (const YAML::BadConversion&) {
            return fail(where + " key " + key + " has the wrong type", error);
        }
        return true;
    }

    static bool require_null(const YAML::Node& parent,
                             const char* key,
                             const std::string& where,
                             std::string& error)
    {
        const YAML::Node node = parent[key];
        if (!node.IsDefined() || !node.IsNull()) {
            return fail(where + " key " + key + " must be null", error);
        }
        return true;
    }

    template <std::size_t N>
    static bool require_double_array(const YAML::Node& parent,
                                     const char* key,
                                     std::array<double, N>& out,
                                     const std::string& where,
                                     std::string& error)
    {
        const YAML::Node node = parent[key];
        if (!node.IsDefined() || !node.IsSequence()) {
            return fail(where + " missing required sequence key: " + key, error);
        }
        if (node.size() != N) {
            return fail(where + " key " + key + " must have " +
                        std::to_string(N) + " elements, got " +
                        std::to_string(node.size()), error);
        }
        for (std::size_t i = 0; i < N; ++i) {
            try {
                out[i] = node[i].as<double>();
            } catch (const YAML::BadConversion&) {
                return fail(where + " key " + key + " element " +
                            std::to_string(i) + " is not a number", error);
            }
        }
        return true;
    }

    static bool require_int_vector(const YAML::Node& parent,
                                   const char* key,
                                   std::vector<int>& out,
                                   const std::string& where,
                                   std::string& error)
    {
        const YAML::Node node = parent[key];
        if (!node.IsDefined() || !node.IsSequence()) {
            return fail(where + " missing required sequence key: " + key, error);
        }
        out.clear();
        out.reserve(node.size());
        for (std::size_t i = 0; i < node.size(); ++i) {
            try {
                out.push_back(node[i].as<int>());
            } catch (const YAML::BadConversion&) {
                return fail(where + " key " + key + " element " +
                            std::to_string(i) + " is not an integer", error);
            }
        }
        return true;
    }

    static bool require_double_pair(const YAML::Node& parent,
                                    const char* key,
                                    std::array<double, 2>& out,
                                    const std::string& where,
                                    std::string& error)
    {
        const YAML::Node node = parent[key];
        if (!node.IsDefined() || !node.IsSequence()) {
            return fail(where + " missing required sequence key: " + key, error);
        }
        if (node.size() != 2) {
            return fail(where + " key " + key + " must be [min, max]", error);
        }
        try {
            out[0] = node[0].as<double>();
            out[1] = node[1].as<double>();
        } catch (const YAML::BadConversion&) {
            return fail(where + " key " + key + " must be two numbers", error);
        }
        if (!std::isfinite(out[0]) || !std::isfinite(out[1])) {
            return fail(where + " key " + key + " values must be finite", error);
        }
        if (out[0] > out[1]) {
            return fail(where + " key " + key + " min must be <= max", error);
        }
        return true;
    }

    static bool require_finite_positive(const YAML::Node& parent,
                                        const char* key,
                                        double& out,
                                        const std::string& where,
                                        std::string& error)
    {
        if (!require_value(parent, key, out, where, error)) {
            return false;
        }
        if (!finite_positive(out)) {
            return fail(where + " key " + key +
                        " must be a finite positive value", error);
        }
        return true;
    }

    static bool require_nonnegative_int(const YAML::Node& parent,
                                        const char* key,
                                        int& out,
                                        const std::string& where,
                                        std::string& error)
    {
        if (!require_value(parent, key, out, where, error)) {
            return false;
        }
        if (out < 0) {
            return fail(where + " key " + key + " must be non-negative", error);
        }
        return true;
    }

    static bool require_string_sequence(const YAML::Node& parent,
                                        const char* key,
                                        const std::string& where,
                                        std::string& error)
    {
        const YAML::Node node = parent[key];
        if (!node.IsDefined() || !node.IsSequence()) {
            return fail(where + " missing required sequence key: " + key, error);
        }
        if (node.size() == 0) {
            return fail(where + " key " + key + " must not be empty", error);
        }
        for (std::size_t i = 0; i < node.size(); ++i) {
            if (!node[i].IsScalar()) {
                return fail(where + " key " + key + " element " +
                            std::to_string(i) + " must be a string", error);
            }
            try {
                (void)node[i].as<std::string>();
            } catch (const YAML::BadConversion&) {
                return fail(where + " key " + key + " element " +
                            std::to_string(i) + " must be a string", error);
            }
        }
        return true;
    }

    static bool validate_delay_range(const YAML::Node& action,
                                     const char* min_key,
                                     const char* max_key,
                                     const std::string& where,
                                     std::string& error)
    {
        int min_value = 0;
        int max_value = 0;
        if (!require_nonnegative_int(action, min_key, min_value, where, error) ||
            !require_nonnegative_int(action, max_key, max_value, where, error)) {
            return false;
        }
        if (min_value > max_value) {
            return fail(where + " key " + min_key + " must be <= " + max_key,
                        error);
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

    void resolve_runtime_default_paths()
    {
        config_.policy.model_path =
            resolve_path(config_.policy.model_path).string();
        config_.recorder.directory =
            resolve_path(config_.recorder.directory.string());
    }

    bool load_joint_topology(const YAML::Node& root, std::string& error)
    {
        std::vector<int> joint_ids_map;
        if (!require_int_vector(root, "joint_ids_map", joint_ids_map,
                                "root", error)) {
            return false;
        }
        if (joint_ids_map.size() != kDof) {
            return fail("joint_ids_map must have " + std::to_string(kDof) +
                        " elements, got " + std::to_string(joint_ids_map.size()),
                        error);
        }

        std::vector<unsigned char> seen(kMotorCount, 0U);
        for (std::size_t i = 0; i < joint_ids_map.size(); ++i) {
            const int motor_index = joint_ids_map[i];
            if (!index_in_range(motor_index, static_cast<int>(kMotorCount))) {
                return fail("joint_ids_map element " + std::to_string(i) +
                            " is out of motor range [0, " +
                            std::to_string(kMotorCount) + "): " +
                            std::to_string(motor_index), error);
            }
            auto& slot = seen[static_cast<std::size_t>(motor_index)];
            if (slot != 0U) {
                return fail("joint_ids_map must be a permutation; duplicate motor " +
                            std::to_string(motor_index), error);
            }
            slot = 1U;
        }

        const auto& left = config_.joint_mapping.left_ankle_parallel;
        const auto& right = config_.joint_mapping.right_ankle_parallel;
        const std::array<int, 4> declared_ankle_motors = {
            left.upper_motor_index, left.lower_motor_index,
            right.upper_motor_index, right.lower_motor_index
        };
        std::vector<int> map_ankle_motors(joint_ids_map.begin() + kDof - 4,
                                          joint_ids_map.end());
        std::sort(map_ankle_motors.begin(), map_ankle_motors.end());
        std::array<int, 4> sorted_declared = declared_ankle_motors;
        std::sort(sorted_declared.begin(), sorted_declared.end());
        if (!std::equal(map_ankle_motors.begin(), map_ankle_motors.end(),
                        sorted_declared.begin())) {
            return fail("joint_ids_map last 4 entries must be exactly the "
                        "default ankle motors", error);
        }

        config_.joint_mapping.model_to_motor_index.assign(
            joint_ids_map.begin(),
            joint_ids_map.begin() + kDirectDriveDofCount);

        std::string mapping_error;
        mapping_ = robot_detail::JointMapping::create(
            static_cast<int>(kDof), config_.joint_mapping, mapping_error);
        if (!mapping_) {
            return fail("joint mapping is invalid: " + mapping_error, error);
        }
        return true;
    }

    bool load_step_dt(const YAML::Node& root, std::string& error)
    {
        return require_finite_positive(root, "step_dt",
                                       config_.policy.step_dt,
                                       "root", error);
    }

    bool load_motor_gains(const YAML::Node& root, std::string& error)
    {
        if (!require_double_array(root, "stiffness", config_.motor.mit_kp,
                                  "root", error) ||
            !require_double_array(root, "damping", config_.motor.mit_kd,
                                  "root", error)) {
            return false;
        }
        if (!finite_array(config_.motor.mit_kp) ||
            !finite_array(config_.motor.mit_kd)) {
            return fail("stiffness/damping values must be finite", error);
        }

        for (std::size_t i = 0; i < kMotorCount; ++i) {
            if (config_.motor.mit_kp[i] < 0.0 ||
                config_.motor.mit_kd[i] < 0.0) {
                return fail("stiffness/damping values must be non-negative",
                            error);
            }
        }

        const std::array<int, 4> ankle_motors = {
            config_.joint_mapping.left_ankle_parallel.upper_motor_index,
            config_.joint_mapping.left_ankle_parallel.lower_motor_index,
            config_.joint_mapping.right_ankle_parallel.upper_motor_index,
            config_.joint_mapping.right_ankle_parallel.lower_motor_index
        };
        const double ankle_kp =
            config_.motor.mit_kp[static_cast<std::size_t>(ankle_motors[0])];
        const double ankle_kd =
            config_.motor.mit_kd[static_cast<std::size_t>(ankle_motors[0])];
        for (int motor_index : ankle_motors) {
            const auto slot = static_cast<std::size_t>(motor_index);
            if (std::abs(config_.motor.mit_kp[slot] - ankle_kp) >
                    kValueTolerance ||
                std::abs(config_.motor.mit_kd[slot] - ankle_kd) >
                    kValueTolerance) {
                return fail("stiffness/damping for ankle motors "
                            "M4/M5/M10/M11 must be identical", error);
            }
        }

        config_.ankle_torque.virtual_kp = {ankle_kp, ankle_kp};
        config_.ankle_torque.virtual_kd = {ankle_kd, ankle_kd};
        return true;
    }

    bool load_default_joint_pos(const YAML::Node& root, std::string& error)
    {
        if (!require_double_array(root, "default_joint_pos",
                                  root_default_joint_pos_, "root", error)) {
            return false;
        }
        if (!finite_array(root_default_joint_pos_)) {
            return fail("default_joint_pos values must be finite", error);
        }
        config_.action.default_joint_pos_rad = root_default_joint_pos_;
        return true;
    }

    bool load_commands(const YAML::Node& root, std::string& error)
    {
        YAML::Node commands;
        if (!require_map(root, "commands", commands, "root", error)) {
            return false;
        }
        if (!check_known_keys(commands, {"base_velocity"},
                              "commands", error)) {
            return false;
        }

        YAML::Node base_velocity;
        if (!require_map(commands, "base_velocity", base_velocity,
                         "commands", error)) {
            return false;
        }
        if (!check_known_keys(base_velocity, {"ranges"},
                              "commands.base_velocity", error)) {
            return false;
        }

        YAML::Node ranges;
        if (!require_map(base_velocity, "ranges", ranges,
                         "commands.base_velocity", error)) {
            return false;
        }
        if (!check_known_keys(ranges, {"lin_vel_x", "lin_vel_y",
                                       "ang_vel_z", "heading"},
                              "commands.base_velocity.ranges", error)) {
            return false;
        }
        std::array<double, 2> ignored_range{};
        return require_double_pair(ranges, "lin_vel_x", ignored_range,
                                   "commands.base_velocity.ranges", error) &&
               require_double_pair(ranges, "lin_vel_y", ignored_range,
                                   "commands.base_velocity.ranges", error) &&
               require_double_pair(ranges, "ang_vel_z", ignored_range,
                                   "commands.base_velocity.ranges", error) &&
               require_null(ranges, "heading",
                            "commands.base_velocity.ranges", error);
    }

    bool load_action(const YAML::Node& root, std::string& error)
    {
        YAML::Node actions;
        if (!require_map(root, "actions", actions, "root", error)) {
            return false;
        }
        if (!check_known_keys(actions, {"JointPositionAction"},
                              "actions", error)) {
            return false;
        }

        YAML::Node action;
        if (!require_map(actions, "JointPositionAction", action,
                         "actions", error)) {
            return false;
        }
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
            !require_string_sequence(action, "joint_names",
                                     "actions.JointPositionAction", error) ||
            !require_double_array(action, "scale",
                                  config_.action.action_scale,
                                  "actions.JointPositionAction", error)) {
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

        std::array<double, kDof> action_offset{};
        if (!require_double_array(action, "offset", action_offset,
                                  "actions.JointPositionAction", error)) {
            return false;
        }
        if (!finite_array(action_offset)) {
            return fail("actions.JointPositionAction offset values must be finite",
                        error);
        }
        if (!arrays_near(action_offset, root_default_joint_pos_)) {
            return fail("default_joint_pos must match "
                        "actions.JointPositionAction.offset", error);
        }
        config_.action.default_joint_pos_rad = action_offset;

        if (!require_finite_positive(action, "raw_action_clip",
                                     config_.action.raw_action_clip,
                                     "actions.JointPositionAction", error) ||
            !validate_delay_range(action, "min_inference_delay",
                                  "max_inference_delay",
                                  "actions.JointPositionAction", error) ||
            !validate_delay_range(action, "min_communication_delay",
                                  "max_communication_delay",
                                  "actions.JointPositionAction", error) ||
            !require_null(action, "joint_ids",
                          "actions.JointPositionAction", error)) {
            return false;
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
        if (!require_map(root, "observations", observations, "root", error)) {
            return false;
        }
        if (!check_known_keys(observations, {"base_ang_vel",
                                             "projected_gravity",
                                             "velocity_commands",
                                             "joint_pos_rel",
                                             "joint_vel_rel",
                                             "last_action"},
                              "observations", error)) {
            return false;
        }

        std::array<double, 3> projected_gravity_scale{};
        std::array<double, kDof> last_action_scale{};
        if (!load_observation_item(observations, "base_ang_vel",
                                   config_.observation_scales.body_ang_vel_scale,
                                   ParamsRule::Empty, error) ||
            !load_observation_item(observations, "projected_gravity",
                                   projected_gravity_scale,
                                   ParamsRule::Empty, error) ||
            !load_observation_item(observations, "velocity_commands",
                                   config_.observation_scales.command_scale,
                                   ParamsRule::BaseVelocityCommand, error) ||
            !load_observation_item(observations, "joint_pos_rel",
                                   config_.observation_scales.dof_pos_scale,
                                   ParamsRule::Empty, error) ||
            !load_observation_item(observations, "joint_vel_rel",
                                   config_.observation_scales.dof_vel_scale,
                                   ParamsRule::Empty, error) ||
            !load_observation_item(observations, "last_action",
                                   last_action_scale,
                                   ParamsRule::Empty, error)) {
            return false;
        }

        if (!all_near(projected_gravity_scale, 1.0)) {
            return fail("observations.projected_gravity scale must be all 1.0 "
                        "because runtime does not apply a configurable scale",
                        error);
        }
        if (!all_near(last_action_scale, 1.0)) {
            return fail("observations.last_action scale must be all 1.0 "
                        "because runtime uses raw last actions directly",
                        error);
        }
        return true;
    }

    template <std::size_t N>
    bool load_observation_item(const YAML::Node& observations,
                               const char* name,
                               std::array<double, N>& scale_out,
                               ParamsRule params_rule,
                               std::string& error)
    {
        const std::string where = std::string("observations.") + name;
        YAML::Node item;
        if (!require_map(observations, name, item, "observations", error)) {
            return false;
        }
        if (!check_known_keys(item, {"params", "clip", "scale",
                                     "history_length"},
                              where, error)) {
            return false;
        }

        if (!validate_observation_params(item, name, params_rule, error) ||
            !require_null(item, "clip", where, error) ||
            !require_double_array(item, "scale", scale_out, where, error)) {
            return false;
        }
        if (!finite_array(scale_out)) {
            return fail(where + " scale values must be finite", error);
        }

        int history_length = 0;
        if (!require_nonnegative_int(item, "history_length", history_length,
                                     where, error)) {
            return false;
        }
        if (history_length != static_cast<int>(policy_observation::kFrameStack)) {
            return fail(where + " history_length must equal the compiled "
                        "frame stack (" +
                        std::to_string(policy_observation::kFrameStack) +
                        "), got " + std::to_string(history_length), error);
        }
        return true;
    }

    bool validate_observation_params(const YAML::Node& item,
                                     const char* name,
                                     ParamsRule params_rule,
                                     std::string& error)
    {
        const std::string where = std::string("observations.") + name +
                                  ".params";
        YAML::Node params;
        if (!require_map(item, "params",
                         params, std::string("observations.") + name, error)) {
            return false;
        }

        if (params_rule == ParamsRule::Empty) {
            return check_known_keys(params, {}, where, error);
        }

        if (!check_known_keys(params, {"command_name"}, where, error)) {
            return false;
        }
        std::string command_name;
        if (!require_value(params, "command_name", command_name,
                           where, error)) {
            return false;
        }
        if (command_name != "base_velocity") {
            return fail(where + " command_name must be base_velocity, got: " +
                        command_name, error);
        }
        return true;
    }

    std::string root_dir_;
    RobotInterfaceConfig& config_;
    std::shared_ptr<const robot_detail::JointMapping> mapping_;
    std::array<double, kDof> root_default_joint_pos_{};
};

}  // namespace

bool load_deploy_config(const std::string& yaml_path,
                        const DeployConfigLoadOptions& options,
                        RobotInterfaceConfig& config,
                        std::string& error)
{
    const std::string root_dir = options.root_dir.empty()
                                     ? std::string(ROBOT_INFERENCE_ROOT_DIR)
                                     : options.root_dir;
    DeployConfigLoader loader(root_dir, config);
    return loader.load(yaml_path, error);
}

bool load_deploy_config(const std::string& yaml_path,
                        RobotInterfaceConfig& config,
                        std::string& error)
{
    return load_deploy_config(yaml_path, DeployConfigLoadOptions{}, config,
                              error);
}

}  // namespace inference
