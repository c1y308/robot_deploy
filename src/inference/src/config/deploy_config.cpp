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

bool finite_positive(double value)
{
    return std::isfinite(value) && value > 0.0;
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

        if (!check_known_keys(root,
                              {"hardware", "joint_ids_map",
                               "motor_to_model_direction", "ankle_parallel",
                               "motor", "policy", "action", "observation",
                               "ankle", "guard", "recorder"},
                              "root", error)) {
            return false;
        }

        return load_hardware(root, error) &&
               load_joint_topology(root, error) &&
               load_ankle(root, error) &&
               load_motor(root, error) &&
               build_motor_ordered_gains(error) &&
               load_policy(root, error) &&
               load_action(root, error) &&
               load_observation(root, error) &&
               load_guard(root, error) &&
               load_recorder(root, error);
    }

private:
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
        for (const auto& kv : node) {
            const std::string key = kv.first.as<std::string>();
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

    static bool require_nonempty_string(const YAML::Node& parent,
                                        const char* key,
                                        std::string& out,
                                        const std::string& where,
                                        std::string& error)
    {
        if (!require_value(parent, key, out, where, error)) {
            return false;
        }
        if (out.empty()) {
            return fail(where + " key " + key + " must not be empty", error);
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

    template <std::size_t N>
    static bool require_int_array(const YAML::Node& parent,
                                  const char* key,
                                  std::array<int, N>& out,
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
                out[i] = node[i].as<int>();
            } catch (const YAML::BadConversion&) {
                return fail(where + " key " + key + " element " +
                            std::to_string(i) + " is not an integer", error);
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

    /* [min, max] 两元素序列 */
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

    static bool require_positive_int(const YAML::Node& parent,
                                     const char* key,
                                     int& out,
                                     const std::string& where,
                                     std::string& error)
    {
        if (!require_value(parent, key, out, where, error)) {
            return false;
        }
        if (out <= 0) {
            return fail(where + " key " + key + " must be positive", error);
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

    bool load_hardware(const YAML::Node& root, std::string& error)
    {
        YAML::Node hardware;
        if (!require_map(root, "hardware", hardware, "root", error)) {
            return false;
        }
        if (!check_known_keys(hardware, {"ethercat_ifname", "imu"},
                              "hardware", error)) {
            return false;
        }
        if (!require_nonempty_string(hardware, "ethercat_ifname",
                                     config_.motor.ethercat_ifname,
                                     "hardware", error)) {
            return false;
        }

        YAML::Node imu;
        if (!require_map(hardware, "imu", imu, "hardware", error)) {
            return false;
        }
        if (!check_known_keys(imu, {"type", "device", "baudrate",
                                    "print_imu", "print_ahrs"},
                              "hardware.imu", error)) {
            return false;
        }
        std::string type_name;
        if (!require_value(imu, "type", type_name, "hardware.imu", error)) {
            return false;
        }
        if (type_name == "xsens_mti_can") {
            config_.imu.type = imu_base::ReaderType::XSENS_MTI_CAN;
        } else if (type_name == "a100_serial") {
            config_.imu.type = imu_base::ReaderType::A100_SERIAL;
        } else {
            return fail("hardware.imu.type must be xsens_mti_can or a100_serial, got: " +
                        type_name, error);
        }
        if (!require_nonempty_string(imu, "device", config_.imu.device,
                                     "hardware.imu", error)) {
            return false;
        }
        return require_positive_int(imu, "baudrate", config_.imu.baudrate,
                                    "hardware.imu", error) &&
               require_value(imu, "print_imu", config_.imu.print_imu,
                             "hardware.imu", error) &&
               require_value(imu, "print_ahrs", config_.imu.print_ahrs,
                             "hardware.imu", error);
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

        if (!require_int_array(root, "motor_to_model_direction",
                               config_.joint_mapping.motor_to_model_direction,
                               "root", error)) {
            return false;
        }

        YAML::Node ankle_parallel;
        if (!require_map(root, "ankle_parallel", ankle_parallel,
                         "root", error)) {
            return false;
        }
        if (!check_known_keys(ankle_parallel, {"left", "right"},
                              "ankle_parallel", error)) {
            return false;
        }
        if (!load_ankle_parallel_map(ankle_parallel, "left",
                                     config_.joint_mapping.left_ankle_parallel,
                                     error) ||
            !load_ankle_parallel_map(ankle_parallel, "right",
                                     config_.joint_mapping.right_ankle_parallel,
                                     error)) {
            return false;
        }

        // 后 4 项不参与直驱映射，只校验其与脚踝声明的 4 个电机一致
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
                        "declared ankle motors", error);
        }

        JointMappingConfig mapping_config;
        mapping_config.model_to_motor_index.assign(joint_ids_map.begin(),
                                                   joint_ids_map.begin() +
                                                       kDirectDriveDofCount);
        mapping_config.motor_to_model_direction =
            config_.joint_mapping.motor_to_model_direction;
        mapping_config.left_ankle_parallel = left;
        mapping_config.right_ankle_parallel = right;

        std::string mapping_error;
        mapping_ = robot_detail::JointMapping::create(
            static_cast<int>(kDof), mapping_config, mapping_error);
        if (!mapping_) {
            return fail("joint mapping is invalid: " + mapping_error, error);
        }
        config_.joint_mapping.model_to_motor_index =
            std::move(mapping_config.model_to_motor_index);
        return true;
    }

    bool load_ankle_parallel_map(const YAML::Node& ankle_parallel,
                                 const char* side,
                                 AnkleParallelMap& out,
                                 std::string& error)
    {
        const std::string where = std::string("ankle_parallel.") + side;
        YAML::Node node;
        if (!require_map(ankle_parallel, side, node, "ankle_parallel", error)) {
            return false;
        }
        if (!check_known_keys(node, {"pitch_dof", "roll_dof",
                                     "upper_motor", "lower_motor"},
                              where, error)) {
            return false;
        }
        return require_value(node, "pitch_dof", out.model_pitch_dof,
                             where, error) &&
               require_value(node, "roll_dof", out.model_roll_dof,
                             where, error) &&
               require_value(node, "upper_motor", out.upper_motor_index,
                             where, error) &&
               require_value(node, "lower_motor", out.lower_motor_index,
                             where, error);
    }

    bool load_ankle(const YAML::Node& root, std::string& error)
    {
        YAML::Node ankle;
        if (!require_map(root, "ankle", ankle, "root", error)) {
            return false;
        }
        if (!check_known_keys(ankle, {"motor_limits", "torque"},
                              "ankle", error)) {
            return false;
        }

        YAML::Node limits;
        if (!require_map(ankle, "motor_limits", limits, "ankle", error)) {
            return false;
        }
        if (!check_known_keys(limits, {"left_upper", "left_lower",
                                       "right_upper", "right_lower"},
                              "ankle.motor_limits", error)) {
            return false;
        }
        // 显式命名键 → 固定内部顺序 [左上, 左下, 右上, 右下]
        const char* limit_keys[4] = {"left_upper", "left_lower",
                                     "right_upper", "right_lower"};
        for (std::size_t i = 0; i < 4; ++i) {
            std::array<double, 2> range;
            if (!require_double_pair(limits, limit_keys[i], range,
                                     "ankle.motor_limits", error)) {
                return false;
            }
            if (!std::isfinite(range[0]) || !std::isfinite(range[1])) {
                return fail(std::string("ankle.motor_limits.") + limit_keys[i] +
                            " values must be finite", error);
            }
            if (range[0] > range[1]) {
                return fail(std::string("ankle.motor_limits.") + limit_keys[i] +
                            " min must be <= max", error);
            }
            config_.ankle_motor_limits.min_rad[i] = range[0];
            config_.ankle_motor_limits.max_rad[i] = range[1];
        }

        YAML::Node torque;
        if (!require_map(ankle, "torque", torque, "ankle", error)) {
            return false;
        }
        if (!check_known_keys(torque, {"virtual_kp", "virtual_kd",
                                       "filter_cutoff_rad_s", "filter_dt_s",
                                       "motor_rated_torque_nm",
                                       "target_torque_limit_permille"},
                              "ankle.torque", error)) {
            return false;
        }
        if (!require_double_array(torque, "virtual_kp",
                                  config_.ankle_torque.virtual_kp,
                                  "ankle.torque", error) ||
            !require_double_array(torque, "virtual_kd",
                                  config_.ankle_torque.virtual_kd,
                                  "ankle.torque", error)) {
            return false;
        }
        if (!finite_array(config_.ankle_torque.virtual_kp) ||
            !finite_array(config_.ankle_torque.virtual_kd)) {
            return fail("ankle.torque virtual_kp/virtual_kd must be finite",
                        error);
        }
        if (config_.ankle_torque.virtual_kp[0] < 0.0 ||
            config_.ankle_torque.virtual_kp[1] < 0.0 ||
            config_.ankle_torque.virtual_kd[0] < 0.0 ||
            config_.ankle_torque.virtual_kd[1] < 0.0) {
            return fail("ankle.torque virtual_kp/virtual_kd must be non-negative",
                        error);
        }
        if (!require_finite_positive(torque, "filter_cutoff_rad_s",
                                     config_.ankle_torque.filter_cutoff_rad_s,
                                     "ankle.torque", error) ||
            !require_finite_positive(torque, "filter_dt_s",
                                     config_.ankle_torque.filter_dt_s,
                                     "ankle.torque", error) ||
            !require_finite_positive(torque, "motor_rated_torque_nm",
                                     config_.ankle_torque.motor_rated_torque_nm,
                                     "ankle.torque", error) ||
            !require_finite_positive(torque, "target_torque_limit_permille",
                                     config_.ankle_torque.target_torque_limit_permille,
                                     "ankle.torque", error)) {
            return false;
        }
        if (config_.ankle_torque.target_torque_limit_permille > 32767.0) {
            return fail("ankle.torque target_torque_limit_permille must be "
                        "explicitly configured in (0, 32767]", error);
        }
        return true;
    }

    bool load_motor(const YAML::Node& root, std::string& error)
    {
        YAML::Node motor;
        if (!require_map(root, "motor", motor, "root", error)) {
            return false;
        }
        if (!check_known_keys(motor, {"control_mode", "kp", "kd",
                                      "wait_all_motors_timeout_ms",
                                      "wait_all_motors_poll_ms",
                                      "control_ready_timeout_ms",
                                      "print_info", "print_ids"},
                              "motor", error)) {
            return false;
        }

        std::string control_mode_name;
        if (!require_value(motor, "control_mode", control_mode_name,
                           "motor", error)) {
            return false;
        }
        // 脚踝扭矩控制要求阻抗模式（原 validate_policy_config 规则）
        if (control_mode_name != "impedance") {
            return fail("motor.control_mode must be impedance, got: " +
                        control_mode_name, error);
        }
        config_.motor.control_mode = motor_base::MotorControlMode::IMPEDANCE;
        config_.motor.num_motors = static_cast<int>(kMotorCount);

        // kp/kd 按模型 DOF 顺序，仅直驱 8 项；脚踝槽位由
        // ankle.torque.virtual_kp/kd 在 build_motor_ordered_gains() 填充
        if (!require_double_array(motor, "kp", direct_drive_kp_,
                                  "motor", error) ||
            !require_double_array(motor, "kd", direct_drive_kd_,
                                  "motor", error)) {
            return false;
        }
        if (!finite_array(direct_drive_kp_) || !finite_array(direct_drive_kd_)) {
            return fail("motor kp/kd values must be finite", error);
        }

        if (!require_positive_int(motor, "wait_all_motors_timeout_ms",
                                  config_.motor.wait_all_motors_timeout_ms,
                                  "motor", error) ||
            !require_positive_int(motor, "wait_all_motors_poll_ms",
                                  config_.motor.wait_all_motors_poll_ms,
                                  "motor", error) ||
            !require_positive_int(motor, "control_ready_timeout_ms",
                                  config_.motor.control_ready_timeout_ms,
                                  "motor", error) ||
            !require_value(motor, "print_info",
                           config_.motor.print_motors_info,
                           "motor", error)) {
            return false;
        }
        if (!require_int_vector(motor, "print_ids",
                                config_.motor.print_motor_ids,
                                "motor", error)) {
            return false;
        }
        for (int motor_id : config_.motor.print_motor_ids) {
            if (!index_in_range(motor_id, static_cast<int>(kMotorCount))) {
                return fail("motor.print_ids value out of range [0, " +
                            std::to_string(kMotorCount) + "): " +
                            std::to_string(motor_id), error);
            }
        }
        return true;
    }

    /* 模型序直驱增益 + 脚踝虚拟增益 → 物理电机顺序的 12 槽位 */
    bool build_motor_ordered_gains(std::string& error)
    {
        std::vector<unsigned char> filled(kMotorCount, 0U);

        std::size_t direct_slot = 0;
        for (int model_index = 0;
             model_index < static_cast<int>(kDof); ++model_index) {
            if (mapping_->is_parallel_model_dof(model_index)) {
                continue;
            }
            const int motor_index =
                mapping_->direct_motor_for_model_dof(model_index);
            if (!index_in_range(motor_index, static_cast<int>(kMotorCount))) {
                return fail("joint mapping returned an out-of-range motor index",
                            error);
            }
            if (filled[static_cast<std::size_t>(motor_index)] != 0U) {
                return fail("motor gain slot written twice: motor " +
                            std::to_string(motor_index), error);
            }
            filled[static_cast<std::size_t>(motor_index)] = 1U;
            config_.motor.mit_kp[static_cast<std::size_t>(motor_index)] =
                direct_drive_kp_[direct_slot];
            config_.motor.mit_kd[static_cast<std::size_t>(motor_index)] =
                direct_drive_kd_[direct_slot];
            ++direct_slot;
        }
        if (direct_slot != kDirectDriveDofCount) {
            return fail("direct drive DOF count mismatch while filling motor gains",
                        error);
        }

        // 4 个脚踝物理电机的增益单一来源：取虚拟轴 [0]（pitch）。
        // 当前 [pitch, roll] 两轴等值（187/9.07）；若将来两轴不等值，
        // 需重新评估脚踝电机复位增益应取哪一轴。
        const int ankle_motor_indices[4] = {
            mapping_->left_ankle().upper_motor_index,
            mapping_->left_ankle().lower_motor_index,
            mapping_->right_ankle().upper_motor_index,
            mapping_->right_ankle().lower_motor_index
        };
        for (int motor_index : ankle_motor_indices) {
            if (!index_in_range(motor_index, static_cast<int>(kMotorCount))) {
                return fail("ankle motor index out of range: " +
                            std::to_string(motor_index), error);
            }
            if (filled[static_cast<std::size_t>(motor_index)] != 0U) {
                return fail("motor gain slot written twice: motor " +
                            std::to_string(motor_index), error);
            }
            filled[static_cast<std::size_t>(motor_index)] = 1U;
            config_.motor.mit_kp[static_cast<std::size_t>(motor_index)] =
                config_.ankle_torque.virtual_kp[0];
            config_.motor.mit_kd[static_cast<std::size_t>(motor_index)] =
                config_.ankle_torque.virtual_kd[0];
        }

        if (std::any_of(filled.begin(), filled.end(),
                        [](unsigned char value) { return value == 0U; })) {
            return fail("motor gain slots are not fully filled after conversion",
                        error);
        }
        return true;
    }

    bool load_policy(const YAML::Node& root, std::string& error)
    {
        YAML::Node policy;
        if (!require_map(root, "policy", policy, "root", error)) {
            return false;
        }
        std::set<std::string> allowed = {"model_path", "step_dt",
                                         "target_interpolation_duration_s",
                                         "raw_action_clip"};
        if (policy_observation::kEnableGaitPhase) {
            allowed.insert("gait_phase");
        }
        if (!check_known_keys(policy, allowed, "policy", error)) {
            return false;
        }

        std::string model_path;
        if (!require_nonempty_string(policy, "model_path", model_path,
                                     "policy", error)) {
            return false;
        }
        config_.policy.model_path = resolve_path(model_path).string();

        if (!require_finite_positive(policy, "step_dt",
                                     config_.policy.step_dt,
                                     "policy", error)) {
            return false;
        }
        if (!require_value(policy, "target_interpolation_duration_s",
                           config_.policy.target_interpolation_duration_s,
                           "policy", error)) {
            return false;
        }
        if (!std::isfinite(config_.policy.target_interpolation_duration_s) ||
            config_.policy.target_interpolation_duration_s < 0.0) {
            return fail("policy.target_interpolation_duration_s must be finite "
                        "and non-negative", error);
        }
        if (!require_finite_positive(policy, "raw_action_clip",
                                     raw_action_clip_, "policy", error)) {
            return false;
        }

        const YAML::Node gait = policy["gait_phase"];
        if constexpr (policy_observation::kEnableGaitPhase) {
            if (!gait.IsDefined()) {
                return fail("policy.gait_phase is required when built with "
                            "ROBOT_POLICY_ENABLE_GAIT_PHASE_OBS=ON "
                            "(705-dim observation)", error);
            }
            if (!gait.IsMap()) {
                return fail("policy.gait_phase must be a map", error);
            }
            if (!check_known_keys(gait, {"period", "stand_threshold",
                                         "move_threshold"},
                                  "policy.gait_phase", error)) {
                return false;
            }
            auto& gait_config = config_.policy.gait;
            if (!require_finite_positive(gait, "period", gait_config.period,
                                         "policy.gait_phase", error) ||
                !require_value(gait, "stand_threshold",
                               gait_config.stand_threshold,
                               "policy.gait_phase", error) ||
                !require_value(gait, "move_threshold",
                               gait_config.move_threshold,
                               "policy.gait_phase", error)) {
                return false;
            }
            if (!std::isfinite(gait_config.stand_threshold) ||
                !std::isfinite(gait_config.move_threshold)) {
                return fail("policy.gait_phase thresholds must be finite",
                            error);
            }
            if (gait_config.move_threshold <= gait_config.stand_threshold) {
                return fail("policy.gait_phase move_threshold must be greater "
                            "than stand_threshold", error);
            }
        } else {
            if (gait.IsDefined()) {
                return fail("policy.gait_phase must not be present when built "
                            "with ROBOT_POLICY_ENABLE_GAIT_PHASE_OBS=OFF "
                            "(675-dim observation)", error);
            }
        }
        return true;
    }

    bool load_action(const YAML::Node& root, std::string& error)
    {
        YAML::Node action;
        if (!require_map(root, "action", action, "root", error)) {
            return false;
        }
        if (!check_known_keys(action, {"default_joint_pos", "scale", "clip"},
                              "action", error)) {
            return false;
        }
        if (!require_double_array(action, "default_joint_pos",
                                  config_.action.default_joint_pos_rad,
                                  "action", error) ||
            !require_double_array(action, "scale",
                                  config_.action.action_scale,
                                  "action", error)) {
            return false;
        }
        if (!finite_array(config_.action.default_joint_pos_rad) ||
            !finite_array(config_.action.action_scale)) {
            return fail("action default_joint_pos/scale values must be finite",
                        error);
        }

        const YAML::Node clip = action["clip"];
        if (!clip.IsDefined() || !clip.IsSequence()) {
            return fail("action missing required sequence key: clip", error);
        }
        if (clip.size() != kDof) {
            return fail("action.clip must have " + std::to_string(kDof) +
                        " ranges, got " + std::to_string(clip.size()), error);
        }
        for (std::size_t i = 0; i < kDof; ++i) {
            const YAML::Node range = clip[i];
            if (!range.IsSequence() || range.size() != 2) {
                return fail("action.clip[" + std::to_string(i) +
                            "] must be [lower, upper]", error);
            }
            try {
                config_.action.action_clip[i][0] = range[0].as<double>();
                config_.action.action_clip[i][1] = range[1].as<double>();
            } catch (const YAML::BadConversion&) {
                return fail("action.clip[" + std::to_string(i) +
                            "] must be two numbers", error);
            }
            if (!std::isfinite(config_.action.action_clip[i][0]) ||
                !std::isfinite(config_.action.action_clip[i][1])) {
                return fail("action.clip values must be finite", error);
            }
            if (config_.action.action_clip[i][0] >
                config_.action.action_clip[i][1]) {
                return fail("action.clip lower bound must be <= upper bound "
                            "for every model DOF", error);
            }
            if (config_.action.action_scale[i] <= 0.0) {
                return fail("action.scale must be > 0 for every model DOF",
                            error);
            }
        }
        config_.action.raw_action_clip = raw_action_clip_;
        return true;
    }

    template <std::size_t N>
    bool load_observation_item_impl(const YAML::Node& observation,
                                    const char* name,
                                    std::array<double, N>& scale_out,
                                    std::string& error)
    {
        const std::string where = std::string("observation.") + name;
        YAML::Node item;
        if (!require_map(observation, name, item, "observation", error)) {
            return false;
        }
        if (!check_known_keys(item, {"scale", "history_length"},
                              where, error)) {
            return false;
        }
        if (!require_double_array(item, "scale", scale_out, where, error)) {
            return false;
        }
        if (!finite_array(scale_out)) {
            return fail(where + " scale values must be finite", error);
        }
        int history_length = 0;
        if (!require_positive_int(item, "history_length", history_length,
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

    bool load_observation(const YAML::Node& root, std::string& error)
    {
        YAML::Node observation;
        if (!require_map(root, "observation", observation, "root", error)) {
            return false;
        }
        if (!check_known_keys(observation, {"base_ang_vel",
                                            "velocity_commands",
                                            "joint_pos_rel",
                                            "joint_vel_rel"},
                              "observation", error)) {
            return false;
        }
        return load_observation_item_impl(
                   observation, "base_ang_vel",
                   config_.observation_scales.body_ang_vel_scale, error) &&
               load_observation_item_impl(
                   observation, "velocity_commands",
                   config_.observation_scales.command_scale, error) &&
               load_observation_item_impl(
                   observation, "joint_pos_rel",
                   config_.observation_scales.dof_pos_scale, error) &&
               load_observation_item_impl(
                   observation, "joint_vel_rel",
                   config_.observation_scales.dof_vel_scale, error);
    }

    bool load_guard(const YAML::Node& root, std::string& error)
    {
        YAML::Node guard;
        if (!require_map(root, "guard", guard, "root", error)) {
            return false;
        }
        if (!check_known_keys(guard, {"max_imu_sample_age_s",
                                      "max_motor_sample_age_s",
                                      "max_sensor_state_skew_s"},
                              "guard", error)) {
            return false;
        }
        return require_finite_positive(guard, "max_imu_sample_age_s",
                                       config_.sensor_guard.max_imu_sample_age_s,
                                       "guard", error) &&
               require_finite_positive(guard, "max_motor_sample_age_s",
                                       config_.sensor_guard.max_motor_sample_age_s,
                                       "guard", error) &&
               require_finite_positive(guard, "max_sensor_state_skew_s",
                                       config_.sensor_guard.max_sensor_state_skew_s,
                                       "guard", error);
    }

    bool load_recorder(const YAML::Node& root, std::string& error)
    {
        YAML::Node recorder;
        if (!require_map(root, "recorder", recorder, "root", error)) {
            return false;
        }
        if (!check_known_keys(recorder, {"enabled", "directory",
                                         "file_prefix", "flush_interval_ms",
                                         "max_queue_depth"},
                              "recorder", error)) {
            return false;
        }
        if (!require_value(recorder, "enabled", config_.recorder.enabled,
                           "recorder", error)) {
            return false;
        }
        std::string directory;
        if (!require_nonempty_string(recorder, "directory", directory,
                                     "recorder", error)) {
            return false;
        }
        config_.recorder.directory = resolve_path(directory);
        if (!require_value(recorder, "file_prefix",
                           config_.recorder.file_prefix,
                           "recorder", error)) {
            return false;
        }
        int flush_interval_ms = 0;
        if (!require_positive_int(recorder, "flush_interval_ms",
                                  flush_interval_ms, "recorder", error)) {
            return false;
        }
        config_.recorder.flush_interval =
            std::chrono::milliseconds(flush_interval_ms);

        long long max_queue_depth = 0;
        if (!require_value(recorder, "max_queue_depth", max_queue_depth,
                           "recorder", error)) {
            return false;
        }
        if (max_queue_depth <= 0) {
            return fail("recorder.max_queue_depth must be positive", error);
        }
        config_.recorder.max_queue_depth =
            static_cast<std::size_t>(max_queue_depth);
        return true;
    }

    std::string root_dir_;
    RobotInterfaceConfig& config_;
    std::shared_ptr<const robot_detail::JointMapping> mapping_;
    std::array<double, kDirectDriveDofCount> direct_drive_kp_;
    std::array<double, kDirectDriveDofCount> direct_drive_kd_;
    double raw_action_clip_ = 0.0;
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
