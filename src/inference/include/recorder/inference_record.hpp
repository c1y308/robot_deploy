#pragma once

#include "policy/policy_observation_config.hpp"
#include "recorder/inference_recorder_config.hpp"
#include "tool/tool.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace inference {

inline constexpr std::size_t kInferenceDof = policy_observation::kDof;
inline constexpr std::size_t kInferenceMotorCount = policy_observation::kDof;

inline std::int64_t steady_now_ns() noexcept
{
    return robot_base::monotonic_now_ns();
}

/* 日志数据一帧定义 */
struct InferenceRecord {
    std::uint64_t frame_index{0};

    std::int64_t motor_sample_timestamp_ns{0};

    std::int64_t inference_start_ns{0};
    std::int64_t inference_end_ns{0};
    
    std::int64_t command_timestamp_ns{0};
    std::int64_t imu_receive_timestamp_ns{0};
    std::uint64_t imu_sample_timestamp_ns{0};
    std::int64_t motor_age_us{0};

    PolicyObservation policy_observation{};                     // 真正送入 TorchScript 的 flatten policy 输入
    std::array<float,  kInferenceDof> raw_action{};             // 模型输出的原始动作向量
    std::array<double, kInferenceDof> target_q_model_rad{};     // 处理之后的目标关节角度（弧度、模型顺序）

    std::array<double, kInferenceMotorCount> target_pos_rad{};  // 电机接收的目标位置（弧度、电机顺序）
    std::array<double, kInferenceMotorCount> target_effort_permille{};  // 电机目标力矩（额定力矩千分之一）
    std::array<double, kInferenceMotorCount> rx_pos_rad{};      // 电机反馈位置（弧度、电机顺序）
    std::array<double, kInferenceMotorCount> rx_vel_rad_s{};    // 电机反馈速度（弧度/秒、电机顺序）
    std::array<double, kInferenceMotorCount> torque_percent{};  // 电机反馈扭矩百分比（电机顺序）

    std::array<std::uint8_t, kInferenceMotorCount> comm_ok{};
    std::array<std::uint8_t, kInferenceMotorCount> enabled{};

    bool command_applied{false};
};

}  // namespace inference
