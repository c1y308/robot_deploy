// P1 Chirp 系统辨识实验：单文件实现，不修改部署路径，不链接 LibTorch。
// 经 src/motors 的 CMake 目标 chirp_test 构建（参考 id_test），同时会进入
// src/inference/build 的 compile_commands.json 供 clangd 索引。
//
// 构建命令（仓库根目录）：
//   cmake -S src/motors -B src/motors/build
//   cmake --build src/motors/build --target chirp_test
// 产物为 src/motors/build/chirp_test。也可手工编译（下列各行合为一条命令，
// 或用 shell 续行符自行折行）：
//   g++ -std=c++17 -O2 src/motors/examples/chirp_test.cpp
//       src/inference/src/robot/action_processor.cpp src/inference/src/robot/joint_mapping.cpp
//       -Isrc/motors/include -Isrc/inference/include -Isrc/base/include
//       -I/home/cat/ethercat/include src/motors/build/libmyactua_ethercat.a
//       -L/home/cat/ethercat/lib -lethercat -lpthread -lrt -o /tmp/chirp_test
//
// 运行前必须填写 kChirpAmplitudeRad / kModelSafeMin/MaxRad / kMotorSafeMin/MaxRad，
// 否则 validate_config 在使能前拒绝启动。--self-test 不连接硬件。

#include "driver/myact/myact_motor_controller.hpp"
#include "driver/myact/motor_units.hpp"
#include "kinematics/ankle_motor_fk.hpp"
#include "kinematics/ankle_motor_jacobian.hpp"
#include "protocol/ethercat/ethercat_adapter_igh.hpp"
#include "robot/action_processor.hpp"
#include "robot/joint_mapping.hpp"
#include "tool/tool.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <malloc.h>
#include <memory>
#include <sstream>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr std::size_t kDof = motor_base::kMaxMotors;
constexpr std::size_t kSampleCapacity = 28'000;
constexpr std::int64_t kCommandValidityNs = 10'000'000;
constexpr double kFrequencyStartHz = 0.1;
constexpr double kFrequencyEndHz = 10.0;
constexpr double kChirpDurationS = 20.0;
constexpr double kRampToNominalS = 3.0;
constexpr double kNominalHoldS = 1.0;
constexpr double kReturnS = 1.0;
constexpr double kFinalHoldS = 1.0;
constexpr double kNominalStartS = kRampToNominalS + kNominalHoldS;
constexpr double kChirpEndS = kNominalStartS + kChirpDurationS;
constexpr double kReturnEndS = kChirpEndS + kReturnS;
constexpr double kExperimentEndS = kReturnEndS + kFinalHoldS;
constexpr double kTwoPi = 6.283185307179586476925286766559;

constexpr std::array<const char*, kDof> kJointNames = {
    "left_hip_roll", "right_hip_roll", "left_hip_pitch", "right_hip_pitch",
    "left_hip_yaw", "right_hip_yaw", "left_knee_pitch", "right_knee_pitch",
    "left_ankle_pitch", "right_ankle_pitch", "left_ankle_roll", "right_ankle_roll",
};
constexpr std::array<int, kDof> kJointIdsMap = {0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11};
constexpr std::array<int, kDof> kMotorDirection = {
    -1, -1, 1, 1, -1, -1, -1, 1, 1, -1, -1, -1,
};
constexpr std::array<double, kDof> kNominalModelRad{};
constexpr std::array<double, kDof> kChirpSign = {
    1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0,
};

// Fill all three arrays before running on hardware.  NaN is intentionally an
// invalid default: it prevents a partially configured test from enabling motors.
constexpr double kUnset = std::numeric_limits<double>::quiet_NaN();
const std::array<double, kDof> kChirpAmplitudeRad = {
    0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
};
const std::array<double, kDof> kModelSafeMinRad = {
    -3.3, -3.3, -3.3, -3.3, -3.3, -3.3,
    -3.3, -3.3, -3.3, -3.3, -3.3, -3.3,
};
const std::array<double, kDof> kModelSafeMaxRad = {
    3.3, 3.3, 3.3, 3.3, 3.3, 3.3,
    3.3, 3.3, 3.3, 3.3, 3.3, 3.3,
};
const std::array<double, kDof> kMotorSafeMinRad = {
    -3.3, -3.3, -3.3, -3.3, -3.3, -3.3,
    -3.3, -3.3, -3.3, -3.3, -3.3, -3.3,
};
const std::array<double, kDof> kMotorSafeMaxRad = {
    3.3, 3.3, 3.3, 3.3, 3.3, 3.3,
    3.3, 3.3, 3.3, 3.3, 3.3, 3.3,
};
constexpr std::array<double, kDof> kMotorKp = {
    180.0, 180.0, 180.0, 180.0, 180.0, 180.0,
    180.0, 180.0, 180.0, 180.0, 180.0, 180.0,
};
constexpr std::array<double, kDof> kMotorKd = {
    10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
    10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
};

enum class Phase : std::uint8_t {
    RampToNominal = 0,
    NominalHold = 1,
    Chirp = 2,
    ReturnToNominal = 3,
    FinalHold = 4,
};

struct Sample {
    std::int64_t timestamp_ns{};
    std::int64_t generation_ns{};
    std::int64_t feedback_timestamp_ns{};
    std::uint64_t cycle_index{};
    double elapsed_s{};
    double chirp_elapsed_s{};
    double cycle_dt_s{};
    std::uint8_t phase{};
    std::uint8_t command_applied{};
    unsigned int working_counter{};
    std::array<double, kDof> q_target{};
    std::array<double, kDof> q_actual{};
    std::array<double, kDof> dq_actual{};
    std::array<double, kDof> motor_position{};
    std::array<double, kDof> motor_velocity{};
    std::array<double, kDof> motor_torque_percent{};
    std::array<myactua::TxPDO, kDof> tx{};
};

struct PendingCommand {
    bool valid{false};
    std::int64_t generation_ns{0};
    double elapsed_s{0.0};
    double chirp_elapsed_s{0.0};
    Phase phase{Phase::RampToNominal};
    std::array<double, kDof> q_target{};
};

struct TargetEval {
    Phase phase{Phase::RampToNominal};
    double chirp_elapsed_s{-1.0};
    bool finished{false};
};

struct ChirpContext;

// This exposes only the existing terminal-fault operation.  It does not alter
// the MYACT controller's state machine or PDO implementation.
class ChirpMotorController final : public myactua::MyActMotorController {
public:
    using myactua::MyActMotorController::MyActMotorController;
    using motor_base::MotorControllerBase::latch_terminal_fault;
};

class ChirpAdapter final : public myactua::EthercatAdapterIGH {
public:
    explicit ChirpAdapter(ChirpContext& context) : context_(context) {}

    void receive_physical() override;
    void send(int index, const myactua::TxPDO& pdo) override;
    void send_physical() override;

private:
    ChirpContext& context_;
};

struct ChirpContext {
    std::array<Sample, kSampleCapacity> samples{};
    std::atomic<std::size_t> sample_count{0};
    std::atomic<bool> armed{false};
    std::atomic<bool> finished{false};
    std::atomic<bool> aborted{false};
    std::atomic<int> abort_reason{0};

    ChirpMotorController* controller{nullptr};
    inference::robot_detail::ActionProcessor* action_processor{nullptr};
    std::shared_ptr<const inference::robot_detail::JointMapping> mapping;

    std::array<myactua::RxPDO, kDof> latest_rx{};
    std::array<myactua::TxPDO, kDof> current_tx{};
    std::size_t tx_count{0};
    PendingCommand queued{};
    PendingCommand applying{};

    ankle_motor_fk::Solver left_fk;
    ankle_motor_fk::Solver right_fk;
    std::string jacobian_error;
    std::string action_error;
    std::array<double, kDof> start_model{};
    std::array<double, kDof> last_q_actual{};
    std::array<double, kDof> last_dq_actual{};
    bool start_captured{false};
    std::int64_t start_ns{0};
    std::int64_t last_sample_ns{0};
    std::uint64_t cycle_index{0};

    void abort_rt(int reason) noexcept
    {
        if (!aborted.exchange(true, std::memory_order_acq_rel)) {
            abort_reason.store(reason, std::memory_order_release);
            armed.store(false, std::memory_order_release);
            if (controller) {
                controller->latch_terminal_fault();
            }
        }
    }

    static double smoothstep(double alpha) noexcept
    {
        alpha = std::max(0.0, std::min(1.0, alpha));
        return alpha * alpha * (3.0 - 2.0 * alpha);
    }

    static double linear_chirp(double t) noexcept
    {
        if (!std::isfinite(t) || t < 0.0 || t >= kChirpDurationS) return 0.0;
        const double k = (kFrequencyEndHz - kFrequencyStartHz) / kChirpDurationS;
        const double phase = kTwoPi * (kFrequencyStartHz * t + 0.5 * k * t * t);
        const double value = std::sin(phase);
        return std::isfinite(value) ? value : 0.0;
    }

    // 目标是 CLOCK_MONOTONIC elapsed time 的纯函数：周期抖动或丢帧只会
    // 重新采样同一条连续时间轨迹，不会扭曲激励本身。各阶段以 nominal 为基准，
    // nominal 全零时与“零位”一致。
    static TargetEval compute_target(
        double elapsed,
        const std::array<double, kDof>& start_model,
        const std::array<double, kDof>& amplitude,
        std::array<double, kDof>& q_target) noexcept
    {
        q_target = kNominalModelRad;
        TargetEval eval;
        if (elapsed < kRampToNominalS) {
            const double blend = smoothstep(elapsed / kRampToNominalS);
            for (std::size_t i = 0; i < kDof; ++i) {
                q_target[i] = kNominalModelRad[i] * blend + start_model[i] * (1.0 - blend);
            }
        } else if (elapsed < kNominalStartS) {
            eval.phase = Phase::NominalHold;
        } else if (elapsed < kChirpEndS) {
            eval.phase = Phase::Chirp;
            eval.chirp_elapsed_s = elapsed - kNominalStartS;
            const double chirp = linear_chirp(eval.chirp_elapsed_s);
            for (std::size_t i = 0; i < kDof; ++i) {
                q_target[i] = kNominalModelRad[i] + kChirpSign[i] * amplitude[i] * chirp;
            }
        } else if (elapsed < kReturnEndS) {
            eval.phase = Phase::ReturnToNominal;
            const double blend = smoothstep((elapsed - kChirpEndS) / kReturnS);
            const double last_chirp = linear_chirp(std::nextafter(kChirpDurationS, 0.0));
            for (std::size_t i = 0; i < kDof; ++i) {
                q_target[i] = kNominalModelRad[i] +
                    kChirpSign[i] * amplitude[i] * last_chirp * (1.0 - blend);
            }
        } else if (elapsed < kExperimentEndS) {
            eval.phase = Phase::FinalHold;
        } else {
            eval.phase = Phase::FinalHold;
            eval.finished = true;
        }
        return eval;
    }

    bool decode_model_state(std::array<motor_base::MotorStatusSnapshot, kDof>& feedback,
                            std::array<double, kDof>& q_model,
                            std::array<double, kDof>& dq_model) noexcept
    {
        q_model.fill(0.0);
        dq_model.fill(0.0);
        for (std::size_t i = 0; i < kDof; ++i) {
            const bool ankle_motor = i == 4 || i == 5 || i == 10 || i == 11;
            const double pos_scale = ankle_motor ? myactua::kAnkleRawPosToRad : myactua::kRawPosToRad;
            const double vel_scale = ankle_motor ? myactua::kAnkleRawVelToRadPerSec : myactua::kRawVelToRadPerSec;
            const auto& rx = latest_rx[i];
            const bool ready = myactua::is_operation_enabled(rx.status_word) &&
                               !myactua::is_fault(rx.status_word) && rx.error == 0 &&
                               rx.op_mode == static_cast<std::int8_t>(myactua::MyactControlMode::PVT);
            const double q = static_cast<double>(rx.pos) * pos_scale;
            const double dq = static_cast<double>(rx.vel) * vel_scale;
            if (!ready || !std::isfinite(q) || !std::isfinite(dq) ||
                q < kMotorSafeMinRad[i] || q > kMotorSafeMaxRad[i]) {
                return false;
            }
            feedback[i].motor_index = static_cast<int>(i);
            feedback[i].position_rad = q;
            feedback[i].velocity_rad_s = dq;
            feedback[i].torque_percent = static_cast<double>(rx.torque) * myactua::kRawTorqueToPercent;
            feedback[i].comm_ok = true;
            feedback[i].enabled = true;
            feedback[i].faulted = false;
            feedback[i].control_ready = true;
            feedback[i].mode = motor_base::MotorControlMode::IMPEDANCE;
            feedback[i].target_mode = motor_base::MotorControlMode::IMPEDANCE;
            feedback[i].host_timestamp_ns = robot_base::monotonic_now_ns();
        }

        for (int model_index = 0; model_index < static_cast<int>(kDof); ++model_index) {
            if (mapping->is_parallel_model_dof(model_index)) continue;
            const int motor = mapping->direct_motor_for_model_dof(model_index);
            q_model[static_cast<std::size_t>(model_index)] =
                static_cast<double>(mapping->direction_for_motor(motor)) * feedback[static_cast<std::size_t>(motor)].position_rad;
            dq_model[static_cast<std::size_t>(model_index)] =
                static_cast<double>(mapping->direction_for_motor(motor)) * feedback[static_cast<std::size_t>(motor)].velocity_rad_s;
        }

        const auto solve_ankle = [&](const inference::AnkleParallelMap& ankle,
                                     ankle_motor_fk::Solver& solver) {
            const int upper = ankle.upper_motor_index;
            const int lower = ankle.lower_motor_index;
            const double upper_q = mapping->direction_for_motor(upper) * feedback[static_cast<std::size_t>(upper)].position_rad;
            const double lower_q = mapping->direction_for_motor(lower) * feedback[static_cast<std::size_t>(lower)].position_rad;
            const double upper_dq = mapping->direction_for_motor(upper) * feedback[static_cast<std::size_t>(upper)].velocity_rad_s;
            const double lower_dq = mapping->direction_for_motor(lower) * feedback[static_cast<std::size_t>(lower)].velocity_rad_s;
            const ankle_motor_fk::FootAngles foot = solver.solve(upper_q, lower_q);
            if (!foot.reachable || !std::isfinite(foot.pitch) || !std::isfinite(foot.roll)) return false;
            ankle_motor_jacobian::Result jacobian;
            jacobian_error.clear();
            if (!ankle_motor_jacobian::solve(foot.pitch, foot.roll, upper_q, lower_q,
                                             jacobian, jacobian_error)) return false;
            q_model[static_cast<std::size_t>(ankle.model_pitch_dof)] = foot.pitch;
            q_model[static_cast<std::size_t>(ankle.model_roll_dof)] = foot.roll;
            dq_model[static_cast<std::size_t>(ankle.model_pitch_dof)] =
                jacobian.virtual_from_motor[0][0] * upper_dq + jacobian.virtual_from_motor[0][1] * lower_dq;
            dq_model[static_cast<std::size_t>(ankle.model_roll_dof)] =
                jacobian.virtual_from_motor[1][0] * upper_dq + jacobian.virtual_from_motor[1][1] * lower_dq;
            return std::isfinite(dq_model[static_cast<std::size_t>(ankle.model_pitch_dof)]) &&
                   std::isfinite(dq_model[static_cast<std::size_t>(ankle.model_roll_dof)]);
        };
        return solve_ankle(mapping->left_ankle(), left_fk) &&
               solve_ankle(mapping->right_ankle(), right_fk);
    }

    void receive_rt(ChirpAdapter& adapter) noexcept
    {
        if (!armed.load(std::memory_order_acquire) ||
            aborted.load(std::memory_order_acquire)) return;

        const auto health = adapter.get_bus_health();
        if (!health.master_link_up || health.wc_state != EC_WC_COMPLETE) {
            abort_rt(1);
            return;
        }
        for (std::size_t i = 0; i < kDof; ++i) latest_rx[i] = adapter.myactua::EthercatAdapterIGH::receive(static_cast<int>(i));

        // Keep these guard solvers in exactly the same initial state and call
        // order as ActionProcessor's ankle torque solvers.  Their successful
        // result proves the latter will not enter its allocating error path.
        if (!start_captured) {
            left_fk.reset();
            right_fk.reset();
        }
        std::array<motor_base::MotorStatusSnapshot, kDof> feedback{};
        std::array<double, kDof> q_actual{};
        std::array<double, kDof> dq_actual{};
        if (!decode_model_state(feedback, q_actual, dq_actual)) {
            abort_rt(2);
            return;
        }
        last_q_actual = q_actual;
        last_dq_actual = dq_actual;
        if (queued.valid) applying = queued;
        for (std::size_t i = 0; i < kDof; ++i) {
            if (!std::isfinite(q_actual[i]) || !std::isfinite(dq_actual[i]) ||
                q_actual[i] < kModelSafeMinRad[i] || q_actual[i] > kModelSafeMaxRad[i]) {
                abort_rt(3);
                return;
            }
        }

        const std::int64_t now_ns = robot_base::monotonic_now_ns();
        if (!start_captured) {
            start_model = q_actual;
            start_captured = true;
            start_ns = now_ns;
        }
        const double elapsed = static_cast<double>(now_ns - start_ns) / 1'000'000'000.0;
        std::array<double, kDof> q_target{};
        const TargetEval eval = compute_target(elapsed, start_model, kChirpAmplitudeRad, q_target);
        if (eval.finished) finished.store(true, std::memory_order_release);
        for (std::size_t i = 0; i < kDof; ++i) {
            if (!std::isfinite(q_target[i]) || q_target[i] < kModelSafeMinRad[i] || q_target[i] > kModelSafeMaxRad[i]) {
                abort_rt(4);
                return;
            }
        }

        inference::robot_detail::ActionProcessor::FixedPolicyMotorCommand motor_command;
        action_error.clear();
        if (!action_processor->build_policy_impedance_command(q_target, feedback, motor_command, action_error) ||
            motor_command.setpoint_count != kDof) {
            abort_rt(5);
            return;
        }
        for (const auto& setpoint : motor_command.setpoints) {
            if (!std::isfinite(setpoint.position_rad) || !std::isfinite(setpoint.velocity_rad_s) ||
                !std::isfinite(setpoint.effort_ff) || !std::isfinite(setpoint.kp) || !std::isfinite(setpoint.kd)) {
                abort_rt(6);
                return;
            }
        }

        motor_base::ControlCommand command = motor_base::ControlCommand::set_impedance_targets_fixed(
            motor_command.setpoints.data(), motor_command.setpoint_count);
        command.timing.produced_at_ns = now_ns;
        command.timing.valid_until_ns = now_ns + kCommandValidityNs;
        const auto result = controller->send_policy_setpoint(command);
        if (result.status != motor_base::CommandSubmitStatus::ACCEPTED) {
            abort_rt(7);
            return;
        }
        queued.valid = true;
        queued.generation_ns = now_ns;
        queued.elapsed_s = elapsed;
        queued.chirp_elapsed_s = eval.chirp_elapsed_s;
        queued.phase = eval.phase;
        queued.q_target = q_target;
    }

    void tx_rt(int index, const myactua::TxPDO& pdo) noexcept
    {
        if (index >= 0 && index < static_cast<int>(kDof)) {
            current_tx[static_cast<std::size_t>(index)] = pdo;
            ++tx_count;
        }
    }

    void send_rt(const ChirpAdapter& adapter) noexcept
    {
        if (!applying.valid || aborted.load(std::memory_order_acquire)) {
            tx_count = 0;
            return;
        }
        if (tx_count != kDof) {
            abort_rt(8);
            tx_count = 0;
            return;
        }
        const std::size_t index = sample_count.fetch_add(1, std::memory_order_relaxed);
        if (index >= samples.size()) {
            abort_rt(9);
            tx_count = 0;
            return;
        }
        const std::int64_t now_ns = robot_base::monotonic_now_ns();
        Sample& sample = samples[index];
        sample.timestamp_ns = now_ns;
        sample.generation_ns = applying.generation_ns;
        sample.feedback_timestamp_ns = now_ns;
        sample.cycle_index = cycle_index++;
        sample.elapsed_s = applying.elapsed_s;
        sample.chirp_elapsed_s = applying.chirp_elapsed_s;
        sample.cycle_dt_s = last_sample_ns == 0 ? 0.0 :
            static_cast<double>(now_ns - last_sample_ns) / 1'000'000'000.0;
        sample.phase = static_cast<std::uint8_t>(applying.phase);
        sample.command_applied = 1;
        sample.working_counter = adapter.get_bus_health().working_counter;
        sample.q_target = applying.q_target;
        sample.tx = current_tx;
        for (std::size_t i = 0; i < kDof; ++i) {
            const bool ankle_motor = i == 4 || i == 5 || i == 10 || i == 11;
            const double pos_scale = ankle_motor ? myactua::kAnkleRawPosToRad : myactua::kRawPosToRad;
            const double vel_scale = ankle_motor ? myactua::kAnkleRawVelToRadPerSec : myactua::kRawVelToRadPerSec;
            sample.motor_position[i] = static_cast<double>(latest_rx[i].pos) * pos_scale;
            sample.motor_velocity[i] = static_cast<double>(latest_rx[i].vel) * vel_scale;
            sample.motor_torque_percent[i] = static_cast<double>(latest_rx[i].torque) * myactua::kRawTorqueToPercent;
        }
        sample.q_actual = last_q_actual;
        sample.dq_actual = last_dq_actual;
        last_sample_ns = now_ns;
        tx_count = 0;
    }
};

void ChirpAdapter::receive_physical()
{
    myactua::EthercatAdapterIGH::receive_physical();
    context_.receive_rt(*this);
}

void ChirpAdapter::send(int index, const myactua::TxPDO& pdo)
{
    context_.tx_rt(index, pdo);
    myactua::EthercatAdapterIGH::send(index, pdo);
}

void ChirpAdapter::send_physical()
{
    context_.send_rt(*this);
    myactua::EthercatAdapterIGH::send_physical();
}

std::atomic<bool> g_interrupt{false};
void signal_handler(int) { g_interrupt.store(true, std::memory_order_release); }

enum class ConfigRejection {
    None,
    Amplitude,
    Range,
    Excursion,
    NoExcitation,
};

// 纯函数：供 validate_config 与 --self-test 共用。返回首个非法关节下标。
ConfigRejection validate_config_values(
    const std::array<double, kDof>& amplitude,
    const std::array<double, kDof>& nominal,
    const std::array<double, kDof>& model_min,
    const std::array<double, kDof>& model_max,
    const std::array<double, kDof>& motor_min,
    const std::array<double, kDof>& motor_max,
    std::size_t& joint) noexcept
{
    bool any_excitation = false;
    for (std::size_t i = 0; i < kDof; ++i) {
        if (!std::isfinite(amplitude[i]) || amplitude[i] < 0.0 ||
            !std::isfinite(nominal[i])) {
            joint = i;
            return ConfigRejection::Amplitude;
        }
        any_excitation = any_excitation || amplitude[i] > 0.0;
        if (!std::isfinite(model_min[i]) || !std::isfinite(model_max[i]) ||
            model_min[i] >= model_max[i] ||
            !std::isfinite(motor_min[i]) || !std::isfinite(motor_max[i]) ||
            motor_min[i] >= motor_max[i]) {
            joint = i;
            return ConfigRejection::Range;
        }
        if (nominal[i] - amplitude[i] < model_min[i] ||
            nominal[i] + amplitude[i] > model_max[i]) {
            joint = i;
            return ConfigRejection::Excursion;
        }
    }
    if (!any_excitation) {
        joint = 0;
        return ConfigRejection::NoExcitation;
    }
    return ConfigRejection::None;
}

bool validate_config()
{
    std::size_t joint = 0;
    switch (validate_config_values(kChirpAmplitudeRad, kNominalModelRad,
                                   kModelSafeMinRad, kModelSafeMaxRad,
                                   kMotorSafeMinRad, kMotorSafeMaxRad, joint)) {
        case ConfigRejection::Amplitude:
            std::cerr << "[chirp] invalid amplitude/nominal for " << kJointNames[joint] << "\n";
            return false;
        case ConfigRejection::Range:
            std::cerr << "[chirp] invalid safe range for " << kJointNames[joint] << "\n";
            return false;
        case ConfigRejection::Excursion:
            std::cerr << "[chirp] nominal +/- amplitude exceeds model range for "
                      << kJointNames[joint] << "\n";
            return false;
        case ConfigRejection::NoExcitation:
            std::cerr << "[chirp] all chirp amplitudes are zero\n";
            return false;
        case ConfigRejection::None:
            return true;
    }
    return false;
}

bool wait_command(myactua::MyActMotorController& controller,
                  motor_base::CommandId id,
                  int timeout_ms)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        const auto state = controller.get_discrete_command_result(id);
        if (state == motor_base::DiscreteCommandResult::SUCCEEDED) return true;
        if (state == motor_base::DiscreteCommandResult::FAILED ||
            state == motor_base::DiscreteCommandResult::UNKNOWN) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

bool run_python(const std::string& code, const std::vector<std::string>& arguments)
{
    std::vector<char*> argv;
    argv.reserve(arguments.size() + 4);
    argv.push_back(const_cast<char*>("python3"));
    argv.push_back(const_cast<char*>("-c"));
    argv.push_back(const_cast<char*>(code.c_str()));
    for (const std::string& argument : arguments) argv.push_back(const_cast<char*>(argument.c_str()));
    argv.push_back(nullptr);
    const pid_t child = fork();
    if (child == 0) {
        execvp("python3", argv.data());
        _exit(127);
    }
    if (child < 0) return false;
    int status = 0;
    return waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

std::string python_prefix()
{
    std::filesystem::path root = std::filesystem::current_path();
    std::filesystem::path bundled;
    for (int depth = 0; depth < 8; ++depth) {
        const auto candidate = root / "src/inference/third_party/python_torch";
        if (std::filesystem::exists(candidate)) {
            bundled = candidate;
            break;
        }
        if (root == root.root_path()) break;
        root = root.parent_path();
    }
    return "import sys; sys.path.insert(0, " + std::string("r'") + bundled.string() + "'); ";
}

bool write_csv_and_pt(const ChirpContext& context, const std::filesystem::path& output,
                      bool interrupted)
{
    const std::size_t count = std::min(context.sample_count.load(std::memory_order_acquire), kSampleCapacity);
    const std::filesystem::path csv = output.string() + ".csv";
    std::ofstream stream(csv);
    if (!stream) return false;
    stream << "timestamp_ns,generation_ns,feedback_timestamp_ns,cycle_index,elapsed_s,chirp_elapsed_s,cycle_dt_s,phase,command_applied,working_counter";
    for (std::size_t i = 0; i < kDof; ++i) stream << ",q_target_" << i;
    for (std::size_t i = 0; i < kDof; ++i) stream << ",q_actual_" << i;
    for (std::size_t i = 0; i < kDof; ++i) stream << ",dq_actual_" << i;
    for (std::size_t i = 0; i < kDof; ++i) stream << ",motor_position_" << i;
    for (std::size_t i = 0; i < kDof; ++i) stream << ",motor_velocity_" << i;
    for (std::size_t i = 0; i < kDof; ++i) stream << ",motor_torque_percent_" << i;
    for (std::size_t i = 0; i < kDof; ++i) stream << ",tx_target_pos_raw_" << i;
    for (std::size_t i = 0; i < kDof; ++i) stream << ",tx_target_vel_raw_" << i;
    for (std::size_t i = 0; i < kDof; ++i) stream << ",tx_target_torque_raw_" << i;
    for (std::size_t i = 0; i < kDof; ++i) stream << ",tx_pvt_kp_raw_" << i;
    for (std::size_t i = 0; i < kDof; ++i) stream << ",tx_pvt_kd_raw_" << i;
    for (std::size_t i = 0; i < kDof; ++i) stream << ",tx_op_mode_" << i;
    stream << '\n' << std::setprecision(17);
    for (std::size_t row = 0; row < count; ++row) {
        const Sample& s = context.samples[row];
        stream << s.timestamp_ns << ',' << s.generation_ns << ',' << s.feedback_timestamp_ns << ','
               << s.cycle_index << ',' << s.elapsed_s << ',' << s.chirp_elapsed_s << ',' << s.cycle_dt_s << ','
               << static_cast<int>(s.phase) << ',' << static_cast<int>(s.command_applied) << ',' << s.working_counter;
        const auto append = [&stream](const auto& values) { for (const auto value : values) stream << ',' << value; };
        append(s.q_target); append(s.q_actual); append(s.dq_actual); append(s.motor_position); append(s.motor_velocity); append(s.motor_torque_percent);
        for (const auto& tx : s.tx) stream << ',' << tx.target_pos;
        for (const auto& tx : s.tx) stream << ',' << tx.target_vel;
        for (const auto& tx : s.tx) stream << ',' << tx.target_torque;
        for (const auto& tx : s.tx) stream << ',' << tx.pvt_kp;
        for (const auto& tx : s.tx) stream << ',' << tx.pvt_kd;
        for (const auto& tx : s.tx) stream << ',' << static_cast<int>(tx.op_mode);
        stream << '\n';
    }
    stream.close();
    if (!stream) return false;

    const std::string script = python_prefix() + R"PY(
import csv, json, sys, torch
csv_path, output_path, metadata_json = sys.argv[1], sys.argv[2], sys.argv[3]
with open(csv_path, newline='') as f:
    rows = list(csv.DictReader(f))
def ints(name): return torch.tensor([int(r[name]) for r in rows], dtype=torch.int64)
def floats(prefix): return torch.tensor([[float(r[f'{prefix}_{i}']) for i in range(12)] for r in rows], dtype=torch.float64)
def raw(prefix): return torch.tensor([[int(r[f'{prefix}_{i}']) for i in range(12)] for r in rows], dtype=torch.int64)
data = {
  'timestamp_ns': ints('timestamp_ns'), 'generation_ns': ints('generation_ns'),
  'feedback_timestamp_ns': ints('feedback_timestamp_ns'), 'cycle_index': ints('cycle_index'),
  'elapsed_time': torch.tensor([float(r['elapsed_s']) for r in rows], dtype=torch.float64),
  'chirp_elapsed_time': torch.tensor([float(r['chirp_elapsed_s']) for r in rows], dtype=torch.float64),
  'cycle_dt_s': torch.tensor([float(r['cycle_dt_s']) for r in rows], dtype=torch.float64),
  'phase': ints('phase'), 'q_target': floats('q_target'), 'q_actual': floats('q_actual'),
  'dq_actual': floats('dq_actual'), 'motor_position': floats('motor_position'),
  'motor_velocity': floats('motor_velocity'), 'motor_torque_percent': floats('motor_torque_percent'),
  'working_counter': ints('working_counter'), 'command_applied': ints('command_applied'),
  'tx_target_pos_raw': raw('tx_target_pos_raw'), 'tx_target_vel_raw': raw('tx_target_vel_raw'),
  'tx_target_torque_raw': raw('tx_target_torque_raw'), 'tx_pvt_kp_raw': raw('tx_pvt_kp_raw'),
  'tx_pvt_kd_raw': raw('tx_pvt_kd_raw'), 'tx_op_mode': raw('tx_op_mode'),
  'metadata': json.loads(metadata_json)}
torch.save(data, output_path)
)PY";
    // 配置数值序列化：非有限值输出 null，保证 metadata 始终是合法 JSON
    //（导出属于系统边界；正常运行时 validate_config 已拒绝非有限配置）。
    const auto json_number = [](std::ostringstream& out, double value) {
        if (std::isfinite(value)) out << value;
        else out << "null";
    };
    std::ostringstream metadata;
    metadata << std::setprecision(17) << "{\"clock\":\"CLOCK_MONOTONIC\",\"frequency_hz\":[0.1,10.0],\"duration_s\":20.0,\"joint_names\":[";
    for (std::size_t i = 0; i < kDof; ++i) metadata << (i == 0 ? "" : ",") << '\"' << kJointNames[i] << '\"';
    metadata << "],\"joint_ids_map\":[";
    for (std::size_t i = 0; i < kDof; ++i) metadata << (i == 0 ? "" : ",") << kJointIdsMap[i];
    metadata << "],\"motor_direction\":[";
    for (std::size_t i = 0; i < kDof; ++i) metadata << (i == 0 ? "" : ",") << kMotorDirection[i];
    metadata << "],\"nominal_rad\":[";
    for (std::size_t i = 0; i < kDof; ++i) { metadata << (i == 0 ? "" : ","); json_number(metadata, kNominalModelRad[i]); }
    metadata << "],\"chirp_amplitude_rad\":[";
    for (std::size_t i = 0; i < kDof; ++i) { metadata << (i == 0 ? "" : ","); json_number(metadata, kChirpAmplitudeRad[i]); }
    metadata << "],\"chirp_sign\":[";
    for (std::size_t i = 0; i < kDof; ++i) metadata << (i == 0 ? "" : ",") << kChirpSign[i];
    metadata << "],\"kp\":[";
    for (std::size_t i = 0; i < kDof; ++i) { metadata << (i == 0 ? "" : ","); json_number(metadata, kMotorKp[i]); }
    metadata << "],\"kd\":[";
    for (std::size_t i = 0; i < kDof; ++i) { metadata << (i == 0 ? "" : ","); json_number(metadata, kMotorKd[i]); }
    metadata << "],\"aborted\":" << (context.aborted.load() ? "true" : "false")
             << ",\"abort_reason\":" << context.abort_reason.load()
             << ",\"interrupted\":" << (interrupted ? "true" : "false") << "}";
    return run_python(script, {csv.string(), output.string(), metadata.str()});
}

void fill_enabled_neutral_rx(ChirpContext& context)
{
    for (std::size_t i = 0; i < kDof; ++i) {
        myactua::RxPDO& rx = context.latest_rx[i];
        rx.status_word = myactua::BIT_OPERATION_ENABLED;
        rx.pos = 0;
        rx.vel = 0;
        rx.torque = 0;
        rx.error = 0;
        rx.op_mode = static_cast<std::int8_t>(myactua::MyactControlMode::PVT);
    }
}

// Chirp 起点为零、超过 20 s 停止激励、异常输入收敛到有限值、
// 不规则时间采样仍等于连续时间解析相位、频率线性递增。
bool self_test_chirp_math()
{
    const double nan_value = std::numeric_limits<double>::quiet_NaN();
    if (ChirpContext::linear_chirp(0.0) != 0.0) return false;
    if (ChirpContext::linear_chirp(kChirpDurationS) != 0.0) return false;
    if (ChirpContext::linear_chirp(kChirpDurationS + 1.0) != 0.0) return false;
    for (const double t : {nan_value, -1.0, std::numeric_limits<double>::infinity()}) {
        if (ChirpContext::linear_chirp(t) != 0.0) return false;
    }
    for (const double t : {0.0, 0.0007, 0.7, 7.3, 19.999}) {
        if (!std::isfinite(ChirpContext::linear_chirp(t))) return false;
    }
    const double k = (kFrequencyEndHz - kFrequencyStartHz) / kChirpDurationS;
    for (const double t : {0.0, 1e-4, 0.0091, 0.5, 3.7, 11.3, 19.9999}) {
        const double phase = kTwoPi * (kFrequencyStartHz * t + 0.5 * k * t * t);
        if (std::abs(ChirpContext::linear_chirp(t) - std::sin(phase)) > 1e-12) return false;
    }
    const auto frequency = [](double t) { return kFrequencyStartHz +
        (kFrequencyEndHz - kFrequencyStartHz) * t / kChirpDurationS; };
    if (!(frequency(0.0) == 0.1 && frequency(10.0) > frequency(1.0) && frequency(20.0) == 10.0)) return false;
    if (ChirpContext::smoothstep(0.0) != 0.0 || ChirpContext::smoothstep(1.0) != 1.0) return false;
    if (!std::isfinite(ChirpContext::smoothstep(nan_value))) return false;
    return true;
}

// 阶段边界目标连续、阶段随时间单调推进、仅在总时长后判定 finished。
bool self_test_phase_continuity()
{
    std::array<double, kDof> amplitude{};
    amplitude.fill(0.1);
    std::array<double, kDof> start{};
    start.fill(0.05);
    for (const double boundary : {kRampToNominalS, kNominalStartS, kChirpEndS, kReturnEndS}) {
        std::array<double, kDof> before{};
        std::array<double, kDof> after{};
        ChirpContext::compute_target(std::nextafter(boundary, 0.0), start, amplitude, before);
        ChirpContext::compute_target(boundary, start, amplitude, after);
        for (std::size_t i = 0; i < kDof; ++i) {
            if (std::abs(before[i] - after[i]) > 1e-9) return false;
        }
    }
    std::uint8_t last_phase = 0;
    for (const double t : {0.0, 1.2, 2.999, 3.0, 3.5, 4.0, 4.2, 15.0, 23.999, 24.0,
                           24.5, 25.0, 25.5, 25.999, 26.0, 27.0}) {
        std::array<double, kDof> q{};
        const TargetEval eval = ChirpContext::compute_target(t, start, amplitude, q);
        if (static_cast<std::uint8_t>(eval.phase) < last_phase) return false;
        last_phase = static_cast<std::uint8_t>(eval.phase);
        for (const double value : q) {
            if (!std::isfinite(value)) return false;
        }
    }
    std::array<double, kDof> q{};
    if (ChirpContext::compute_target(std::nextafter(kExperimentEndS, 0.0), start, amplitude, q).finished) return false;
    if (!ChirpContext::compute_target(kExperimentEndS, start, amplitude, q).finished) return false;
    // NaN elapsed：全部比较为假，按已完成处理并输出有限目标（快速失败）。
    const TargetEval nan_eval = ChirpContext::compute_target(
        std::numeric_limits<double>::quiet_NaN(), start, amplitude, q);
    if (!nan_eval.finished) return false;
    for (const double value : q) {
        if (!std::isfinite(value)) return false;
    }
    return true;
}

// 配置拒绝条件：未填写（NaN）、全零幅值、乱序/NaN 安全范围、超出模型范围。
bool self_test_config_rejection()
{
    std::array<double, kDof> amplitude{};
    amplitude.fill(0.1);
    std::array<double, kDof> nominal{};
    std::array<double, kDof> lower{};
    lower.fill(-0.5);
    std::array<double, kDof> upper{};
    upper.fill(0.5);
    std::array<double, kDof> nan_values{};
    nan_values.fill(std::numeric_limits<double>::quiet_NaN());
    std::array<double, kDof> zero{};
    std::size_t joint = 0;

    if (validate_config_values(amplitude, nominal, lower, upper, lower, upper, joint) !=
        ConfigRejection::None) return false;
    joint = 0;
    if (validate_config_values(nan_values, nominal, lower, upper, lower, upper, joint) !=
        ConfigRejection::Amplitude) return false;
    joint = 0;
    if (validate_config_values(zero, nominal, lower, upper, lower, upper, joint) !=
        ConfigRejection::NoExcitation) return false;
    joint = 0;
    if (validate_config_values(amplitude, nominal, upper, lower, lower, upper, joint) !=
        ConfigRejection::Range) return false;
    joint = 0;
    if (validate_config_values(amplitude, nominal, lower, upper, nan_values, nan_values, joint) !=
        ConfigRejection::Range) return false;
    std::array<double, kDof> too_big{};
    too_big.fill(0.6);
    joint = 0;
    if (validate_config_values(too_big, nominal, lower, upper, lower, upper, joint) !=
        ConfigRejection::Excursion) return false;
    return true;
}

// 映射拒绝条件：非法 dof、方向非 ±1、电机重复、直驱尺寸不符、踝映射越界。
bool self_test_mapping_rejection()
{
    using inference::robot_detail::JointMapping;
    const auto count = static_cast<int>(kDof);
    std::string error;
    const inference::JointMappingConfig valid_config;
    if (!JointMapping::create(count, valid_config, error)) return false;

    if (JointMapping::create(0, valid_config, error)) return false;

    inference::JointMappingConfig bad_direction = valid_config;
    bad_direction.motor_to_model_direction[0] = 0;
    if (JointMapping::create(count, bad_direction, error)) return false;

    inference::JointMappingConfig duplicate = valid_config;
    duplicate.model_to_motor_index[0] = duplicate.model_to_motor_index[1];
    if (JointMapping::create(count, duplicate, error)) return false;

    inference::JointMappingConfig wrong_size = valid_config;
    wrong_size.model_to_motor_index.pop_back();
    if (JointMapping::create(count, wrong_size, error)) return false;

    inference::JointMappingConfig bad_ankle = valid_config;
    bad_ankle.left_ankle_parallel.upper_motor_index = -1;
    if (JointMapping::create(count, bad_ankle, error)) return false;
    return true;
}

// RT 计算路径：异常反馈必须被拒绝；稳态 decode + 目标计算 + ActionProcessor
// 全程零动态分配（mallinfo2 审计），输出有限。
bool self_test_rt_compute_audit()
{
    auto context = std::make_unique<ChirpContext>();
    context->jacobian_error.reserve(256);
    context->action_error.reserve(256);
    std::string mapping_error;
    context->mapping = inference::robot_detail::JointMapping::create(
        static_cast<int>(kDof), inference::JointMappingConfig{}, mapping_error);
    if (!context->mapping) return false;

    inference::ActionConfig action_config;
    action_config.default_joint_pos_rad = kNominalModelRad;
    inference::AnkleMotorLimitConfig ankle_limits;
    inference::AnkleTorqueControlConfig torque_config;
    torque_config.virtual_kp = {180.0, 180.0};
    torque_config.virtual_kd = {10.0, 10.0};
    inference::robot_detail::ActionProcessor processor(
        context->mapping, action_config, ankle_limits, kMotorKp, kMotorKd, torque_config);
    context->action_processor = &processor;
    fill_enabled_neutral_rx(*context);

    std::array<motor_base::MotorStatusSnapshot, kDof> feedback{};
    std::array<double, kDof> q{};
    std::array<double, kDof> dq{};
    {
        const myactua::RxPDO saved = context->latest_rx[3];
        context->latest_rx[3].status_word = 0;
        if (context->decode_model_state(feedback, q, dq)) return false;
        context->latest_rx[3] = saved;
        context->latest_rx[3].status_word = myactua::BIT_OPERATION_ENABLED | myactua::BIT_FAULT;
        if (context->decode_model_state(feedback, q, dq)) return false;
        context->latest_rx[3] = saved;
        context->latest_rx[3].error = 1;
        if (context->decode_model_state(feedback, q, dq)) return false;
        context->latest_rx[3] = saved;
    }

    if (!context->decode_model_state(feedback, q, dq)) return false;
    std::array<double, kDof> amplitude{};
    amplitude.fill(0.1);
    std::array<double, kDof> target{};
    inference::robot_detail::ActionProcessor::FixedPolicyMotorCommand command;
    ChirpContext::compute_target(4.5, kNominalModelRad, amplitude, target);
    if (!context->action_processor->build_policy_impedance_command(
            target, feedback, command, context->action_error)) return false;

    const struct mallinfo2 before = mallinfo2();
    for (int cycle = 0; cycle < 100; ++cycle) {
        if (!context->decode_model_state(feedback, q, dq)) return false;
        ChirpContext::compute_target(kNominalStartS + 0.001 * cycle, kNominalModelRad, amplitude, target);
        if (!context->action_processor->build_policy_impedance_command(
                target, feedback, command, context->action_error)) return false;
        for (const auto& setpoint : command.setpoints) {
            if (!std::isfinite(setpoint.position_rad) || !std::isfinite(setpoint.velocity_rad_s) ||
                !std::isfinite(setpoint.effort_ff) || !std::isfinite(setpoint.kp) ||
                !std::isfinite(setpoint.kd)) return false;
        }
    }
    const struct mallinfo2 after = mallinfo2();
    return after.uordblks == before.uordblks && after.hblks == before.hblks;
}

// 生成/下发样本对应：send_rt 写入的样本携带生成时间、阶段、目标与实际 TxPDO；
// PDO 集不完整时立即终止且不写样本。
bool self_test_sample_pairing()
{
    auto context = std::make_unique<ChirpContext>();
    ChirpAdapter adapter(*context);
    fill_enabled_neutral_rx(*context);

    context->applying.valid = true;
    context->applying.generation_ns = 1'700'000'000'000'000;
    context->applying.elapsed_s = 4.25;
    context->applying.chirp_elapsed_s = 0.25;
    context->applying.phase = Phase::Chirp;
    context->applying.q_target.fill(0.07);

    myactua::TxPDO pdo{};
    pdo.target_pos = 42;
    for (std::size_t i = 0; i < kDof; ++i) context->tx_rt(static_cast<int>(i), pdo);
    context->send_rt(adapter);
    if (context->sample_count.load() != 1) return false;
    const Sample& sample = context->samples[0];
    if (sample.generation_ns != 1'700'000'000'000'000 ||
        sample.elapsed_s != 4.25 || sample.chirp_elapsed_s != 0.25 ||
        sample.phase != static_cast<std::uint8_t>(Phase::Chirp) ||
        sample.command_applied != 1 || sample.cycle_index != 0 ||
        sample.timestamp_ns <= 0) return false;
    for (std::size_t i = 0; i < kDof; ++i) {
        if (sample.q_target[i] != 0.07 || sample.tx[i].target_pos != 42) return false;
    }

    context->tx_rt(0, pdo);
    context->send_rt(adapter);
    return context->aborted.load() && context->abort_reason.load() == 8 &&
           context->sample_count.load() == 1;
}

// 导出验证：合成样本走真实 CSV -> torch.save 路径，用独立脚本检查
// 形状、dtype、float64 精度回读与 metadata（含终止原因）。
bool self_test_export_round_trip()
{
    auto context = std::make_unique<ChirpContext>();
    for (std::size_t row = 0; row < 3; ++row) {
        Sample& sample = context->samples[row];
        sample.timestamp_ns = 1'700'000'000'000'000 + static_cast<std::int64_t>(row) * 1'000'000;
        sample.generation_ns = sample.timestamp_ns - 500'000;
        sample.feedback_timestamp_ns = sample.timestamp_ns;
        sample.cycle_index = row;
        sample.elapsed_s = 0.001 * static_cast<double>(row);
        sample.chirp_elapsed_s = -1.0;
        sample.cycle_dt_s = 0.001;
        sample.phase = static_cast<std::uint8_t>(row);
        sample.command_applied = 1;
        sample.working_counter = 12;
        for (std::size_t i = 0; i < kDof; ++i) {
            sample.q_target[i] = 0.25 * static_cast<double>(row) + 0.001 * static_cast<double>(i);
            sample.q_actual[i] = -0.5;
            sample.dq_actual[i] = 0.0;
            sample.motor_position[i] = 0.3;
            sample.motor_velocity[i] = 0.0;
            sample.motor_torque_percent[i] = 0.0;
            sample.tx[i].target_pos = 100 + static_cast<std::int32_t>(row);
        }
    }
    context->sample_count.store(3);

    const std::string stem = (std::filesystem::temp_directory_path() /
        ("chirp_selftest_" + std::to_string(static_cast<long>(::getpid())))).string();
    const std::filesystem::path output = stem + ".pt";
    const std::filesystem::path csv = stem + ".csv";
    std::error_code ec;
    std::filesystem::remove(output, ec);
    std::filesystem::remove(csv, ec);
    if (!write_csv_and_pt(*context, output, true)) return false;

    const std::string verify = python_prefix() + R"PY(
import sys, torch
data = torch.load(sys.argv[1], weights_only=True)
assert data['q_target'].shape == (3, 12)
assert data['q_target'].dtype == torch.float64
assert data['timestamp_ns'].dtype == torch.int64
assert data['timestamp_ns'][0].item() == 1700000000000000
assert abs(data['q_target'][2][11].item() - (0.25*2 + 0.001*11)) < 1e-15
assert data['tx_target_pos_raw'].dtype == torch.int64
assert data['tx_target_pos_raw'][1][0].item() == 101
assert data['working_counter'].tolist() == [12, 12, 12]
meta = data['metadata']
assert meta['joint_names'][0] == 'left_hip_roll' and len(meta['joint_names']) == 12
assert meta['joint_ids_map'] == [0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11]
assert meta['kp'][0] == 180.0 and meta['kd'][0] == 10.0
assert meta['aborted'] is False and meta['abort_reason'] == 0
assert meta['interrupted'] is True
)PY";
    const bool verified = run_python(verify, {output.string()});
    std::filesystem::remove(output, ec);
    std::filesystem::remove(csv, ec);
    return verified;
}

bool self_test()
{
    if (!self_test_chirp_math()) return false;
    if (!self_test_phase_continuity()) return false;
    if (!self_test_config_rejection()) return false;
    if (!self_test_mapping_rejection()) return false;
    if (!self_test_rt_compute_audit()) return false;
    if (!self_test_sample_pairing()) return false;
    if (!self_test_export_round_trip()) return false;
    for (const auto& name : kJointNames) if (std::strlen(name) == 0) return false;
    const std::string check = python_prefix() + "import torch, io; b=io.BytesIO(); torch.save({'q': torch.zeros((1,12), dtype=torch.float64)}, b); b.seek(0); assert tuple(torch.load(b, weights_only=True)['q'].shape)==(1,12)";
    return run_python(check, {});
}

}  // namespace

int main(int argc, char** argv)
{
    bool run_self_test = false;
    std::filesystem::path output = "chirp_data.pt";
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--self-test") == 0) run_self_test = true;
        else if (std::strcmp(argv[i], "--output") == 0 && i + 1 < argc) output = argv[++i];
        else {
            std::cerr << "usage: chirp_test [--self-test] [--output chirp_data.pt]\n";
            return 2;
        }
    }
    if (run_self_test) {
        const bool passed = self_test();
        std::cout << "[chirp] self-test " << (passed ? "passed" : "failed") << '\n';
        return passed ? 0 : 1;
    }
    if (!validate_config()) return 2;
    if (std::filesystem::exists(output) || std::filesystem::exists(output.string() + ".csv")) {
        std::cerr << "[chirp] output already exists; choose a new --output path\n";
        return 2;
    }
    const std::string python_check = python_prefix() + "import torch; assert torch.__version__";
    if (!run_python(python_check, {})) {
        std::cerr << "[chirp] Python/PyTorch exporter preflight failed\n";
        return 2;
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    static ChirpContext context;
    context.jacobian_error.reserve(256);
    context.action_error.reserve(256);
    // The RT writer must not fault in this sizeable static log on its first
    // sample.  Touch every page before starting EtherCAT scheduling.
    volatile unsigned char* log_pages = reinterpret_cast<volatile unsigned char*>(context.samples.data());
    for (std::size_t offset = 0; offset < sizeof(context.samples); offset += 4096) {
        log_pages[offset] = 0;
    }
    ChirpAdapter adapter(context);
    myactua::MyActMotorController::Options options;
    options.rt_period_ns = 1'000'000;
    options.status_publish_period_ms = 1;
    options.comm_watchdog_fault_cycles = 10;
    options.rt_thread_options = {{}, robot_base::ThreadSchedulingPolicy::FIFO, 80, 0};
    ChirpMotorController controller(std::shared_ptr<myactua::EthercatAdapter>(&adapter, [](myactua::EthercatAdapter*) {}),
                                    static_cast<int>(kDof), options);
    context.controller = &controller;

    inference::JointMappingConfig mapping_config;
    mapping_config.model_to_motor_index.assign(kJointIdsMap.begin(), kJointIdsMap.begin() + 8);
    mapping_config.motor_to_model_direction = kMotorDirection;
    std::string mapping_error;
    context.mapping = inference::robot_detail::JointMapping::create(static_cast<int>(kDof), mapping_config, mapping_error);
    if (!context.mapping) {
        std::cerr << "[chirp] joint mapping invalid: " << mapping_error << '\n';
        return 2;
    }
    inference::ActionConfig action_config;
    action_config.default_joint_pos_rad = kNominalModelRad;
    inference::AnkleMotorLimitConfig ankle_limits;
    ankle_limits.min_rad = {kMotorSafeMinRad[4], kMotorSafeMinRad[5], kMotorSafeMinRad[10], kMotorSafeMinRad[11]};
    ankle_limits.max_rad = {kMotorSafeMaxRad[4], kMotorSafeMaxRad[5], kMotorSafeMaxRad[10], kMotorSafeMaxRad[11]};
    inference::AnkleTorqueControlConfig torque_config;
    torque_config.virtual_kp = {180.0, 180.0};
    torque_config.virtual_kd = {10.0, 10.0};
    inference::robot_detail::ActionProcessor action_processor(context.mapping, action_config, ankle_limits,
                                                               kMotorKp, kMotorKd, torque_config);
    context.action_processor = &action_processor;

    std::cout << "[chirp] duration=20 s frequency=0.1->10 Hz control_rate=1000 Hz joints=12\n";
    if (!controller.connect() || !controller.wait_all_motors_ready(20'000, 100)) {
        std::cerr << "[chirp] EtherCAT is not ready\n";
        return 1;
    }
    controller.set_print_info({});
    if (!controller.start()) {
        std::cerr << "[chirp] failed to start EtherCAT RT thread\n";
        return 1;
    }
    bool stop_requested = false;
    for (int i = 0; i < static_cast<int>(kDof); ++i) {
        const auto result = controller.send_discrete_command(
            motor_base::ControlCommand::set_mode(motor_base::MotorControlMode::IMPEDANCE, i));
        if (result.status != motor_base::CommandSubmitStatus::ACCEPTED || !result.command_id ||
            !wait_command(controller, *result.command_id, 4'000)) {
            std::cerr << "[chirp] failed to enter impedance mode\n";
            stop_requested = true;
            break;
        }
    }
    if (!stop_requested) {
        const auto result = controller.send_discrete_command(motor_base::ControlCommand::restart());
        if (result.status != motor_base::CommandSubmitStatus::ACCEPTED || !result.command_id ||
            !wait_command(controller, *result.command_id, 4'000)) {
            std::cerr << "[chirp] failed to enable motors\n";
            stop_requested = true;
        }
    }
    if (!stop_requested) context.armed.store(true, std::memory_order_release);
    while (!stop_requested && !g_interrupt.load(std::memory_order_acquire) &&
           !context.finished.load(std::memory_order_acquire) && !context.aborted.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    context.armed.store(false, std::memory_order_release);
    const auto stop = controller.send_discrete_command(motor_base::ControlCommand::stop());
    const bool stopped = stop.command_id && wait_command(controller, *stop.command_id, 4'000);
    if (stopped) controller.shutdown();
    const bool exported = write_csv_and_pt(context, output, g_interrupt.load(std::memory_order_acquire));
    if (!exported) std::cerr << "[chirp] export failed; CSV may be available beside " << output << '\n';
    if (!stopped) {
        std::cerr << "[chirp] STOP was not confirmed; RT controller remains active\n";
        return 1;
    }
    std::cout << "[chirp] samples=" << context.sample_count.load() << " output=" << output << '\n';
    return (context.aborted.load() || !exported || g_interrupt.load()) ? 1 : 0;
}
