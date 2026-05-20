#include "robot_interface.hpp"
#include "torch_policy_runner.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <iostream>
#include <thread>
#include <utility>
#include <vector>


namespace inference {
namespace {

constexpr double kPi = 3.14159265358979323846;

bool finite_array3(const std::array<double, 3>& values)
{
    return std::isfinite(values[0]) &&
           std::isfinite(values[1]) &&
           std::isfinite(values[2]);
}

bool finite_array4(const std::array<double, 4>& values)
{
    return std::isfinite(values[0]) &&
           std::isfinite(values[1]) &&
           std::isfinite(values[2]) &&
           std::isfinite(values[3]);
}

bool finite_vector(const std::vector<double>& values)
{
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    });
}

bool valid_motor_direction_values(const std::vector<int>& directions)
{
    return std::all_of(directions.begin(), directions.end(), [](int direction) {
        return direction == 1 || direction == -1;
    });
}

int direction_for_motor(const RobotInterfaceConfig& config, int motor_index)
{
    if (config.motor_to_model_direction.empty()) {
        return 1;
    }
    return config.motor_to_model_direction[motor_index];
}

bool compute_projected_gravity(const std::array<double, 4>& quat,
                               std::array<double, 3>& projected_gravity)
{
    if (!finite_array4(quat)) {
        return false;
    }

    const double norm_sq = quat[0] * quat[0] +
                           quat[1] * quat[1] +
                           quat[2] * quat[2] +
                           quat[3] * quat[3];
    if (!std::isfinite(norm_sq) || norm_sq <= 1.0e-12) {
        return false;
    }

    const double inv_norm = 1.0 / std::sqrt(norm_sq);
    const double w = quat[0] * inv_norm;
    const double x = quat[1] * inv_norm;
    const double y = quat[2] * inv_norm;
    const double z = quat[3] * inv_norm;

    // IsaacLab quat_rotate_inverse(q, [0, 0, -1]): world gravity in body frame.
    constexpr std::array<double, 3> gravity{0.0, 0.0, -1.0};
    const double cross_x = y * gravity[2] - z * gravity[1];
    const double cross_y = z * gravity[0] - x * gravity[2];
    const double cross_z = x * gravity[1] - y * gravity[0];
    const double dot = x * gravity[0] + y * gravity[1] + z * gravity[2];
    const double a_scale = 2.0 * w * w - 1.0;

    projected_gravity[0] = gravity[0] * a_scale - 2.0 * w * cross_x + 2.0 * x * dot;
    projected_gravity[1] = gravity[1] * a_scale - 2.0 * w * cross_y + 2.0 * y * dot;
    projected_gravity[2] = gravity[2] * a_scale - 2.0 * w * cross_z + 2.0 * z * dot;

    return finite_array3(projected_gravity);
}

template <std::size_t TermSize, std::size_t ObservationSize>
void fill_term_history(std::array<float, ObservationSize>& history,
                       std::size_t offset,
                       std::size_t frame_stack,
                       const std::array<float, TermSize>& current_term)
{
    for (std::size_t frame = 0; frame < frame_stack; ++frame) {
        std::copy(current_term.begin(),
                  current_term.end(),
                  history.begin() + offset + frame * TermSize);
    }
}

template <std::size_t TermSize, std::size_t ObservationSize>
void append_term_history(std::array<float, ObservationSize>& history,
                         std::size_t offset,
                         std::size_t frame_stack,
                         const std::array<float, TermSize>& current_term)
{
    const std::size_t term_history_size = frame_stack * TermSize;
    std::copy(history.begin() + offset + TermSize,
              history.begin() + offset + term_history_size,
              history.begin() + offset);
    std::copy(current_term.begin(),
              current_term.end(),
              history.begin() + offset + term_history_size - TermSize);
}

bool is_mit_mode(myactua::ControlMode mode)
{
    return mode == myactua::ControlMode::PVT;
}

}  // namespace

RobotInterface::RobotInterface(RobotInterfaceConfig config)
    : config_(std::move(config)) {}

RobotInterface::~RobotInterface() {
    unload_policy();
    deinit_imu();
    deinit_motors();
}



/********************************************************************* */
bool RobotInterface::initial_and_start_imu() {
    if (imu_initialized_.load()) {
        return true;
    }

    imu::Config_t imu_cfg;
    imu_cfg.device      = config_.imu_device;
    imu_cfg.baudrate    = config_.imu_baudrate;
    imu_cfg.print_imu   = config_.imu_print_imu;
    imu_cfg.print_ahrs  = config_.imu_print_ahrs;
    imu_cfg.print_stats = config_.imu_print_stats;

    ahrs_ready_.store(false);
    imu_reader_ = std::make_unique<imu::IMUReader>();
    imu_reader_->set_imu_callback([this](const imu::IMUData_t& data) {
        (void)data;
    });
    imu_reader_->set_ahrs_callback([this](const imu::AHRSData_t& data) {
        // 策略输入使用 AHRS 的角速度、欧拉角和四元数原始值；Yaw 不做启动零偏。
        std::lock_guard<std::mutex> lock(imu_mutex_);
        ang_vel_[0] = static_cast<double>(data.roll_speed);
        ang_vel_[1] = static_cast<double>(data.pitch_speed);
        ang_vel_[2] = static_cast<double>(data.heading_speed);
        body_ang_vel_ = ang_vel_;

        euler_[0] = static_cast<double>(data.roll);
        euler_[1] = static_cast<double>(data.pitch);
        euler_[2] = static_cast<double>(data.heading);
        
        quat_[0] = static_cast<double>(data.qw);
        quat_[1] = static_cast<double>(data.qx);
        quat_[2] = static_cast<double>(data.qy);
        quat_[3] = static_cast<double>(data.qz);
        ahrs_ready_.store(true);
    });

    if (!imu_reader_->start(imu_cfg)) {
        std::cerr << "[RobotInterface] IMU start failed.\n";
        imu_reader_.reset();
        imu_initialized_.store(false);
        return false;
    }

    imu_initialized_.store(true);
    return true;
}


void RobotInterface::deinit_imu() {
    if (imu_reader_) {
        imu_reader_->stop();
    }
    imu_reader_.reset();
    imu_initialized_.store(false);
    ahrs_ready_.store(false);
}

/* 获取IMU角度(从AHRS帧得到) */
std::array<double, 4> RobotInterface::get_quat() const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return quat_;
}

//** 获取角速度(从AHRS帧得到) */
std::array<double, 3> RobotInterface::get_ang_vel() const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return ang_vel_;
}

std::array<double, 3> RobotInterface::get_body_ang_vel() const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return body_ang_vel_;
}

std::array<double, 3> RobotInterface::get_euler() const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return euler_;
}


/*************************************************************************** */
/* 初始化并启动电机 */
bool RobotInterface::initial_and_start_motors() {
    if (motors_initialized_.load()) {
        return true;
    }

    if (!validate_motor_config()) {
        return false;
    }

    adapter_    = std::make_shared<myactua::EthercatAdapterIGH>();
    controller_ = std::make_unique<myactua::MYACTUA>(adapter_, config_.num_motors);

    std::cout << "[RobotInterface] Connecting EtherCAT on "
              << config_.ethercat_ifname << "...\n";

    if (!controller_->connect(config_.ethercat_ifname.c_str())) {
        std::cerr << "[RobotInterface] EtherCAT connect failed.\n";
        controller_.reset();
        adapter_.reset();
        return false;
    }

    if (!controller_->wait_all_slaves_ready(config_.wait_all_slaves_timeout_ms,
                                            config_.wait_all_slaves_poll_ms)) {
        std::cerr << "[RobotInterface] Not all slaves became ready in timeout.\n";
        controller_.reset();
        adapter_.reset();
        return false;
    }

    /* 设置电机控制模式 */
    for (int i = 0; i < config_.num_motors; ++i) {
        controller_->send_command(myactua::ControlCommand(myactua::CommandType::SET_MODE,
                                                             i,
                                                            {},
                                                               config_.motor_control_mode));
    }

    /* 根据配置决定是否打印电机信息 */
    if (config_.print_motors_info) {
        controller_->set_print_info(config_.print_motor_ids);
    } else {
        controller_->set_print_info({});
    }

    /* 启动实时线程 */
    controller_->start();

    controller_->send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
    motors_initialized_.store(true);
    motion_enabled_.store(false);

    return true;
}


bool RobotInterface::validate_motor_config() const {
    if (config_.num_motors <= 0) {
        std::cerr << "[RobotInterface] num_motors must be positive\n";
        return false;
    }

    if (!config_.motor_to_model_direction.empty()) {
        if (static_cast<int>(config_.motor_to_model_direction.size()) != config_.num_motors) {
            std::cerr << "[RobotInterface] motor_to_model_direction size must be "
                      << config_.num_motors << " or empty, got="
                      << config_.motor_to_model_direction.size() << "\n";
            return false;
        }
        if (!valid_motor_direction_values(config_.motor_to_model_direction)) {
            std::cerr << "[RobotInterface] motor_to_model_direction values must be 1 or -1\n";
            return false;
        }
    }

    if (is_mit_mode(config_.motor_control_mode)) {
        if (static_cast<int>(config_.mit_kp.size()) != config_.num_motors) {
            std::cerr << "[RobotInterface] MIT mode requires mit_kp size="
                      << config_.num_motors << ", got=" << config_.mit_kp.size()
                      << "\n";
            return false;
        }
        if (static_cast<int>(config_.mit_kd.size()) != config_.num_motors) {
            std::cerr << "[RobotInterface] MIT mode requires mit_kd size="
                      << config_.num_motors << ", got=" << config_.mit_kd.size()
                      << "\n";
            return false;
        }
        if (!finite_vector(config_.mit_kp) || !finite_vector(config_.mit_kd)) {
            std::cerr << "[RobotInterface] MIT mode requires finite mit_kp/mit_kd values\n";
            return false;
        }
    }

    return true;
}


void RobotInterface::deinit_motors() {
    if (motors_initialized_.load() && controller_) {
        stop_motors(-1);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        controller_->shutdown();
    }

    controller_.reset();
    adapter_.reset();
    motors_initialized_.store(false);
    motion_enabled_.store(false);
}


bool RobotInterface::stop_motors(int slave_index) {
    if (!motors_initialized_.load() || !controller_) {
        return false;
    }
    if (slave_index >= config_.num_motors) {
        std::cerr << "[RobotInterface] stop_motors invalid slave_index="
                  << slave_index << "\n";
        return false;
    }

    controller_->send_command(myactua::ControlCommand(myactua::CommandType::STOP, slave_index));
    if (slave_index < 0) {
        motion_enabled_.store(false);
    }
    return true;
}


bool RobotInterface::restart_motors(int slave_index) {
    if (!motors_initialized_.load() || !controller_) {
        return false;
    }
    if (slave_index >= config_.num_motors) {
        std::cerr << "[RobotInterface] restart_motors invalid slave_index="
                  << slave_index << "\n";
        return false;
    }

    controller_->send_command(myactua::ControlCommand(myactua::CommandType::RESTART,
                                                      slave_index));
    if (slave_index < 0 || config_.num_motors == 1) {
        motion_enabled_.store(true);
    }
    return true;
}

/* 输入的关节目标位置（弧度）按照模型电机索引、坐标系; 并完成限位检查; 并完成坐标系反转 */
bool RobotInterface::apply_action(const std::vector<double>& target_q_model_rad) {
    if (!motors_initialized_.load() || !controller_) {
        return false;
    }
    if (!motion_enabled_.load()) {
        std::cerr << "[RobotInterface] apply_action rejected: motors are stopped. "
                  << "Call restart_motors(-1) first.\n";
        return false;
    }
    if (static_cast<int>(target_q_model_rad.size()) != config_.num_motors) {
        std::cerr << "[RobotInterface] apply_action size mismatch. expected="
                  << config_.num_motors << " got=" << target_q_model_rad.size() << "\n";
        return false;
    }

    std::vector<double> target_rad(config_.num_motors, 0.0);
    const bool has_model_mapping =
        static_cast<int>(config_.model_to_motor_index.size()) == config_.num_motors;
    const bool has_relative_limits =
        config_.stand_pose_rad.size() == static_cast<size_t>(config_.num_motors) &&
        config_.joint_min_rad.size()  == static_cast<size_t>(config_.num_motors) &&
        config_.joint_max_rad.size()  == static_cast<size_t>(config_.num_motors);


    for (int model_index = 0; model_index < config_.num_motors; ++model_index) {
        const int motor_index = has_model_mapping
            ? config_.model_to_motor_index[model_index]
            : model_index;
        if (motor_index < 0 || motor_index >= config_.num_motors) {
            std::cerr << "[RobotInterface] apply_action rejected: "
                      << "model_to_motor_index contains invalid motor index "
                      << motor_index << "\n";
            return false;
        }

        double q = target_q_model_rad[model_index];
        if (has_relative_limits) {
            const double lower_limit =
                config_.stand_pose_rad[model_index] + config_.joint_min_rad[model_index];
            const double upper_limit =
                config_.stand_pose_rad[model_index] + config_.joint_max_rad[model_index];
            q = std::max(lower_limit, std::min(upper_limit, q));
        }
        /* 进行坐标系映射 */
        target_rad[motor_index] =
            static_cast<double>(direction_for_motor(config_, motor_index)) * q;
    }

    if (is_mit_mode(config_.motor_control_mode)) {
        std::vector<myactua::MitSetpoint> mit_setpoints(config_.num_motors);
        for (int i = 0; i < config_.num_motors; ++i) {
            mit_setpoints[i] = myactua::MitSetpoint(myactua::MYACTUA::rad_to_deg(target_rad[i]),
                                                    0.0,
                                                    0.0,
                                                    config_.mit_kp[i],
                                                    config_.mit_kd[i]);
        }
        controller_->send_command(myactua::ControlCommand(
            myactua::CommandType::SET_MIT_SETPOINTS, -1, mit_setpoints));
        return true;
    }

    if (config_.motor_control_mode != myactua::ControlMode::CSP) {
        std::cerr << "[RobotInterface] apply_action supports only MIT/PVT or CSP mode\n";
        return false;
    }

    std::vector<double> target_deg(config_.num_motors, 0.0);
    for (int i = 0; i < config_.num_motors; ++i) {
        target_deg[i] = myactua::MYACTUA::rad_to_deg(target_rad[i]);
    }

    controller_->send_command(myactua::ControlCommand(myactua::CommandType::SET_SETPOINTS,
                                                      -1,
                                                      target_deg));
    return true;
}


std::vector<double> RobotInterface::get_joint_q() const {
    std::vector<double> q(config_.num_motors, 0.0);
    if (!controller_) {
        return q;
    }
    q = controller_->get_joint_q_rad();
    return q;
}


std::vector<double> RobotInterface::get_joint_vel() const {
    std::vector<double> dq(config_.num_motors, 0.0);
    if (!controller_) {
        return dq;
    }
    dq = controller_->get_joint_vel_rad_s();
    return dq;
}


std::vector<double> RobotInterface::get_joint_tau() const {
    std::vector<double> tau(config_.num_motors, 0.0);
    if (!controller_) {
        return tau;
    }
    tau = controller_->get_joint_tau_raw();
    return tau;
}

/* 恢复到初始姿态 */
bool RobotInterface::reset_joints() {
    if (!motors_initialized_.load() || !controller_) {
        return false;
    }
    if (!motion_enabled_.load()) {
        std::cerr << "[RobotInterface] reset_joints rejected: motors are stopped. "
                  << "Call restart_motors(-1) first.\n";
        return false;
    }

    std::vector<double> target_model(config_.num_motors, 0.0);
    if (static_cast<int>(config_.stand_pose_rad.size()) == config_.num_motors) {
        target_model = config_.stand_pose_rad;
    }

    const std::vector<double> q0_motor = get_joint_q();
    std::vector<double> q0_model(config_.num_motors, 0.0);

    const bool has_model_mapping =
        static_cast<int>(config_.model_to_motor_index.size()) == config_.num_motors;

    for (int model_index = 0; model_index < config_.num_motors; ++model_index) {
        const int motor_index = has_model_mapping
            ? config_.model_to_motor_index[model_index]
            : model_index;
        if (motor_index < 0 || motor_index >= config_.num_motors) {
            std::cerr << "[RobotInterface] reset_joints rejected: "
                      << "model_to_motor_index contains invalid motor index "
                      << motor_index << "\n";
            return false;
        }
        q0_model[model_index] =
            static_cast<double>(direction_for_motor(config_, motor_index)) *
            q0_motor[motor_index];
    }

    const int ramp_steps = 100;
    const auto dt = std::chrono::milliseconds(20);

    for (int k = 1; k <= ramp_steps; ++k) {
        const double alpha = static_cast<double>(k) / static_cast<double>(ramp_steps);
        std::vector<double> q_cmd_model(config_.num_motors, 0.0);
        for (int i = 0; i < config_.num_motors; ++i) {
            q_cmd_model[i] = q0_model[i] * (1.0 - alpha) + target_model[i] * alpha;
        }
        if (!apply_action(q_cmd_model)) {
            return false;
        }
        std::this_thread::sleep_for(dt);
    }

    return true;
}


/*************************************************************************** */
bool RobotInterface::load_policy() {
    std::lock_guard<std::mutex> lock(policy_mutex_);
    // 加载前先清掉旧 runner 和上一周期动作，避免失败后残留旧策略状态。
    policy_runner_.reset();
    last_action_raw_.fill(0.0F);
    observation_history_.fill(0.0F);
    observation_history_ready_ = false;
    policy_start_time_ = std::chrono::steady_clock::now();

    if (!validate_policy_config()) {
        return false;
    }

    auto runner = std::make_unique<TorchPolicyRunner>();
    if (!runner->load(config_.policy_model_path)) {
        std::cerr << "[RobotInterface] load_policy failed: "
                  << runner->last_error() << "\n";
        return false;
    }

    policy_runner_ = std::move(runner);
    return true;
}

/* 检查 policy 参数是否有效 */
bool RobotInterface::validate_policy_config() const {
    auto fail = [](const std::string& message) {
        std::cerr << "[RobotInterface] invalid policy config: "
                  << message << "\n";
        return false;
    };

    if (config_.num_motors != kPolicyDof) {
        return fail("num_motors must be 12");
    }
    if (config_.policy_model_path.empty()) {
        return fail("policy_model_path is empty");
    }
#if POLICY_V3
    if (!std::isfinite(config_.policy_cycle_time_s) || config_.policy_cycle_time_s <= 0.0) {
        return fail("policy_cycle_time_s must be finite and > 0");
    }
#endif
    if (!std::isfinite(config_.action_clip) || config_.action_clip <= 0.0) {
        return fail("action_clip must be finite and > 0");
    }
    if (config_.stand_pose_rad.size() != static_cast<std::size_t>(kPolicyDof)) {
        return fail("stand_pose_rad must have 12 values");
    }
    if (config_.action_scale.size() != static_cast<std::size_t>(kPolicyDof)) {
        return fail("action_scale must have 12 values");
    }
    if (config_.joint_min_rad.size() != static_cast<std::size_t>(kPolicyDof) ||
        config_.joint_max_rad.size() != static_cast<std::size_t>(kPolicyDof)) {
        return fail("joint_min_rad and joint_max_rad must have 12 values");
    }
    if (config_.model_to_motor_index.size() != static_cast<std::size_t>(kPolicyDof)) {
        return fail("model_to_motor_index must have 12 values");
    }
    if (!config_.motor_to_model_direction.empty() &&
        config_.motor_to_model_direction.size() != static_cast<std::size_t>(kPolicyDof)) {
        return fail("motor_to_model_direction must have 12 values or be empty");
    }
    if (config_.dof_pos_scale.size() != static_cast<std::size_t>(kPolicyDof) ||
        config_.dof_vel_scale.size() != static_cast<std::size_t>(kPolicyDof)) {
        return fail("dof_pos_scale and dof_vel_scale must have 12 values");
    }

    if (!finite_vector(config_.stand_pose_rad) ||
        !finite_vector(config_.action_scale) ||
        !finite_vector(config_.joint_min_rad) ||
        !finite_vector(config_.joint_max_rad) ||
        !finite_vector(config_.dof_pos_scale) ||
        !finite_vector(config_.dof_vel_scale)) {
        return fail("all vector policy values must be finite");
    }
    if (!finite_array3(config_.command_scale) ||
        !finite_array3(config_.body_ang_vel_scale)) {
        return fail("command/body_ang_vel scales must be finite");
    }
    if (!valid_motor_direction_values(config_.motor_to_model_direction)) {
        return fail("motor_to_model_direction values must be 1 or -1");
    }
#if POLICY_V3
    if (!finite_array3(config_.euler_scale)) {
        return fail("command/body_ang_vel/euler scales must be finite");
    }
#endif

    // 映射必须是模型 12 个 DOF 到 12 个电机逻辑索引的一一对应关系。
    std::array<bool, kPolicyDof> seen = {};
    for (int model_index = 0; model_index < kPolicyDof; ++model_index) {
        const int motor_index = config_.model_to_motor_index[model_index];
        if (motor_index < 0 || motor_index >= kPolicyDof) {
            return fail("model_to_motor_index contains an out-of-range motor index");
        }
        if (seen[motor_index]) {
            return fail("model_to_motor_index must be a permutation without duplicates");
        }
        seen[motor_index] = true;
    }

    for (int i = 0; i < kPolicyDof; ++i) {
        if (config_.action_scale[i] <= 0.0) {
            return fail("action_scale must be > 0 for every model DOF");
        }
        if (config_.joint_min_rad[i] > config_.joint_max_rad[i]) {
            return fail("joint_min_rad must be <= joint_max_rad for every model DOF");
        }
        if (config_.joint_min_rad[i] > 0.0 || config_.joint_max_rad[i] < 0.0) {
            return fail("relative joint limits must include 0 for every model DOF");
        }
    }

    return true;
}


void RobotInterface::unload_policy() {
    std::lock_guard<std::mutex> lock(policy_mutex_);
    policy_runner_.reset();
    last_action_raw_.fill(0.0F);
    observation_history_.fill(0.0F);
    observation_history_ready_ = false;
}


void RobotInterface::reset_policy_state() {
    std::lock_guard<std::mutex> lock(policy_mutex_);
    last_action_raw_.fill(0.0F);
    observation_history_.fill(0.0F);
    observation_history_ready_ = false;
    policy_start_time_ = std::chrono::steady_clock::now();
}


bool RobotInterface::policy_step(double vx, double vy, double yaw_rate) {
    std::lock_guard<std::mutex> lock(policy_mutex_);

    if (!policy_runner_ || !policy_runner_->is_loaded()) {
        return handle_policy_step_failure("policy is not loaded");
    }

    // 单次策略闭环：状态采样 -> 帧观测 -> 模型推理 -> 目标关节角 -> CSP 下发。
    std::array<float, kPolicyObservationSize> observation = {};
    if (!build_policy_observation(vx, vy, yaw_rate, observation)) {
        return handle_policy_step_failure("failed to build policy observation");
    }

    std::array<float, kPolicyDof> raw_action = {};
    if (!policy_runner_->infer(observation, raw_action)) {
        return handle_policy_step_failure(policy_runner_->last_error());
    }

#if POLICY_V3
    // last_action_raw_ 保存模型原始输出，下一周期作为 v3 单帧观测的 last_action 输入。
#else
    // last_action_raw_ 保存模型原始输出，下一周期进入 policy.pt 观测的 165-224 维。
#endif
    last_action_raw_ = raw_action;

    std::vector<double> target_q_model_rad(config_.num_motors, 0.0);
    for (int model_index = 0; model_index < kPolicyDof; ++model_index) {
        // 模型输出先按训练约定截断/缩放，再叠加模型顺序的站立姿态。
        const double clipped_action =
            std::max(-config_.action_clip,
                     std::min(config_.action_clip, static_cast<double>(raw_action[model_index])));

        target_q_model_rad[model_index] =
            config_.stand_pose_rad[model_index] + clipped_action * config_.action_scale[model_index];
    }

    if (!apply_action(target_q_model_rad)) {
        return handle_policy_step_failure("failed to apply policy target action");
    }

    return true;
}

/* 结合输入的目标速度构建完整模型输入 */
bool RobotInterface::build_policy_observation(
    double vx,
    double vy,
    double yaw_rate,
    std::array<float, kPolicyObservationSize>& observation) {

    if (!motors_initialized_.load() || !controller_) {
        std::cerr << "[RobotInterface] policy observation rejected: motors are not initialized\n";
        return false;
    }
    if (!imu_initialized_.load()) {
        std::cerr << "[RobotInterface] policy observation rejected: IMU is not initialized\n";
        return false;
    }
    if (!ahrs_ready_.load()) {
        std::cerr << "[RobotInterface] policy observation rejected: AHRS data is not ready\n";
        return false;
    }
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(yaw_rate)) {
        std::cerr << "[RobotInterface] policy observation rejected: command is not finite\n";
        return false;
    }

    const std::vector<double> q_rad    = get_joint_q();     // 拿到关节角度
    const std::vector<double> dq_rad_s = get_joint_vel();   // 拿到关节速度
    if (q_rad.size() != static_cast<std::size_t>(kPolicyDof) ||
        dq_rad_s.size() != static_cast<std::size_t>(kPolicyDof)) {
        std::cerr << "[RobotInterface] policy observation rejected: joint state size mismatch\n";
        return false;
    }

    std::array<double, 3> body_ang_vel = {};
    std::array<double, 3> euler = {};
    std::array<double, 4> quat = {};
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        body_ang_vel = body_ang_vel_;
        euler = euler_;
        quat = quat_;
    }

#if POLICY_V3
    if (!finite_vector(q_rad) || !finite_vector(dq_rad_s) ||
        !finite_array3(body_ang_vel) || !finite_array3(euler)) {
        std::cerr << "[RobotInterface] policy observation rejected: sensor value is not finite\n";
        return false;
    }

    const auto now = std::chrono::steady_clock::now();
    const double elapsed_s =
        std::chrono::duration<double>(now - policy_start_time_).count();
    double phase = std::fmod(elapsed_s * 0.1 / config_.policy_cycle_time_s, 1.0);
    if (phase < 0.0) {
        phase += 1.0;
    }
    // 单帧观测顺序必须和训练完全一致：phase、command、q、dq、last_action、IMU。
    std::array<float, kPolicySingleObservationSize> current_observation = {};
    current_observation[0] = static_cast<float>(std::sin(2.0 * kPi * phase));
    current_observation[1] = static_cast<float>(std::cos(2.0 * kPi * phase));

    current_observation[2] = static_cast<float>(vx * config_.command_scale[0]);
    current_observation[3] = static_cast<float>(vy * config_.command_scale[1]);
    current_observation[4] = static_cast<float>(yaw_rate * config_.command_scale[2]);

    for (int model_index = 0; model_index < kPolicyDof; ++model_index) {
        /* 电机关节映射转换 */
        const int motor_index = config_.model_to_motor_index[model_index];
        const double direction =
            static_cast<double>(direction_for_motor(config_, motor_index));
        const double q_model = direction * q_rad[motor_index];
        const double dq_model = direction * dq_rad_s[motor_index];
        current_observation[5 + model_index]  =
            static_cast<float>((q_model - config_.stand_pose_rad[model_index]) *
                               config_.dof_pos_scale[model_index]);
        current_observation[17 + model_index] =
            static_cast<float>(dq_model * config_.dof_vel_scale[model_index]);
        current_observation[29 + model_index] = last_action_raw_[model_index];
    }

    for (int i = 0; i < 3; ++i) {
        current_observation[41 + i] =
            static_cast<float>(body_ang_vel[i] * config_.body_ang_vel_scale[i]);
        current_observation[44 + i] =
            static_cast<float>(euler[i] * config_.euler_scale[i]);
    }

    /* 填充观测历史，首次填充用当前观测值来填满 */
    if (!observation_history_ready_) {
        for (int frame = 0; frame < kPolicyFrameStack; ++frame) {
            std::copy(current_observation.begin(),
                       current_observation.end(),
                     observation_history_.begin() + frame * kPolicySingleObservationSize);
        }
        observation_history_ready_ = true;
    } 
    /* 之后把当前观测帧添加到末尾 */
    else {
        std::copy(observation_history_.begin() + kPolicySingleObservationSize,
                   observation_history_.end(),
                 observation_history_.begin());
        std::copy(current_observation.begin(),
                   current_observation.end(),
                 observation_history_.end() - kPolicySingleObservationSize);
    }

    observation = observation_history_;
    return true;

#else
    if (!finite_vector(q_rad) || !finite_vector(dq_rad_s) ||
        !finite_array3(body_ang_vel) || !finite_array4(quat)) {
        std::cerr << "[RobotInterface] policy observation rejected: sensor value is not finite\n";
        return false;
    }

    std::array<double, 3> projected_gravity_double = {};
    if (!compute_projected_gravity(quat, projected_gravity_double)) {
        std::cerr << "[RobotInterface] policy observation rejected: quaternion is invalid\n";
        return false;
    }

    std::array<float, 3> base_ang_vel = {};
    std::array<float, 3> projected_gravity = {};
    std::array<float, 3> velocity_commands = {
        static_cast<float>(vx * config_.command_scale[0]),
        static_cast<float>(vy * config_.command_scale[1]),
        static_cast<float>(yaw_rate * config_.command_scale[2])
    };
    std::array<float, kPolicyDof> joint_pos_rel = {};
    std::array<float, kPolicyDof> joint_vel_rel = {};

    for (int i = 0; i < 3; ++i) {
        base_ang_vel[i] =
            static_cast<float>(body_ang_vel[i] * config_.body_ang_vel_scale[i]);
        projected_gravity[i] = static_cast<float>(projected_gravity_double[i]);
    }
    for (int model_index = 0; model_index < kPolicyDof; ++model_index) {
        /* 电机关节映射转换 */
        const int motor_index = config_.model_to_motor_index[model_index];
        /* 完成电机坐标系映射 */
        const double direction =
            static_cast<double>(direction_for_motor(config_, motor_index));
        const double q_model  = direction * q_rad[motor_index];
        const double dq_model = direction * dq_rad_s[motor_index];

        /* 转换得到基于模型索引的关节位置和速度 */
        joint_pos_rel[model_index] =
            static_cast<float>((q_model - config_.stand_pose_rad[model_index]) *
                               config_.dof_pos_scale[model_index]);
        joint_vel_rel[model_index] =
            static_cast<float>(dq_model * config_.dof_vel_scale[model_index]);
    }

    constexpr std::size_t kBaseAngVelOffset = 0;
    constexpr std::size_t kProjectedGravityOffset = 15;
    constexpr std::size_t kVelocityCommandsOffset = 30;
    constexpr std::size_t kJointPosRelOffset = 45;
    constexpr std::size_t kJointVelRelOffset = 105;
    constexpr std::size_t kLastActionOffset = 165;

    if (!observation_history_ready_) {
        fill_term_history(observation_history_, kBaseAngVelOffset,
                          kPolicyFrameStack, base_ang_vel);
        fill_term_history(observation_history_, kProjectedGravityOffset,
                          kPolicyFrameStack, projected_gravity);
        fill_term_history(observation_history_, kVelocityCommandsOffset,
                          kPolicyFrameStack, velocity_commands);
        fill_term_history(observation_history_, kJointPosRelOffset,
                          kPolicyFrameStack, joint_pos_rel);
        fill_term_history(observation_history_, kJointVelRelOffset,
                          kPolicyFrameStack, joint_vel_rel);
        fill_term_history(observation_history_, kLastActionOffset,
                          kPolicyFrameStack, last_action_raw_);
        observation_history_ready_ = true;
    } else {
        append_term_history(observation_history_, kBaseAngVelOffset,
                            kPolicyFrameStack, base_ang_vel);
        append_term_history(observation_history_, kProjectedGravityOffset,
                            kPolicyFrameStack, projected_gravity);
        append_term_history(observation_history_, kVelocityCommandsOffset,
                            kPolicyFrameStack, velocity_commands);
        append_term_history(observation_history_, kJointPosRelOffset,
                            kPolicyFrameStack, joint_pos_rel);
        append_term_history(observation_history_, kJointVelRelOffset,
                            kPolicyFrameStack, joint_vel_rel);
        append_term_history(observation_history_, kLastActionOffset,
                            kPolicyFrameStack, last_action_raw_);
    }

    observation = observation_history_;
    return true;

#endif
}


bool RobotInterface::handle_policy_step_failure(const std::string& message) {
    std::cerr << "[RobotInterface] policy_step failed: " << message << "\n";
    // 策略链路任何一步失败都停机，避免继续执行上一周期的目标。
    stop_motors(-1);
    return false;
}

}  // namespace inference
