#include "robot_interface.hpp"
#include "torch_policy_runner.hpp"

#include <algorithm>
#include <array>
#include <chrono>
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

bool finite_vector(const std::vector<double>& values)
{
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    });
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
        // 策略输入使用 AHRS 的角速度和欧拉角原始值；Yaw 不做启动零偏。
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


/*************************************************************************** */
/* 初始化并启动电机 */
bool RobotInterface::initial_and_start_motors() {
    if (motors_initialized_.load()) {
        return true;
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

    if (!wait_all_slaves_ready()) {
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
                                                               myactua::ControlMode::CSP));
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


/* 等待所有电机从站就绪 */
bool RobotInterface::wait_all_slaves_ready() const {
    if (!adapter_) {
        return false;
    }

    const auto start_time = std::chrono::steady_clock::now();
    const int timeout_ms = std::max(0, config_.wait_all_slaves_timeout_ms);
    const int poll_ms    = std::max(1, config_.wait_all_slaves_poll_ms);
    const auto deadline = start_time + std::chrono::milliseconds(timeout_ms);
    auto next_log_time = start_time;

    while (timeout_ms == 0 || std::chrono::steady_clock::now() < deadline) {
        adapter_->receivePhysical();
        adapter_->sendPhysical();

        int ready_count = 0;
        for (int i = 0; i < config_.num_motors; ++i) {
            if (adapter_->isConfigured(i)) {
                ++ready_count;
            }
        }
        if (ready_count == config_.num_motors) {
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time).count();
            std::cout << "[RobotInterface] All slaves ready in "
                      << elapsed_ms << " ms\n";
            return true;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= next_log_time) {
            std::cout << "[RobotInterface] EtherCAT ready: "
                      << ready_count << "/" << config_.num_motors << "\n";
            next_log_time = now + std::chrono::milliseconds(poll_ms);
        }

        if (timeout_ms == 0) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time).count();
    std::cout << "[RobotInterface] wait_all_slaves_ready timeout after "
              << elapsed_ms << " ms\n";
    return false;
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


bool RobotInterface::apply_action(const std::vector<double>& target_q_rad) {
    if (!motors_initialized_.load() || !controller_) {
        return false;
    }
    if (!motion_enabled_.load()) {
        std::cerr << "[RobotInterface] apply_action rejected: motors are stopped. "
                  << "Call restart_motors(-1) first.\n";
        return false;
    }
    if (static_cast<int>(target_q_rad.size()) != config_.num_motors) {
        std::cerr << "[RobotInterface] apply_action size mismatch. expected="
                  << config_.num_motors << " got=" << target_q_rad.size() << "\n";
        return false;
    }

    std::vector<double> target_deg(config_.num_motors, 0.0);
    for (int i = 0; i < config_.num_motors; ++i) {
        double q = target_q_rad[i];
        // if (config_.joint_min_rad.size() == static_cast<size_t>(config_.num_motors)) {
        //     q = std::max(q, config_.joint_min_rad[i]);
        // }
        // if (config_.joint_max_rad.size() == static_cast<size_t>(config_.num_motors)) {
        //     q = std::min(q, config_.joint_max_rad[i]);
        // }
        target_deg[i] = myactua::MYACTUA::rad_to_deg(q);
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

    std::vector<double> target(config_.num_motors, 0.0);
    if (static_cast<int>(config_.stand_pose_rad.size()) == config_.num_motors) {
        target = config_.stand_pose_rad;
    }

    const std::vector<double> q0 = get_joint_q();
    const int ramp_steps = 100;
    const auto dt = std::chrono::milliseconds(20);

    for (int k = 1; k <= ramp_steps; ++k) {
        const double alpha = static_cast<double>(k) / static_cast<double>(ramp_steps);
        std::vector<double> q_cmd(config_.num_motors, 0.0);
        for (int i = 0; i < config_.num_motors; ++i) {
            q_cmd[i] = q0[i] * (1.0 - alpha) + target[i] * alpha;
        }
        if (!apply_action(q_cmd)) {
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
    if (!std::isfinite(config_.policy_cycle_time_s) || config_.policy_cycle_time_s <= 0.0) {
        return fail("policy_cycle_time_s must be finite and > 0");
    }
    if (!std::isfinite(config_.action_clip) || config_.action_clip <= 0.0) {
        return fail("action_clip must be finite and > 0");
    }
    if (!std::isfinite(config_.action_scale) || config_.action_scale <= 0.0) {
        return fail("action_scale must be finite and > 0");
    }

    if (config_.stand_pose_rad.size() != static_cast<std::size_t>(kPolicyDof)) {
        return fail("stand_pose_rad must have 12 values");
    }
    if (config_.joint_min_rad.size() != static_cast<std::size_t>(kPolicyDof) ||
        config_.joint_max_rad.size() != static_cast<std::size_t>(kPolicyDof)) {
        return fail("joint_min_rad and joint_max_rad must have 12 values");
    }
    if (config_.model_to_motor_index.size() != static_cast<std::size_t>(kPolicyDof)) {
        return fail("model_to_motor_index must have 12 values");
    }
    if (config_.dof_pos_scale.size() != static_cast<std::size_t>(kPolicyDof) ||
        config_.dof_vel_scale.size() != static_cast<std::size_t>(kPolicyDof)) {
        return fail("dof_pos_scale and dof_vel_scale must have 12 values");
    }

    if (!finite_vector(config_.stand_pose_rad) ||
        !finite_vector(config_.joint_min_rad) ||
        !finite_vector(config_.joint_max_rad) ||
        !finite_vector(config_.dof_pos_scale) ||
        !finite_vector(config_.dof_vel_scale)) {
        return fail("all vector policy values must be finite");
    }
    if (!finite_array3(config_.command_scale) ||
        !finite_array3(config_.body_ang_vel_scale) ||
        !finite_array3(config_.euler_scale)) {
        return fail("command/body_ang_vel/euler scales must be finite");
    }

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
        if (config_.joint_min_rad[i] > config_.joint_max_rad[i]) {
            return fail("joint_min_rad must be <= joint_max_rad for every motor");
        }
        if (config_.stand_pose_rad[i] < config_.joint_min_rad[i] ||
            config_.stand_pose_rad[i] > config_.joint_max_rad[i]) {
            return fail("stand_pose_rad must be inside joint limits for every motor");
        }
    }

    return true;
}


void RobotInterface::unload_policy() {
    std::lock_guard<std::mutex> lock(policy_mutex_);
    policy_runner_.reset();
    last_action_raw_.fill(0.0F);
}


void RobotInterface::reset_policy_state() {
    std::lock_guard<std::mutex> lock(policy_mutex_);
    last_action_raw_.fill(0.0F);
    policy_start_time_ = std::chrono::steady_clock::now();
}


bool RobotInterface::policy_step(double vx, double vy, double yaw_rate) {
    std::lock_guard<std::mutex> lock(policy_mutex_);

    if (!policy_runner_ || !policy_runner_->is_loaded()) {
        return handle_policy_step_failure("policy is not loaded");
    }

    // 单次策略闭环：状态采样 -> 47 维观测 -> 模型推理 -> 目标关节角 -> CSP 下发。
    std::array<float, kPolicyObservationSize> observation = {};
    if (!build_policy_observation(vx, vy, yaw_rate, observation)) {
        return handle_policy_step_failure("failed to build policy observation");
    }

    std::array<float, kPolicyDof> raw_action = {};
    if (!policy_runner_->infer(observation, raw_action)) {
        return handle_policy_step_failure(policy_runner_->last_error());
    }

    // last_action_raw_ 保存模型原始输出，下一周期作为观测的 29-40 维输入。
    last_action_raw_ = raw_action;

    std::vector<double> target_q_rad(config_.num_motors, 0.0);
    for (int model_index = 0; model_index < kPolicyDof; ++model_index) {
        const int motor_index = config_.model_to_motor_index[model_index];
        // 模型输出先按训练约定截断/缩放，再叠加站立姿态并进入硬限位。
        const double clipped_action =
            std::max(-config_.action_clip,
                     std::min(config_.action_clip,
                              static_cast<double>(raw_action[model_index])));
        const double target =
            config_.stand_pose_rad[motor_index] + clipped_action * config_.action_scale;
        target_q_rad[motor_index] =
            std::max(config_.joint_min_rad[motor_index],
                     std::min(config_.joint_max_rad[motor_index], target));
    }

    if (!apply_action(target_q_rad)) {
        return handle_policy_step_failure("failed to apply policy target action");
    }

    return true;
}

/* 结合输入的目标速度构建完整模型输入 */
bool RobotInterface::build_policy_observation(
    double vx,
    double vy,
    double yaw_rate,
    std::array<float, kPolicyObservationSize>& observation) const {

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
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        body_ang_vel = body_ang_vel_;
        euler = euler_;
    }

    if (!finite_vector(q_rad) || !finite_vector(dq_rad_s) ||
        !finite_array3(body_ang_vel) || !finite_array3(euler)) {
        std::cerr << "[RobotInterface] policy observation rejected: sensor value is not finite\n";
        return false;
    }

    const auto now = std::chrono::steady_clock::now();
    const double elapsed_s =
        std::chrono::duration<double>(now - policy_start_time_).count();
    // 相位只由策略启动后的时间和训练周期决定，输入为 sin/cos 避免 0/1 跳变。
    double phase = std::fmod(elapsed_s / config_.policy_cycle_time_s, 1.0);
    if (phase < 0.0) {
        phase += 1.0;
    }

    // 观测顺序必须和训练完全一致：phase、command、q、dq、last_action、IMU。
    observation.fill(0.0F);
    observation[0] = static_cast<float>(std::sin(2.0 * kPi * phase));
    observation[1] = static_cast<float>(std::cos(2.0 * kPi * phase));

    observation[2] = static_cast<float>(vx * config_.command_scale[0]);
    observation[3] = static_cast<float>(vy * config_.command_scale[1]);
    observation[4] = static_cast<float>(yaw_rate * config_.command_scale[2]);

    for (int model_index = 0; model_index < kPolicyDof; ++model_index) {
        const int motor_index = config_.model_to_motor_index[model_index];
        // 按模型关节顺序取电机状态；位置使用相对站立姿态，不使用绝对角。
        observation[5 + model_index]  =
            static_cast<float>((q_rad[motor_index] - config_.stand_pose_rad[motor_index]) *
                               config_.dof_pos_scale[model_index]);
        observation[17 + model_index] =
            static_cast<float>(dq_rad_s[motor_index] *
                               config_.dof_vel_scale[model_index]);
        observation[29 + model_index] = last_action_raw_[model_index];
    }

    for (int i = 0; i < 3; ++i) {
        observation[41 + i] =
            static_cast<float>(body_ang_vel[i] * config_.body_ang_vel_scale[i]);
        observation[44 + i] =
            static_cast<float>(euler[i] * config_.euler_scale[i]);
    }

    return true;
}


bool RobotInterface::handle_policy_step_failure(const std::string& message) {
    std::cerr << "[RobotInterface] policy_step failed: " << message << "\n";
    // 策略链路任何一步失败都停机，避免继续执行上一周期的目标。
    stop_motors(-1);
    return false;
}

}  // namespace inference
