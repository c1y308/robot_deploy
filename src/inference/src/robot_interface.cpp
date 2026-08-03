#include "robot_interface.hpp"
#include "torch_policy_runner.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>


namespace inference {
namespace {

/* policy_step 推理输出和 apply_action 映射后的电机目标写到这里。 */
const std::filesystem::path& policy_output_log_dir()
{
    static const std::filesystem::path path =
        std::filesystem::path(__FILE__).parent_path().parent_path() /
        "log" / "policy_output";
    return path;
}

/* apply_action after each target command writes target tracking error here. */
const std::filesystem::path& motor_pos_error_log_dir()
{
    static const std::filesystem::path path =
        std::filesystem::path(__FILE__).parent_path().parent_path() /
        "log" / "motor_pos_error";
    return path;
}

void append_indexed_columns(std::ostream& stream,
                            const char* prefix,
                            int count)
{
    for (int i = 0; i < count; ++i) {
        stream << ',' << prefix << '_' << i;
    }
}

void write_policy_output_csv_header(std::ostream& stream)
{
    stream << "frame_index,elapsed_us";
    append_indexed_columns(stream, "raw_action", 12);
    append_indexed_columns(stream, "target_q_model_rad", 12);
    append_indexed_columns(stream, "target_rad", 12);
    stream << '\n';
}

void write_motor_pos_error_csv_header(std::ostream& stream, int motor_count)
{
    stream << "frame_index,elapsed_us";
    append_indexed_columns(stream, "target_pos_deg", motor_count);
    append_indexed_columns(stream, "rx_pos_deg", motor_count);
    append_indexed_columns(stream, "tar_err_deg", motor_count);
    stream << '\n';
}

/* 检查 3 维数组中的数值是否全部为有限值。 */
bool finite_array3(const std::array<double, 3>& values)
{
    return std::isfinite(values[0]) &&
           std::isfinite(values[1]) &&
           std::isfinite(values[2]);
}

/* 检查动态数组中的数值是否全部为有限值。 */
bool finite_vector(const std::vector<double>& values)
{
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    });
}

/* 检查 action 截断范围中的上下界是否全部为有限值。 */
bool finite_action_clip_ranges(const std::vector<std::array<double, 2>>& ranges)
{
    return std::all_of(ranges.begin(), ranges.end(), [](const auto& range) {
        return std::isfinite(range[0]) && std::isfinite(range[1]);
    });
}

/* 检查电机方向配置是否只包含 1 或 -1。 */
bool valid_motor_direction_values(const std::vector<int>& directions)
{
    return std::all_of(directions.begin(), directions.end(), [](int direction) {
        return direction == 1 || direction == -1;
    });
}

/* 获取指定电机从物理坐标到模型坐标的方向符号。 */
int direction_for_motor(const RobotInterfaceConfig& config, int motor_index)
{
    if (config.motor_to_model_direction.empty()) {
        return 1;
    }
    return config.motor_to_model_direction[motor_index];
}

bool index_in_range(int index, int count)
{
    return index >= 0 && index < count;
}

bool ankle_parallel_map_indices_in_range(
    const RobotInterfaceConfig::AnkleParallelMap& ankle_map,
    int count)
{
    return index_in_range(ankle_map.model_pitch_dof, count) &&
           index_in_range(ankle_map.model_roll_dof, count) &&
           index_in_range(ankle_map.upper_motor_index, count) &&
           index_in_range(ankle_map.lower_motor_index, count);
}

std::array<float, 2> gait_phase_observation(std::uint64_t episode_length,
                                            double step_dt,
                                            double period)
{
    constexpr double kTwoPi = 6.28318530717958647692;
    const double global_phase =
        std::fmod(static_cast<double>(episode_length) * step_dt, period) / period;
    return {
        static_cast<float>(std::sin(global_phase * kTwoPi)),
        static_cast<float>(std::cos(global_phase * kTwoPi))
    };
}


/* 判断当前电机控制模式是否走 MIT/PVT setpoint 下发路径。 */
bool is_mit_mode(myactua::ControlMode mode)
{
    return mode == myactua::ControlMode::PVT;
}

}  // namespace

/* 保存外部传入的接口配置，后续由初始化函数按模块使用。 */
RobotInterface::RobotInterface(RobotInterfaceConfig config)
    : config_(std::move(config)) {}

/* 析构时依次释放策略、IMU 和电机资源，保证后台线程退出。 */
RobotInterface::~RobotInterface() {
    unload_policy();
    deinit_imu();
    deinit_motors();
}

/* 保存策略使用的机器人目标速度指令，顺序为 [vx, vy, yaw_rate]。 */
void RobotInterface::set_target_velocity(double vx, double vy, double yaw_rate) {
    std::lock_guard<std::mutex> lock(target_velocity_mutex_);
    target_velocity_ = {vx, vy, yaw_rate};
}

/* 获取当前保存的机器人目标速度指令。 */
std::array<double, 3> RobotInterface::get_target_velocity() const {
    std::lock_guard<std::mutex> lock(target_velocity_mutex_);
    return target_velocity_;
}



/********************************************************************* */
/* 初始化并启动 IMU 读取线程，注册 AHRS 回调以缓存姿态与角速度。 */
bool RobotInterface::initial_and_start_imu() {
    if (imu_initialized_.load()) {
        return true;
    }

    imu::Config_t imu_cfg;
    imu_cfg.device      = config_.imu_device;
    imu_cfg.baudrate    = config_.imu_baudrate;
    imu_cfg.print_imu   = config_.imu_print_imu;
    imu_cfg.print_ahrs  = config_.imu_print_ahrs;

    ahrs_ready_.store(false);
    projected_gravity_valid_ = false;
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

        projected_gravity_[0] = static_cast<double>(data.projected_gravity_x);
        projected_gravity_[1] = static_cast<double>(data.projected_gravity_y);
        projected_gravity_[2] = static_cast<double>(data.projected_gravity_z);
        projected_gravity_valid_ = data.projected_gravity_valid;
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

/* 停止 IMU 读取线程并清空 IMU/AHRS 就绪状态。 */
void RobotInterface::deinit_imu() {
    if (imu_reader_) {
        imu_reader_->stop();
    }
    imu_reader_.reset();
    imu_initialized_.store(false);
    ahrs_ready_.store(false);
    projected_gravity_valid_ = false;
}

/* 获取 AHRS 输出的四元数，顺序为 [w, x, y, z]。 */
std::array<double, 4> RobotInterface::get_quat() const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return quat_;
}

/* 获取 IMU AHRS 输出的角速度。 */
std::array<double, 3> RobotInterface::get_ang_vel() const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return ang_vel_;
}

/* 获取机身坐标系角速度，供策略观测使用。 */
std::array<double, 3> RobotInterface::get_body_ang_vel() const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return body_ang_vel_;
}

/* 获取 IMU AHRS 输出的欧拉角。 */
std::array<double, 3> RobotInterface::get_euler() const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return euler_;
}

/* 获取投影重力向量；数据无效时返回零向量。 */
std::array<double, 3> RobotInterface::get_projected_gravity() const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (!projected_gravity_valid_) {
        return {0.0, 0.0, 0.0};
    }
    return projected_gravity_;
}


/*************************************************************************** */
/* 初始化 EtherCAT 电机控制器，配置控制模式并启动实时线程。 */
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
        controller_->send_command(
            myactua::ControlCommand::SetMode(config_.motor_control_mode, i));
    }

    /* 根据配置决定是否打印电机信息 */
    if (config_.print_motors_info) {
        controller_->set_print_info(config_.print_motor_ids);
    } else {
        controller_->set_print_info({});
    }

    /* 启动实时线程 */
    controller_->start();

    controller_->send_command(myactua::ControlCommand::Stop());
    motors_initialized_.store(true);
    motion_enabled_.store(false);
    reset_motor_pos_error_log();
    start_motor_pos_error_log();

    return true;
}

/* 校验电机数量、方向和 MIT/PVT 控制参数是否满足下发要求。 */
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

/* 停止电机控制器并释放 EtherCAT 相关资源。 */
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
    reset_motor_pos_error_log();
}

/* 发送 STOP 指令；slave_index 为 -1 时停止全部电机并关闭运动使能。 */
bool RobotInterface::stop_motors(int slave_index) {
    if (!motors_initialized_.load() || !controller_) {
        return false;
    }
    if (slave_index >= config_.num_motors) {
        std::cerr << "[RobotInterface] stop_motors invalid slave_index="
                  << slave_index << "\n";
        return false;
    }

    controller_->send_command(myactua::ControlCommand::Stop(slave_index));
    if (slave_index < 0) {
        motion_enabled_.store(false);
    }
    return true;
}

/* 发送 RESTART 指令；全部电机重启成功后允许后续动作下发。 */
bool RobotInterface::restart_motors(int slave_index) {
    if (!motors_initialized_.load() || !controller_) {
        return false;
    }
    if (slave_index >= config_.num_motors) {
        std::cerr << "[RobotInterface] restart_motors invalid slave_index="
                  << slave_index << "\n";
        return false;
    }

    controller_->send_command(myactua::ControlCommand::Restart(slave_index));
    if (slave_index < 0 || config_.num_motors == 1) {
        motion_enabled_.store(true);
    }
    return true;
}

/* 判断指定模型 DOF 是否为显式配置的脚踝并联机构 DOF。 */
bool RobotInterface::is_parallel_ankle_model_dof(int model_index) const {
    return model_index == config_.left_ankle_parallel.model_pitch_dof ||
           model_index == config_.left_ankle_parallel.model_roll_dof ||
           model_index == config_.right_ankle_parallel.model_pitch_dof ||
           model_index == config_.right_ankle_parallel.model_roll_dof;
}

/* 返回普通单关节 DOF 在 compact model_to_motor_index 中的位置。 */
int RobotInterface::direct_model_mapping_slot(int model_index) const {
    int slot = 0;
    for (int i = 0; i < model_index; ++i) {
        if (!is_parallel_ankle_model_dof(i)) {
            ++slot;
        }
    }
    return slot;
}

/* 对单条腿的脚踝并联机构执行逆解，将 roll/pitch 转为两条驱动链电机角度。 */
bool RobotInterface::apply_ankle_ik(
    const std::vector<double>& target_q_model_rad,  // 模型 DOF 顺序输出的目标关节角(弧度)
    std::vector<double>& target_rad,                // 电机物理顺序的目标关节角(弧度)
    const RobotInterfaceConfig::AnkleParallelMap& ankle_map,
    ankle_motor_ik::Solver& solver,                 // IK 求解器
    double& last_upper_motor, double& last_lower_motor,
    bool& solved) const {                           // 上一次求解是否成功，失败时回退到上一次成功值

    if (!ankle_parallel_map_indices_in_range(ankle_map, config_.num_motors)) {
        std::cerr << "[RobotInterface] apply_action rejected: "
                  << "ankle parallel map contains an out-of-range index\n";
        return false;
    }

    const bool has_relative_limits =
        config_.stand_pose_rad.size() == static_cast<size_t>(config_.num_motors) &&
        config_.joint_min_rad.size()  == static_cast<size_t>(config_.num_motors) &&
        config_.joint_max_rad.size()  == static_cast<size_t>(config_.num_motors);

    /* 对 pitch 限位（相对 stand_pose_rad） */
    const int ankle_model_pitch_dof = ankle_map.model_pitch_dof;
    const int ankle_model_roll_dof = ankle_map.model_roll_dof;
    double pitch = target_q_model_rad[ankle_model_pitch_dof];
    if (has_relative_limits) {
        const double lo = config_.stand_pose_rad[ankle_model_pitch_dof] +
                          config_.joint_min_rad[ankle_model_pitch_dof];
        const double hi = config_.stand_pose_rad[ankle_model_pitch_dof] +
                          config_.joint_max_rad[ankle_model_pitch_dof];
        pitch = std::max(lo, std::min(hi, pitch));
    }

    /* 对 roll 限位（相对 stand_pose_rad） */
    double roll = target_q_model_rad[ankle_model_roll_dof];
    if (has_relative_limits) {
        const double lo = config_.stand_pose_rad[ankle_model_roll_dof] +
                          config_.joint_min_rad[ankle_model_roll_dof];
        const double hi = config_.stand_pose_rad[ankle_model_roll_dof] +
                          config_.joint_max_rad[ankle_model_roll_dof];
        roll = std::max(lo, std::min(hi, roll));
    }

    /* IK 求解：输入 (roll, pitch)，输出两条几何链 motor1/motor2 */
    ankle_motor_ik::MotorAngles result = solver.solve(roll, pitch);

    double upper_motor = 0.0;
    double lower_motor = 0.0;
    if (result.reachable()) {
        upper_motor = result.motor1;
        lower_motor = result.motor2;
        last_upper_motor = upper_motor;
        last_lower_motor = lower_motor;
        solved = true;
    } else if (solved) {
        /* 不可达时保持上一次成功值 */
        upper_motor = last_upper_motor;
        lower_motor = last_lower_motor;
    } else {
        /* 从未成功过，回零 */
        upper_motor = 0.0;
        lower_motor = 0.0;
    }

    /* 应用方向映射后写入电机目标 */
    target_rad[ankle_map.upper_motor_index] =
        static_cast<double>(direction_for_motor(config_, ankle_map.upper_motor_index)) *
        upper_motor;
    target_rad[ankle_map.lower_motor_index] =
        static_cast<double>(direction_for_motor(config_, ankle_map.lower_motor_index)) *
        lower_motor;
    return true;
}


/* 将模型 DOF 顺序目标关节位置（弧度）转换为电机顺序目标位置（弧度）。
    args:
        target_q_model_rad: 模型 DOF 顺序的目标关节角，单位为弧度
        target_rad: 输出电机顺序的目标关节角，单位为弧度
*/
bool RobotInterface::build_action_target_rad(   const std::vector<double>& target_q_model_rad,
                                                std::vector<double>& target_rad) const {

    if (static_cast<int>(target_q_model_rad.size()) != config_.num_motors) {
        std::cerr << "[RobotInterface] apply_action size mismatch. expected="
                  << config_.num_motors << " got=" << target_q_model_rad.size() << "\n";
        return false;
    }

    target_rad.assign(config_.num_motors, 0.0);

    /* 检查是否具有相对限位 */
    const bool has_relative_limits =
        config_.stand_pose_rad.size() == static_cast<size_t>(config_.num_motors) &&
        config_.joint_min_rad.size()  == static_cast<size_t>(config_.num_motors) &&
        config_.joint_max_rad.size()  == static_cast<size_t>(config_.num_motors);

    /* 按照模型 dof 顺序遍历电机 */
    for (int model_index = 0; model_index < config_.num_motors; ++model_index) {
        /* 脚踝 DOF 由 apply_ankle_ik 处理，跳过直接映射。 */
        if (is_parallel_ankle_model_dof(model_index)) {
            continue;
        }

        /* 开始从模型 dof 顺序转换得到物理顺序 */
        const int mapping_slot = direct_model_mapping_slot(model_index);
        if (mapping_slot < 0 ||
            mapping_slot >= static_cast<int>(config_.model_to_motor_index.size())) {
            std::cerr << "[RobotInterface] apply_action rejected: "
                      << "model_to_motor_index missing direct mapping for model index "
                      << model_index << "\n";
            return false;
        }
        const int motor_index = config_.model_to_motor_index[mapping_slot];
        if (motor_index < 0 || motor_index >= config_.num_motors) {
            std::cerr << "[RobotInterface] apply_action rejected: "
                      << "model_to_motor_index contains invalid motor index "
                      << motor_index << "\n";
            return false;
        }

        /* 将模型输出的关节角度限制在相对 stand_pose_rad 的 joint_min/joint_max 范围内 */
        double q = target_q_model_rad[model_index];
        if (has_relative_limits) {
            const double lower_limit = config_.stand_pose_rad[model_index] + config_.joint_min_rad[model_index];
            const double upper_limit = config_.stand_pose_rad[model_index] + config_.joint_max_rad[model_index];
            q = std::max(lower_limit, std::min(upper_limit, q));
        }
        /* 将模型输出的电机目标值转换为电机目标值：1.完成方向映射 2.将模型输出值映射到物理电机ID  */
        target_rad[motor_index] = static_cast<double>(direction_for_motor(config_, motor_index)) * q;
    }

    /* 脚踝并联机构逆解：将 roll/pitch 转为两条驱动链电机角度（弧度） */
    if (!apply_ankle_ik(target_q_model_rad, target_rad,
                        config_.left_ankle_parallel,
                        left_ankle_solver_,
                        left_ankle_last_upper_motor_,
                        left_ankle_last_lower_motor_,
                        left_ankle_solved_)) {
        return false;
    }
    if (!apply_ankle_ik(target_q_model_rad, target_rad,
                        config_.right_ankle_parallel,
                        right_ankle_solver_,
                        right_ankle_last_upper_motor_,
                        right_ankle_last_lower_motor_,
                        right_ankle_solved_)) {
        return false;
    }

    return true;
}

/* 下发模型 DOF 顺序的目标关节角，内部完成限位、映射和方向转换。 */
bool RobotInterface::apply_action(const std::vector<double>& target_q_model_rad) {
    if (!motors_initialized_.load() || !controller_) {
        return false;
    }
    if (!motion_enabled_.load()) {
        std::cerr << "[RobotInterface] apply_action rejected: motors are stopped. "
                  << "Call restart_motors(-1) first.\n";
        return false;
    }

    std::vector<double> target_rad;
    if (!build_action_target_rad(target_q_model_rad, target_rad)) {
        return false;
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
        controller_->send_command(myactua::ControlCommand::SetMitSetpoints(std::move(mit_setpoints)));
        log_motor_pos_error(target_rad);
        return true;
    }

    /* 位置控制模式 */
    if (config_.motor_control_mode != myactua::ControlMode::CSP) {
        std::cerr << "[RobotInterface] apply_action supports only MIT/PVT or CSP mode\n";
        return false;
    }

    std::vector<double> target_deg(config_.num_motors, 0.0);
    for (int i = 0; i < config_.num_motors; ++i) {
        target_deg[i] = myactua::MYACTUA::rad_to_deg(target_rad[i]);
    }

    controller_->send_command(myactua::ControlCommand::SetScalarSetpoints(std::move(target_deg)));
    log_motor_pos_error(target_rad);
    return true;
}

/* 获取当前关节角，单位 rad；控制器未创建时返回零向量。 */
std::vector<double> RobotInterface::get_joint_q() const {
    std::vector<double> q(config_.num_motors, 0.0);
    if (!controller_) {
        return q;
    }
    q = controller_->get_joint_q_rad();
    return q;
}

/* 获取当前关节速度，单位 rad/s；控制器未创建时返回零向量。 */
std::vector<double> RobotInterface::get_joint_vel() const {
    std::vector<double> dq(config_.num_motors, 0.0);
    if (!controller_) {
        return dq;
    }
    dq = controller_->get_joint_vel_rad_s();
    return dq;
}

/* 获取当前关节力矩原始值；控制器未创建时返回零向量。 */
std::vector<double> RobotInterface::get_joint_tau() const {
    std::vector<double> tau(config_.num_motors, 0.0);
    if (!controller_) {
        return tau;
    }
    tau = controller_->get_joint_tau_raw();
    return tau;
}

/* 按平滑插值将关节恢复到 stand_pose_rad 初始姿态。 */
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

    for (int model_index = 0; model_index < config_.num_motors; ++model_index) {
        if (is_parallel_ankle_model_dof(model_index)) {
            continue;
        }

        const int mapping_slot = direct_model_mapping_slot(model_index);
        if (mapping_slot < 0 ||
            mapping_slot >= static_cast<int>(config_.model_to_motor_index.size())) {
            std::cerr << "[RobotInterface] reset_joints rejected: "
                      << "model_to_motor_index missing direct mapping for model index "
                      << model_index << "\n";
            return false;
        }
        const int motor_index = config_.model_to_motor_index[mapping_slot];
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

    auto fill_ankle_q0_model = [&](const RobotInterfaceConfig::AnkleParallelMap& ankle_map) {
        if (!ankle_parallel_map_indices_in_range(ankle_map, config_.num_motors)) {
            std::cerr << "[RobotInterface] reset_joints rejected: "
                      << "ankle parallel map contains an out-of-range index\n";
            return false;
        }

        const double upper_motor =
            static_cast<double>(direction_for_motor(config_, ankle_map.upper_motor_index)) *
            q0_motor[ankle_map.upper_motor_index];
        const double lower_motor =
            static_cast<double>(direction_for_motor(config_, ankle_map.lower_motor_index)) *
            q0_motor[ankle_map.lower_motor_index];

        const double initial_pitch = target_model[ankle_map.model_pitch_dof];
        const double initial_roll = target_model[ankle_map.model_roll_dof];
        const ankle_motor_fk::FootAngles foot =
            ankle_motor_fk::solve(upper_motor, lower_motor, initial_roll, initial_pitch);

        if (foot.reachable) {
            q0_model[ankle_map.model_pitch_dof] = foot.pitch;
            q0_model[ankle_map.model_roll_dof] = foot.roll;
        } else {
            q0_model[ankle_map.model_pitch_dof] = target_model[ankle_map.model_pitch_dof];
            q0_model[ankle_map.model_roll_dof] = target_model[ankle_map.model_roll_dof];
        }
        return true;
    };

    if (!fill_ankle_q0_model(config_.left_ankle_parallel) ||
        !fill_ankle_q0_model(config_.right_ankle_parallel)) {
        return false;
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
/* 加载 TorchScript 策略模型，并重置上一周期动作和观测历史。 */
bool RobotInterface::load_policy() {
    std::lock_guard<std::mutex> lock(policy_mutex_);
    // 加载前先清掉旧 runner 和上一周期动作，避免失败后残留旧策略状态。
    policy_runner_.reset();
    last_action_raw_.fill(0.0F);
    observation_history_.fill(0.0F);
    observation_history_ready_ = false;
    policy_episode_length_ = 0;
    policy_start_time_ = std::chrono::steady_clock::now();
    reset_motor_pos_error_log();

    if (!validate_policy_config()) {
        return false;
    }
    reset_ankle_fk_state();

    auto runner = std::make_unique<TorchPolicyRunner>();
    if (!runner->load(config_.policy_model_path)) {
        std::cerr << "[RobotInterface] load_policy failed: "
                  << runner->last_error() << "\n";
        return false;
    }

    policy_runner_ = std::move(runner);
    start_policy_output_log();
    start_motor_pos_error_log();
    return true;
}

/* 检查策略模型路径、映射、缩放、限位和观测参数是否有效。 */
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
    if (config_.action_clip.size() != static_cast<std::size_t>(kPolicyDof)) {
        return fail("action_clip must have 12 ranges");
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
    if (!config_.motor_to_model_direction.empty() &&
        config_.motor_to_model_direction.size() != static_cast<std::size_t>(kPolicyDof)) {
        return fail("motor_to_model_direction must have 12 values or be empty");
    }
    if (config_.dof_pos_scale.size() != static_cast<std::size_t>(kPolicyDof) ||
        config_.dof_vel_scale.size() != static_cast<std::size_t>(kPolicyDof)) {
        return fail("dof_pos_scale and dof_vel_scale must have 12 values");
    }

    if (!finite_vector(config_.stand_pose_rad) ||
        !finite_action_clip_ranges(config_.action_clip) ||
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
    if (!std::isfinite(config_.policy_step_dt) ||
        config_.policy_step_dt <= 0.0) {
        return fail("policy_step_dt must be a finite positive value");
    }
    if constexpr (policy_observation::kEnableGaitPhase) {
        if (!std::isfinite(config_.gait_phase_period) ||
            config_.gait_phase_period <= 0.0) {
            return fail("gait_phase_period must be a finite positive value");
        }
    }
    if (!valid_motor_direction_values(config_.motor_to_model_direction)) {
        return fail("motor_to_model_direction values must be 1 or -1");
    }
    if (!ankle_parallel_map_indices_in_range(config_.left_ankle_parallel, kPolicyDof) ||
        !ankle_parallel_map_indices_in_range(config_.right_ankle_parallel, kPolicyDof)) {
        return fail("ankle parallel maps contain an out-of-range index");
    }

    std::array<bool, kPolicyDof> seen_ankle_model_dof = {};
    std::array<bool, kPolicyDof> seen_motor = {};

    auto mark_model_dof = [&](int model_dof) {
        if (seen_ankle_model_dof[model_dof]) {
            return false;
        }
        seen_ankle_model_dof[model_dof] = true;
        return true;
    };

    auto mark_motor = [&](int motor_index) {
        if (seen_motor[motor_index]) {
            return false;
        }
        seen_motor[motor_index] = true;
        return true;
    };

    const auto& left = config_.left_ankle_parallel;
    const auto& right = config_.right_ankle_parallel;
    if (!mark_model_dof(left.model_pitch_dof) ||
        !mark_model_dof(left.model_roll_dof) ||
        !mark_model_dof(right.model_pitch_dof) ||
        !mark_model_dof(right.model_roll_dof)) {
        return fail("ankle parallel model DOFs must be distinct");
    }

    if (!mark_motor(left.upper_motor_index) ||
        !mark_motor(left.lower_motor_index) ||
        !mark_motor(right.upper_motor_index) ||
        !mark_motor(right.lower_motor_index)) {
        return fail("ankle parallel motor indices must be distinct");
    }

    const int direct_model_dof_count = static_cast<int>(
        std::count(seen_ankle_model_dof.begin(), seen_ankle_model_dof.end(), false));
    if (config_.model_to_motor_index.size() !=
        static_cast<std::size_t>(direct_model_dof_count)) {
        std::ostringstream message;
        message << "model_to_motor_index must have "
                << direct_model_dof_count
                << " direct-DOF values";
        return fail(message.str());
    }

    for (int mapping_slot = 0; mapping_slot < direct_model_dof_count; ++mapping_slot) {
        const int motor_index = config_.model_to_motor_index[mapping_slot];
        if (motor_index < 0 || motor_index >= kPolicyDof) {
            return fail("model_to_motor_index contains an out-of-range motor index");
        }
        if (!mark_motor(motor_index)) {
            return fail("direct and ankle motor mappings must be a permutation without duplicates");
        }
    }

    if (std::any_of(seen_motor.begin(), seen_motor.end(), [](bool is_seen) {
            return !is_seen;
        })) {
        return fail("direct and ankle motor mappings must cover all motors");
    }

    for (int i = 0; i < kPolicyDof; ++i) {
        if (config_.action_clip[i][0] > config_.action_clip[i][1]) {
            return fail("action_clip lower bound must be <= upper bound for every model DOF");
        }
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

/* 卸载策略模型并清空策略运行状态。 */
void RobotInterface::unload_policy() {
    std::lock_guard<std::mutex> lock(policy_mutex_);
    policy_runner_.reset();
    last_action_raw_.fill(0.0F);
    observation_history_.fill(0.0F);
    observation_history_ready_ = false;
    policy_episode_length_ = 0;
    reset_policy_output_log();
    reset_motor_pos_error_log();

    /* 重置脚踝 IK 求解器状态 */
    left_ankle_solver_.reset();
    right_ankle_solver_.reset();
    left_ankle_solved_ = false;
    right_ankle_solved_ = false;
    left_ankle_last_upper_motor_ = 0.0;
    left_ankle_last_lower_motor_ = 0.0;
    right_ankle_last_upper_motor_ = 0.0;
    right_ankle_last_lower_motor_ = 0.0;

    reset_ankle_fk_state();
}

/* 保留已加载策略，仅重置动作历史、观测历史和相位起点。 */
void RobotInterface::reset_policy_state() {
    std::lock_guard<std::mutex> lock(policy_mutex_);
    last_action_raw_.fill(0.0F);
    observation_history_.fill(0.0F);
    observation_history_ready_ = false;
    policy_episode_length_ = 0;
    policy_start_time_ = std::chrono::steady_clock::now();
    reset_policy_output_log();
    reset_motor_pos_error_log();

    /* 重置脚踝 IK 求解器状态 */
    left_ankle_solver_.reset();
    right_ankle_solver_.reset();
    left_ankle_solved_ = false;
    right_ankle_solved_ = false;
    left_ankle_last_upper_motor_ = 0.0;
    left_ankle_last_lower_motor_ = 0.0;
    right_ankle_last_upper_motor_ = 0.0;
    right_ankle_last_lower_motor_ = 0.0;

    reset_ankle_fk_state();
    start_policy_output_log();
    start_motor_pos_error_log();
}

void RobotInterface::reset_ankle_fk_state() {
    auto reset_solver = [&](const RobotInterfaceConfig::AnkleParallelMap& ankle_map,
                            ankle_motor_fk::Solver& solver) {
        double pitch = 0.0;
        double roll = 0.0;
        if (config_.stand_pose_rad.size() == static_cast<std::size_t>(kPolicyDof) &&
            index_in_range(ankle_map.model_pitch_dof, kPolicyDof) &&
            index_in_range(ankle_map.model_roll_dof, kPolicyDof)) {
            pitch = config_.stand_pose_rad[ankle_map.model_pitch_dof];
            roll = config_.stand_pose_rad[ankle_map.model_roll_dof];
        }
        solver.reset(roll, pitch);
    };

    reset_solver(config_.left_ankle_parallel, left_ankle_fk_solver_);
    reset_solver(config_.right_ankle_parallel, right_ankle_fk_solver_);
    ankle_fk_observation_ready_ = false;
    ankle_fk_last_time_ = {};
}

/* Stop the current async policy output log and reset its frame counter. */
void RobotInterface::reset_policy_output_log() {
    policy_output_logger_.stop();
    policy_output_frame_index_ = 0;
    policy_output_log_failed_ = false;
}

bool RobotInterface::start_policy_output_log() {
    if (!config_.enable_policy_output_log || policy_output_log_failed_) {
        return false;
    }
    if (policy_output_logger_.is_running()) {
        return true;
    }

    std::ostringstream header;
    write_policy_output_csv_header(header);

    AsyncCsvLogger::Config logger_config;
    logger_config.directory = policy_output_log_dir();
    logger_config.file_prefix = "policy_output";
    logger_config.header = header.str();
    logger_config.flush_interval = std::chrono::milliseconds(
        std::max(1, config_.async_log_flush_interval_ms));
    logger_config.max_queue_depth = config_.async_log_queue_depth;

    if (!policy_output_logger_.start(std::move(logger_config))) {
        std::cerr << "[RobotInterface] failed to open policy output log: "
                  << policy_output_logger_.last_error() << "\n";
        policy_output_log_failed_ = true;
        return false;
    }

    std::cout << "[RobotInterface] policy output log: "
              << policy_output_logger_.log_path() << "\n";
    return true;
}

bool RobotInterface::ensure_policy_output_log() {
    if (!config_.enable_policy_output_log || policy_output_log_failed_) {
        return false;
    }
    if (policy_output_logger_.is_running()) {
        return true;
    }
    return start_policy_output_log();
}

/* Queue policy raw_action, model target, and motor target for async CSV output. */
void RobotInterface::log_policy_output(
    const std::array<float, kPolicyDof>& raw_action,
    const std::vector<double>& target_q_model_rad,
    const std::vector<double>& target_rad) {
    if (!config_.enable_policy_output_log || !ensure_policy_output_log()) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto elapsed_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            now - policy_start_time_).count();

    std::ostringstream row;
    row << std::setprecision(9)
        << policy_output_frame_index_ << ','
        << elapsed_us;
    for (float value : raw_action) {
        row << ',' << value;
    }
    for (double value : target_q_model_rad) {
        row << ',' << value;
    }
    for (double value : target_rad) {
        row << ',' << value;
    }

    if (!policy_output_logger_.enqueue(row.str())) {
        std::cerr << "[RobotInterface] failed to queue policy output log: "
                  << policy_output_logger_.last_error() << "\n";
        policy_output_log_failed_ = true;
        return;
    }

    ++policy_output_frame_index_;
}

/* Stop the current async motor position error log and reset its frame counter. */
void RobotInterface::reset_motor_pos_error_log() {
    motor_pos_error_logger_.stop();
    motor_pos_error_frame_index_ = 0;
    motor_pos_error_log_failed_ = false;
    motor_pos_error_log_start_time_ = std::chrono::steady_clock::now();
}

bool RobotInterface::start_motor_pos_error_log() {
    if (!config_.enable_motor_pos_error_log || motor_pos_error_log_failed_) {
        return false;
    }
    if (motor_pos_error_logger_.is_running()) {
        return true;
    }

    std::ostringstream header;
    write_motor_pos_error_csv_header(header, config_.num_motors);

    AsyncCsvLogger::Config logger_config;
    logger_config.directory = motor_pos_error_log_dir();
    logger_config.file_prefix = "motor_pos_error";
    logger_config.header = header.str();
    logger_config.flush_interval = std::chrono::milliseconds(
        std::max(1, config_.async_log_flush_interval_ms));
    logger_config.max_queue_depth = config_.async_log_queue_depth;

    motor_pos_error_log_start_time_ = std::chrono::steady_clock::now();
    if (!motor_pos_error_logger_.start(std::move(logger_config))) {
        std::cerr << "[RobotInterface] failed to open motor position error log: "
                  << motor_pos_error_logger_.last_error() << "\n";
        motor_pos_error_log_failed_ = true;
        return false;
    }

    std::cout << "[RobotInterface] motor position error log: "
              << motor_pos_error_logger_.log_path() << "\n";
    return true;
}

bool RobotInterface::ensure_motor_pos_error_log() {
    if (!config_.enable_motor_pos_error_log || motor_pos_error_log_failed_) {
        return false;
    }
    if (motor_pos_error_logger_.is_running()) {
        return true;
    }
    return start_motor_pos_error_log();
}

/* Queue target position, feedback position, and TAR_ERR_DEG for async CSV output. */
void RobotInterface::log_motor_pos_error(const std::vector<double>& target_rad) {
    if (!config_.enable_motor_pos_error_log ||
        !controller_ ||
        static_cast<int>(target_rad.size()) != config_.num_motors) {
        return;
    }
    if (!ensure_motor_pos_error_log()) {
        return;
    }

    const std::vector<myactua::MotorStatusSnapshot> status = controller_->get_status();
    if (static_cast<int>(status.size()) != config_.num_motors) {
        std::cerr << "[RobotInterface] failed to write motor position error log: "
                  << "status size mismatch\n";
        motor_pos_error_logger_.stop();
        motor_pos_error_log_failed_ = true;
        return;
    }

    std::vector<double> target_pos_deg(config_.num_motors, 0.0);
    std::vector<double> rx_pos_deg(config_.num_motors, 0.0);
    std::vector<double> tar_err_deg(config_.num_motors, 0.0);
    for (int i = 0; i < config_.num_motors; ++i) {
        target_pos_deg[i] = myactua::MYACTUA::rad_to_deg(target_rad[i]);
        rx_pos_deg[i] = myactua::MYACTUA::rad_to_deg(
            myactua::MYACTUA::raw_pos_to_rad(status[i].position));
        tar_err_deg[i] = target_pos_deg[i] - rx_pos_deg[i];
    }

    const auto now = std::chrono::steady_clock::now();
    const auto elapsed_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            now - motor_pos_error_log_start_time_).count();

    std::ostringstream row;
    row << std::setprecision(9)
        << motor_pos_error_frame_index_ << ','
        << elapsed_us;
    for (double value : target_pos_deg) {
        row << ',' << value;
    }
    for (double value : rx_pos_deg) {
        row << ',' << value;
    }
    for (double value : tar_err_deg) {
        row << ',' << value;
    }

    if (!motor_pos_error_logger_.enqueue(row.str())) {
        std::cerr << "[RobotInterface] failed to queue motor position error log: "
                  << motor_pos_error_logger_.last_error() << "\n";
        motor_pos_error_log_failed_ = true;
        return;
    }

    ++motor_pos_error_frame_index_;
}

/* 兼容旧接口：先保存外部传入的速度指令，再按保存值执行策略闭环。 */
bool RobotInterface::policy_step(double vx, double vy, double yaw_rate) {
    set_target_velocity(vx, vy, yaw_rate);
    return policy_step();
}

/* 执行一次策略闭环：使用保存的目标速度构建观测、模型推理并下发目标关节角。 */
bool RobotInterface::policy_step() {
    const std::array<double, 3> target_velocity = get_target_velocity();

    std::lock_guard<std::mutex> lock(policy_mutex_);

    if (!policy_runner_ || !policy_runner_->is_loaded()) {
        return handle_policy_step_failure("policy is not loaded");
    }

    // 单次策略闭环：状态采样 -> 帧观测 -> 模型推理 -> 目标关节角 -> CSP 下发。
    std::array<float, kPolicyObservationSize> observation = {};
    if (!build_policy_observation(target_velocity[0],
                                  target_velocity[1],
                                  target_velocity[2],
                                  observation)) {
        return handle_policy_step_failure("failed to build policy observation");
    }

    std::array<float, kPolicyDof> raw_action = {};
    if (!policy_runner_->infer(observation, raw_action)) {
        return handle_policy_step_failure(policy_runner_->last_error());
    }

    // last_action_raw_ 保存模型原始输出，下一周期进入 policy.pt 的 last_action 观测历史。
    last_action_raw_ = raw_action;

    std::vector<double> target_q_model_rad(config_.num_motors, 0.0);
    for (int model_index = 0; model_index < kPolicyDof; ++model_index) {
        // 模型输出先按训练约定缩放，再截断缩放后的动作偏移，最后叠加模型顺序的站立姿态。
        const auto& action_clip = config_.action_clip[model_index];
        const double scaled_action =
            static_cast<double>(raw_action[model_index]) * config_.action_scale[model_index];
        const double clipped_action_offset =
            std::max(action_clip[0],
                     std::min(action_clip[1], scaled_action));

        target_q_model_rad[model_index] =
            config_.stand_pose_rad[model_index] + clipped_action_offset;
    }

    if (config_.enable_policy_output_log) {
        std::vector<double> target_rad;
        if (!build_action_target_rad(target_q_model_rad, target_rad)) {
            return handle_policy_step_failure("failed to build action target rad");
        }
        log_policy_output(raw_action, target_q_model_rad, target_rad);
    }

    if (!apply_action(target_q_model_rad)) {
        return handle_policy_step_failure("failed to apply policy target action");
    }

    ++policy_episode_length_;
    return true;
}


/* 首次构建观测历史时，用当前 term 填满历史帧中关于当前的 term。 */
template <std::size_t TermSize, std::size_t ObservationSize>
void fill_term_history(std::array<float, ObservationSize>& history,
                       std::size_t offset,
                       std::size_t frame_stack,
                       const std::array<float, TermSize>& current_term)
{
    for (std::size_t frame = 0; frame < frame_stack; ++frame) {
        std::copy(current_term.begin(), current_term.end(), history.begin() + offset + frame * TermSize);
    }
}

/* 后续构建观测历史时，左移旧帧并把当前 term 追加到末尾。 */
template <std::size_t TermSize, std::size_t ObservationSize>
void append_term_history(std::array<float, ObservationSize>& history,
                         std::size_t offset,
                         std::size_t frame_stack,
                         const std::array<float, TermSize>& current_term)
{
    const std::size_t term_history_size = frame_stack * TermSize;
    /* 左移旧帧 */
    std::copy(history.begin() + offset + TermSize, history.begin() + offset + term_history_size, history.begin() + offset);
    /* 拷贝新帧 */
    std::copy(current_term.begin(), current_term.end(), history.begin() + offset + term_history_size - TermSize);
}


void RobotInterface::build_joint_observation_terms(
    const std::vector<double>& q_rad,
    const std::vector<double>& dq_rad_s,
    std::chrono::steady_clock::time_point now,
    std::array<float, kPolicyDof>& joint_pos_rel,
    std::array<float, kPolicyDof>& joint_vel_rel) {

    /* 普通单关节按直接映射填充；并联脚踝跳过后由 FK 写入。 */
    for (int model_index = 0; model_index < kPolicyDof; ++model_index) {
        if (is_parallel_ankle_model_dof(model_index)) {
            continue;
        }

        const int mapping_slot = direct_model_mapping_slot(model_index);
        if (mapping_slot < 0 ||
            mapping_slot >= static_cast<int>(config_.model_to_motor_index.size())) {
            continue;
        }
        const int motor_index = config_.model_to_motor_index[mapping_slot];
        if (!index_in_range(motor_index, kPolicyDof)) {
            continue;
        }
        const double direction = static_cast<double>(direction_for_motor(config_, motor_index));
        const double q_model  = direction * q_rad[motor_index];
        const double dq_model = direction * dq_rad_s[motor_index];

        joint_pos_rel[model_index] =
            static_cast<float>((q_model - config_.stand_pose_rad[model_index]) *
                               config_.dof_pos_scale[model_index]);
        joint_vel_rel[model_index] =
            static_cast<float>(dq_model * config_.dof_vel_scale[model_index]);
    }

    const double dt = ankle_fk_observation_ready_
        ? std::chrono::duration<double>(now - ankle_fk_last_time_).count()
        : 0.0;

    const bool has_valid_dt = ankle_fk_observation_ready_ &&
                              std::isfinite(dt) &&
                              dt > 0.0;

    apply_ankle_fk_observation(q_rad, dt, has_valid_dt,
                               config_.left_ankle_parallel,
                               left_ankle_fk_solver_,
                               joint_pos_rel, joint_vel_rel);
    apply_ankle_fk_observation(q_rad, dt, has_valid_dt,
                               config_.right_ankle_parallel,
                               right_ankle_fk_solver_,
                               joint_pos_rel, joint_vel_rel);

    ankle_fk_observation_ready_ = true;
    ankle_fk_last_time_ = now;
}

void RobotInterface::apply_ankle_fk_observation(
    const std::vector<double>& q_rad,
    double dt,
    bool has_valid_dt,
    const RobotInterfaceConfig::AnkleParallelMap& ankle_map,
    ankle_motor_fk::Solver& solver,
    std::array<float, kPolicyDof>& joint_pos_rel,
    std::array<float, kPolicyDof>& joint_vel_rel) {

    if (!ankle_parallel_map_indices_in_range(ankle_map, kPolicyDof)) {
        return;
    }

    const int ankle_model_pitch_dof = ankle_map.model_pitch_dof;
    const int ankle_model_roll_dof = ankle_map.model_roll_dof;
    const double upper_motor =
        static_cast<double>(direction_for_motor(config_, ankle_map.upper_motor_index)) *
        q_rad[ankle_map.upper_motor_index];
    const double lower_motor =
        static_cast<double>(direction_for_motor(config_, ankle_map.lower_motor_index)) *
        q_rad[ankle_map.lower_motor_index];

    const double previous_roll = solver.previous_roll();
    const double previous_pitch = solver.previous_pitch();
    const ankle_motor_fk::FootAngles foot = solver.solve(upper_motor, lower_motor);

    joint_pos_rel[ankle_model_pitch_dof] =
        static_cast<float>((foot.pitch - config_.stand_pose_rad[ankle_model_pitch_dof]) *
                           config_.dof_pos_scale[ankle_model_pitch_dof]);
    joint_pos_rel[ankle_model_roll_dof] =
        static_cast<float>((foot.roll - config_.stand_pose_rad[ankle_model_roll_dof]) *
                           config_.dof_pos_scale[ankle_model_roll_dof]);

    double pitch_velocity = 0.0;
    double roll_velocity = 0.0;
    if (has_valid_dt) {
        pitch_velocity = ankle_motor_ik::wrap_to_pi(foot.pitch - previous_pitch) / dt;
        roll_velocity = ankle_motor_ik::wrap_to_pi(foot.roll - previous_roll) / dt;
    }

    joint_vel_rel[ankle_model_pitch_dof] =
        static_cast<float>(pitch_velocity * config_.dof_vel_scale[ankle_model_pitch_dof]);
    joint_vel_rel[ankle_model_roll_dof] =
        static_cast<float>(roll_velocity * config_.dof_vel_scale[ankle_model_roll_dof]);
}

/* 结合速度指令、关节状态和 IMU 数据构建完整策略观测。 */
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
    if (q_rad.size() != static_cast<std::size_t>(kPolicyDof) || dq_rad_s.size() != static_cast<std::size_t>(kPolicyDof)) {
        std::cerr << "[RobotInterface] policy observation rejected: joint state size mismatch\n";
        return false;
    }

    std::array<double, 3> body_ang_vel = {};
    std::array<double, 3> projected_gravity_double = {};
    bool projected_gravity_valid = false;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        body_ang_vel = body_ang_vel_;
        projected_gravity_double = projected_gravity_;
        projected_gravity_valid = projected_gravity_valid_;
    }

    const auto now = std::chrono::steady_clock::now();

    if (!finite_vector(q_rad) || !finite_vector(dq_rad_s) ||
        !finite_array3(body_ang_vel) || !finite_array3(projected_gravity_double)) {
        std::cerr << "[RobotInterface] policy observation rejected: sensor value is not finite\n";
        return false;
    }
    if (!projected_gravity_valid) {
        std::cerr << "[RobotInterface] policy observation rejected: projected gravity is invalid\n";
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

    /* 构建缩放后的 角速度 与 projected_gravity */
    for (int i = 0; i < 3; ++i) {
        base_ang_vel[i] = static_cast<float>(body_ang_vel[i] * config_.body_ang_vel_scale[i]);
        projected_gravity[i] = static_cast<float>(projected_gravity_double[i]);
    }

    build_joint_observation_terms(q_rad, dq_rad_s, now, joint_pos_rel, joint_vel_rel);

    constexpr std::size_t kBaseAngVelOffset = 0;
    constexpr std::size_t kBaseAngVelHistorySize =
        policy_observation::kFrameStack * policy_observation::kBaseAngVelSize;
    constexpr std::size_t kProjectedGravityOffset =
        kBaseAngVelOffset + kBaseAngVelHistorySize;
    constexpr std::size_t kProjectedGravityHistorySize =
        policy_observation::kFrameStack * policy_observation::kProjectedGravitySize;
    constexpr std::size_t kVelocityCommandsOffset =
        kProjectedGravityOffset + kProjectedGravityHistorySize;
    constexpr std::size_t kVelocityCommandsHistorySize =
        policy_observation::kFrameStack * policy_observation::kVelocityCommandsSize;
    constexpr std::size_t kGaitPhaseOffset =
        kVelocityCommandsOffset + kVelocityCommandsHistorySize;
    constexpr std::size_t kGaitPhaseHistorySize =
        policy_observation::kEnableGaitPhase
            ? policy_observation::kFrameStack * policy_observation::kGaitPhaseSize
            : 0;
    constexpr std::size_t kJointPosRelOffset =
        kGaitPhaseOffset + kGaitPhaseHistorySize;
    constexpr std::size_t kJointPosRelHistorySize =
        policy_observation::kFrameStack * policy_observation::kJointPosRelSize;
    constexpr std::size_t kJointVelRelOffset =
        kJointPosRelOffset + kJointPosRelHistorySize;
    constexpr std::size_t kJointVelRelHistorySize =
        policy_observation::kFrameStack * policy_observation::kJointVelRelSize;
    constexpr std::size_t kLastActionOffset =
        kJointVelRelOffset + kJointVelRelHistorySize;
    constexpr std::size_t kObservationEnd =
        kLastActionOffset + policy_observation::kFrameStack * policy_observation::kLastActionSize;
    static_assert(kObservationEnd == policy_observation::kObservationSize,
                  "policy observation offsets must cover the configured input size");

    if (!observation_history_ready_) {
        fill_term_history(observation_history_, kBaseAngVelOffset,
                          kPolicyFrameStack, base_ang_vel);
        fill_term_history(observation_history_, kProjectedGravityOffset,
                          kPolicyFrameStack, projected_gravity);
        fill_term_history(observation_history_, kVelocityCommandsOffset,
                          kPolicyFrameStack, velocity_commands);
        if constexpr (policy_observation::kEnableGaitPhase) {
            const std::array<float, 2> gait_phase =
                gait_phase_observation(policy_episode_length_,
                                       config_.policy_step_dt,
                                       config_.gait_phase_period);
            fill_term_history(observation_history_, kGaitPhaseOffset,
                              kPolicyFrameStack, gait_phase);
        }
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
        if constexpr (policy_observation::kEnableGaitPhase) {
            const std::array<float, 2> gait_phase =
                gait_phase_observation(policy_episode_length_,
                                       config_.policy_step_dt,
                                       config_.gait_phase_period);
            append_term_history(observation_history_, kGaitPhaseOffset,
                                kPolicyFrameStack, gait_phase);
        }
        append_term_history(observation_history_, kJointPosRelOffset,
                            kPolicyFrameStack, joint_pos_rel);
        append_term_history(observation_history_, kJointVelRelOffset,
                            kPolicyFrameStack, joint_vel_rel);
        append_term_history(observation_history_, kLastActionOffset,
                            kPolicyFrameStack, last_action_raw_);
    }

    observation = observation_history_;
    return true;
}

/* 处理策略执行失败：打印错误并停止全部电机。 */
bool RobotInterface::handle_policy_step_failure(const std::string& message) {
    std::cerr << "[RobotInterface] policy_step failed: " << message << "\n";
    // 策略链路任何一步失败都停机，避免继续执行上一周期的目标。
    stop_motors(-1);
    return false;
}

}  // namespace inference
