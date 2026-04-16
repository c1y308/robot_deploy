#include "robot_interface.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>


namespace inference {

RobotInterface::RobotInterface(RobotInterfaceConfig config)
    : config_(std::move(config)) {}

RobotInterface::~RobotInterface() {
    deinit_motors();
}




bool RobotInterface::initial_and_start_imu() {
    return initial_start_imu_reader();
}


void RobotInterface::deinit_imu() {
    stop_imu_reader();
}


/* 初始化并启动 IMU 读取器 */
bool RobotInterface::initial_start_imu_reader() {
    if (imu_initialized_.load()) {
        return true;
    }

    imu::Config_t imu_cfg;
    imu_cfg.device      = config_.imu_device;
    imu_cfg.baudrate    = config_.imu_baudrate;
    imu_cfg.print_imu   = config_.imu_print_imu;
    imu_cfg.print_ahrs  = config_.imu_print_ahrs;
    imu_cfg.print_stats = config_.imu_print_stats;

    imu_reader_ = std::make_unique<imu::IMUReader>();
    imu_reader_->set_imu_callback([this](const imu::IMUData_t& data) {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        ang_vel_[0] = static_cast<double>(data.gyroscope_x);
        ang_vel_[1] = static_cast<double>(data.gyroscope_y);
        ang_vel_[2] = static_cast<double>(data.gyroscope_z);
        // 当前 parser 仅提供 IMU 帧，不含四元数。先保持单位四元数占位。
    });
    imu_reader_->set_ahrs_callback([this](const imu::AHRSData_t& data) {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        ang_vel_[0] = static_cast<double>(data.roll_speed);
        ang_vel_[1] = static_cast<double>(data.pitch_speed);
        ang_vel_[2] = static_cast<double>(data.heading_speed);
        quat_[0] = static_cast<double>(data.qw);
        quat_[1] = static_cast<double>(data.qx);
        quat_[2] = static_cast<double>(data.qy);
        quat_[3] = static_cast<double>(data.qz);
    });

    /* 调用 imu class的初始化函数 */
    if (!imu_reader_->initialize(imu_cfg)) {
        std::cerr << "[RobotInterface] IMU initialize failed.\n";
        imu_reader_.reset();
        imu_initialized_.store(false);
        return false;
    }

    imu_thread_ = std::thread([this]() {
        if (imu_reader_) {
            imu_reader_->run();
        }
    });
    imu_initialized_.store(true);
    return true;
}

/* 停止 IMU 读取器 */
void RobotInterface::stop_imu_reader() {
    if (imu_reader_) {
        imu_reader_->stop();
    }
    if (imu_thread_.joinable()) {
        imu_thread_.join();
    }
    imu_reader_.reset();
    imu_initialized_.store(false);
}





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
        controller_->set_mode(myactua::ControlMode::CSP, i);
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
    const int max_tries  = (timeout_ms == 0) ? 1 : (timeout_ms + poll_ms - 1) / poll_ms;

    for (int t = 0; t < max_tries; ++t) {
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

        std::cout << "[RobotInterface] EtherCAT ready: "
                  << ready_count << "/" << config_.num_motors << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
    }

    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time).count();
    std::cout << "[RobotInterface] wait_all_slaves_ready timeout after "
              << elapsed_ms << " ms\n";
    return false;
}


void RobotInterface::deinit_motors() {
    stop_imu_reader();

    if (motors_initialized_.load() && controller_) {
        controller_->send_command(myactua::ControlCommand(myactua::CommandType::STOP, -1));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        controller_->shutdown();
    }

    controller_.reset();
    adapter_.reset();
    motors_initialized_.store(false);
    imu_initialized_.store(false);
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


std::array<double, 4> RobotInterface::get_quat() const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return quat_;
}


std::array<double, 3> RobotInterface::get_ang_vel() const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return ang_vel_;
}


bool RobotInterface::apply_action(const std::vector<double>& target_q_rad) {
    if (!motors_initialized_.load() || !controller_) {
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


bool RobotInterface::reset_joints() {
    if (!motors_initialized_.load()) {
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


}  // namespace inference
