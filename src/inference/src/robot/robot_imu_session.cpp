#include "robot/robot_imu_session.hpp"

#include "imu_reader.hpp"

#include <chrono>
#include <iostream>
#include <utility>

namespace inference {
namespace {

std::int64_t imu_steady_now_ns() noexcept
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace

RobotImuSession::RobotImuSession(ImuConfig config)
    : config_(std::move(config)) {}

RobotImuSession::~RobotImuSession()
{
    deinitialize();
}

bool RobotImuSession::initialize_and_start()
{
    if (initialized_.load()) {
        return true;
    }

    imu::Config_t imu_cfg;
    imu_cfg.device      = config_.device;
    imu_cfg.baudrate    = config_.baudrate;
    imu_cfg.print_imu   = config_.print_imu;
    imu_cfg.print_ahrs  = config_.print_ahrs;

    ahrs_ready_.store(false);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.ahrs_ready = false;
        state_.projected_gravity_valid = false;
    }

    reader_ = std::make_unique<imu::IMUReader>();
    reader_->set_imu_callback([](const imu::IMUData_t& data) {
        (void)data;
    });
    reader_->set_ahrs_callback([this](const imu::AHRSData_t& data) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.timestamp_ns = imu_steady_now_ns();

        state_.body_ang_vel[0] = static_cast<double>(data.roll_speed);
        state_.body_ang_vel[1] = static_cast<double>(data.pitch_speed);
        state_.body_ang_vel[2] = static_cast<double>(data.heading_speed);

        state_.euler[0] = static_cast<double>(data.roll);
        state_.euler[1] = static_cast<double>(data.pitch);
        state_.euler[2] = static_cast<double>(data.heading);

        state_.quat[0] = static_cast<double>(data.qw);
        state_.quat[1] = static_cast<double>(data.qx);
        state_.quat[2] = static_cast<double>(data.qy);
        state_.quat[3] = static_cast<double>(data.qz);

        state_.projected_gravity[0] = static_cast<double>(data.projected_gravity_x);
        state_.projected_gravity[1] = static_cast<double>(data.projected_gravity_y);
        state_.projected_gravity[2] = static_cast<double>(data.projected_gravity_z);
        state_.projected_gravity_valid = data.projected_gravity_valid;
        state_.ahrs_ready = true;
        ahrs_ready_.store(true);
    });

    if (!reader_->start(imu_cfg)) {
        std::cerr << "[RobotInterface] IMU start failed.\n";
        reader_.reset();
        initialized_.store(false);
        return false;
    }

    initialized_.store(true);
    return true;
}

void RobotImuSession::deinitialize()
{
    if (reader_) {
        reader_->stop();
    }
    reader_.reset();
    initialized_.store(false);
    ahrs_ready_.store(false);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.ahrs_ready = false;
        state_.projected_gravity_valid = false;
    }
}

ImuStateSnapshot RobotImuSession::get_state() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

std::array<double, 4> RobotImuSession::get_quat() const
{
    return get_state().quat;
}

std::array<double, 3> RobotImuSession::get_body_ang_vel() const
{
    return get_state().body_ang_vel;
}

std::array<double, 3> RobotImuSession::get_euler() const
{
    return get_state().euler;
}

std::array<double, 3> RobotImuSession::get_projected_gravity() const
{
    const ImuStateSnapshot state = get_state();
    if (!state.projected_gravity_valid) {
        return {0.0, 0.0, 0.0};
    }
    return state.projected_gravity;
}

}  // namespace inference
