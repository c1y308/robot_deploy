#include "robot/robot_imu_session.hpp"

#include "imu_base/imu_base.hpp"
#include "driver/a100/a100_reader.hpp"
#include "driver/xsens_mti/xsens_reader.hpp"
#include "tool/tool.hpp"

#include <iostream>
#include <utility>

namespace inference {
namespace {

std::int64_t ahrs_publish_timestamp_ns(const imu_base::AHRSData& data) noexcept
{
    return data.host_publish_timestamp_ns != 0
               ? data.host_publish_timestamp_ns
               : robot_base::monotonic_now_ns();
}

std::int64_t fallback_imu_sample_timestamp_ns(
    std::int64_t host_receive_timestamp_ns,
    std::int64_t host_publish_timestamp_ns) noexcept
{
    return host_receive_timestamp_ns != 0 ? host_receive_timestamp_ns
                                          : host_publish_timestamp_ns;
}

}  // namespace

RobotImuSession::RobotImuSession(ImuConfig config)
    : config_(std::move(config)),
      timestamp_mapper_(config_.type)
{
}

RobotImuSession::~RobotImuSession()
{
    deinitialize();
}

bool RobotImuSession::initialize_and_start()
{
    if (initialized_.load()) {
        return true;
    }

    imu_base::ReaderConfig imu_cfg;
    imu_cfg.type        = config_.type;
    imu_cfg.device      = config_.device;
    imu_cfg.baudrate    = config_.baudrate;
    imu_cfg.print_imu   = config_.print_imu;
    imu_cfg.print_ahrs  = config_.print_ahrs;

    ahrs_ready_.store(false);
    timestamp_mapper_.reset(config_.type);
    ahrs_state_channel_.reset_empty();
    latest_ahrs_state_cache_ = AhrsStateSnapshot();
    has_ahrs_state_cache_ = false;

    switch (config_.type) {
        case imu_base::ReaderType::A100_SERIAL:
            reader_ = std::make_unique<imu::IMUReader>();
            break;
        case imu_base::ReaderType::XSENS_MTI_CAN:
            reader_ = std::make_unique<imu::XsensMtiCanReader>();
            break;
    }
    reader_->set_imu_callback([](const imu_base::IMUData& data) {
        (void)data;
    });
    reader_->set_ahrs_callback([this](const imu_base::AHRSData& data) {
        const std::int64_t host_publish_timestamp_ns =
            ahrs_publish_timestamp_ns(data);
        const std::int64_t host_receive_timestamp_ns =
            data.host_receive_timestamp_ns;

        std::int64_t host_sample_timestamp_ns = 0;
        const bool mapped_device_timestamp =
            data.timestamp_valid &&
            timestamp_mapper_.map_device_timestamp_us(
                data.timestamp,
                host_receive_timestamp_ns,
                host_sample_timestamp_ns);
        if (!mapped_device_timestamp) {
            host_sample_timestamp_ns = fallback_imu_sample_timestamp_ns(
                host_receive_timestamp_ns,
                host_publish_timestamp_ns);
        }

        AhrsStateSnapshot state;
        state.device_timestamp_us = data.timestamp;
        state.device_timestamp_valid = mapped_device_timestamp;
        state.host_receive_timestamp_ns = host_receive_timestamp_ns;
        state.host_publish_timestamp_ns = host_publish_timestamp_ns;
        state.host_sample_timestamp_ns = host_sample_timestamp_ns;
        state.timestamp_ns = host_sample_timestamp_ns;

        state.body_ang_vel[0] = static_cast<double>(data.roll_speed);
        state.body_ang_vel[1] = static_cast<double>(data.pitch_speed);
        state.body_ang_vel[2] = static_cast<double>(data.heading_speed);

        state.euler[0] = static_cast<double>(data.roll);
        state.euler[1] = static_cast<double>(data.pitch);
        state.euler[2] = static_cast<double>(data.heading);

        state.quat[0] = static_cast<double>(data.qw);
        state.quat[1] = static_cast<double>(data.qx);
        state.quat[2] = static_cast<double>(data.qy);
        state.quat[3] = static_cast<double>(data.qz);

        state.projected_gravity[0] = static_cast<double>(data.projected_gravity_x);
        state.projected_gravity[1] = static_cast<double>(data.projected_gravity_y);
        state.projected_gravity[2] = static_cast<double>(data.projected_gravity_z);
        state.projected_gravity_valid = data.projected_gravity_valid;
        state.ahrs_ready = true;
        ahrs_state_channel_.publish(state);
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
    timestamp_mapper_.reset(config_.type);
    ahrs_state_channel_.reset_empty();
    latest_ahrs_state_cache_ = AhrsStateSnapshot();
    has_ahrs_state_cache_ = false;
}

bool RobotImuSession::get_ahrs_snapshot(AhrsStateSnapshot& out)
{
    AhrsStateSnapshot latest;
    if (ahrs_state_channel_.try_consume_latest(latest)) {
        latest_ahrs_state_cache_ = latest;
        has_ahrs_state_cache_ = true;
    }
    if (!has_ahrs_state_cache_) {
        return false;
    }

    out = latest_ahrs_state_cache_;
    return true;
}

}  // namespace inference
