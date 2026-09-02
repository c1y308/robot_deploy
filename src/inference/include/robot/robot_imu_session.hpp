#pragma once

#include "robot/robot_config.hpp"
#include "spsc_latest_value/spsc_latest_value.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <type_traits>

namespace imu_base {
class IMUReaderBase;
}

namespace inference {

struct AhrsStateSnapshot {
    std::int64_t receive_timestamp_ns{0};
    std::uint64_t sample_timestamp_ns{0};

    std::array<double, 4> quat{1.0, 0.0, 0.0, 0.0};
    std::array<double, 3> body_ang_vel{0.0, 0.0, 0.0};
    std::array<double, 3> euler{0.0, 0.0, 0.0};
    std::array<double, 3> projected_gravity{0.0, 0.0, -1.0};

    bool ahrs_ready{false};
    bool projected_gravity_valid{false};
};

static_assert(std::is_trivially_copyable<AhrsStateSnapshot>::value,
              "AhrsStateSnapshot must be trivially copyable for SpscLatestValue");

inline imu_base::ReaderConfig make_reader_config(const ImuConfig& config)
{
    imu_base::ReaderConfig imu_cfg;
    imu_cfg.type = config.type;
    imu_cfg.device = config.device;
    imu_cfg.baudrate = config.baudrate;
    imu_cfg.configure_can = config.configure_can;
    imu_cfg.can_bitrate = config.can_bitrate;
    imu_cfg.print_imu = config.print_imu;
    imu_cfg.print_ahrs = config.print_ahrs;
    return imu_cfg;
}

class RobotImuSession {
public:
    explicit RobotImuSession(ImuConfig config);
    ~RobotImuSession();

    RobotImuSession(const RobotImuSession&) = delete;
    RobotImuSession& operator=(const RobotImuSession&) = delete;

    bool initialize_and_start();
    void deinitialize();

    bool is_initialized() const noexcept { return initialized_.load(); }
    bool ahrs_ready() const noexcept { return ahrs_ready_.load(); }

    bool get_ahrs_snapshot(AhrsStateSnapshot& out);

private:
    ImuConfig config_;
    std::unique_ptr<imu_base::IMUReaderBase> reader_;

    std::atomic<bool> initialized_{false};
    std::atomic<bool> ahrs_ready_{false};

    robot_base::SpscLatestValue<AhrsStateSnapshot> ahrs_state_channel_;
    AhrsStateSnapshot latest_ahrs_state_cache_;
    bool has_ahrs_state_cache_{false};
};

}  // namespace inference
