#pragma once

#include "robot/robot_config.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>

namespace imu {
class IMUReader;
}

namespace inference {

struct ImuStateSnapshot {
    std::int64_t timestamp_ns{0};

    std::array<double, 4> quat{1.0, 0.0, 0.0, 0.0};
    std::array<double, 3> body_ang_vel{0.0, 0.0, 0.0};
    std::array<double, 3> euler{0.0, 0.0, 0.0};
    std::array<double, 3> projected_gravity{0.0, 0.0, -1.0};

    bool ahrs_ready{false};
    bool projected_gravity_valid{false};
};

class RobotImuSession {
public:
    explicit RobotImuSession(ImuConfig config = {});
    ~RobotImuSession();

    RobotImuSession(const RobotImuSession&) = delete;
    RobotImuSession& operator=(const RobotImuSession&) = delete;

    bool initialize_and_start();
    void deinitialize();

    bool is_initialized() const noexcept { return initialized_.load(); }
    bool ahrs_ready() const noexcept { return ahrs_ready_.load(); }

    ImuStateSnapshot get_state() const;

    std::array<double, 4> get_quat() const;
    std::array<double, 3> get_body_ang_vel() const;
    std::array<double, 3> get_euler() const;
    std::array<double, 3> get_projected_gravity() const;

private:
    ImuConfig config_;
    std::unique_ptr<imu::IMUReader> reader_;

    std::atomic<bool> initialized_{false};
    std::atomic<bool> ahrs_ready_{false};

    mutable std::mutex mutex_;
    ImuStateSnapshot state_;
};

}  // namespace inference
