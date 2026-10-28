#pragma once

#include "robot/robot_config.hpp"
#include "spsc_latest_channel/spsc_latest_channel.hpp"

#include <array>
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

    std::array<double, 3> body_ang_vel{0.0, 0.0, 0.0};
    std::array<double, 3> euler{0.0, 0.0, 0.0};
    std::array<double, 3> projected_gravity{0.0, 0.0, -1.0};

    bool projected_gravity_valid{false};
};

static_assert(std::is_trivially_copyable<AhrsStateSnapshot>::value,
              "AhrsStateSnapshot must be trivially copyable for SpscLatestChannel");

class RobotImuSession {
public:
    // 生命周期和状态读取由所有者线程串行调用；AHRS 回调仅发布独立快照。
    explicit RobotImuSession(ImuConfig config,
                             RuntimeThreadingConfig runtime = {});
    ~RobotImuSession();

    RobotImuSession(const RobotImuSession&) = delete;
    RobotImuSession& operator=(const RobotImuSession&) = delete;

    bool initialize();
    void deinitialize();

    bool is_initialized() const noexcept { return initialized_; }

    bool get_ahrs_snapshot(AhrsStateSnapshot& out);

private:
    ImuConfig config_;
    RuntimeThreadingConfig runtime_;
    std::unique_ptr<imu_base::IMUReaderBase> reader_;

    bool initialized_{false};

    robot_base::SpscLatestChannel<AhrsStateSnapshot> ahrs_state_channel_;
    AhrsStateSnapshot latest_ahrs_state_cache_;
    bool has_ahrs_state_cache_{false};
};

}  // namespace inference
