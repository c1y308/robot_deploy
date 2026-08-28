#pragma once

#include <cstdint>
#include <functional>
#include <string>

namespace imu_base {

struct ReaderConfig {
    std::string device = "/dev/ttyUSB0";
    int baudrate = 921600;
    bool print_imu = false;
    bool print_ahrs = false;
};

struct IMUData {
    float gyroscope_x = 0.0F;
    float gyroscope_y = 0.0F;
    float gyroscope_z = 0.0F;

    float accelerometer_x = 0.0F;
    float accelerometer_y = 0.0F;
    float accelerometer_z = 0.0F;

    float magnetometer_x = 0.0F;
    float magnetometer_y = 0.0F;
    float magnetometer_z = 0.0F;

    float imu_temperature = 0.0F;
    float pressure = 0.0F;
    float pressure_temperature = 0.0F;
    std::uint32_t timestamp = 0;
};

struct AHRSData {
    float roll_speed = 0.0F;
    float pitch_speed = 0.0F;
    float heading_speed = 0.0F;
    float roll = 0.0F;
    float pitch = 0.0F;
    float heading = 0.0F;
    float qw = 0.0F;
    float qx = 0.0F;
    float qy = 0.0F;
    float qz = 0.0F;
    float projected_gravity_x = 0.0F;
    float projected_gravity_y = 0.0F;
    float projected_gravity_z = 0.0F;
    bool projected_gravity_valid = false;
    std::uint64_t timestamp = 0;
};

struct ParserInfo {
    std::uint64_t total_bytes = 0;
    std::uint64_t total_frames = 0;
    std::uint64_t imu_frames = 0;
    std::uint64_t ahrs_frames = 0;
    std::uint64_t error_frames = 0;
};

class IMUReaderBase {
public:
    using IMUCallback = std::function<void(const IMUData&)>;
    using AHRSCallback = std::function<void(const AHRSData&)>;

    virtual ~IMUReaderBase() = default;

    IMUReaderBase(const IMUReaderBase&) = delete;
    IMUReaderBase& operator=(const IMUReaderBase&) = delete;

    virtual bool start(const ReaderConfig& config) = 0;
    virtual void stop() = 0;
    virtual bool is_running() const = 0;
    virtual const ParserInfo& get_info() const = 0;

    virtual void set_imu_callback(IMUCallback callback) = 0;
    virtual void set_ahrs_callback(AHRSCallback callback) = 0;

protected:
    IMUReaderBase() = default;
};

}  // namespace imu_base
