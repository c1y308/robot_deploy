#ifndef __TYPES_HPP__
#define __TYPES_HPP__

#include <cstdint>
#include <string>

namespace imu {
//  起始标志、结束标志
constexpr uint8_t FRAME_HEAD = 0xFC;
constexpr uint8_t FRAME_END = 0xFD;
constexpr uint8_t TYPE_IMU = 0x40;
constexpr uint8_t TYPE_AHRS = 0x41;
constexpr uint8_t TYPE_INSGPS = 0x42;
// 数据长度
constexpr uint8_t IMU_LEN = 0x3C;
constexpr uint8_t AHRS_LEN = 0x38;
constexpr uint8_t INSGPS_LEN = 0x50;
// 帧大小
constexpr size_t IMU_FRAME_SIZE = 64;
constexpr size_t AHRS_FRAME_SIZE = 56;
constexpr size_t INSGPS_FRAME_SIZE = 80;

/* 定义帧的数据类型 */
enum class DataType_e {
    NONE = 0,
    IMU,
    AHRS,
    INSGPS
};


struct IMUData_t {
    float gyroscope_x;  // rad/s
    float gyroscope_y;
    float gyroscope_z;

    float accelerometer_x;  // m/s²
    float accelerometer_y;
    float accelerometer_z;

    float magnetometer_x;  // mG
    float magnetometer_y;
    float magnetometer_z;

    float imu_temperature;  // °C
    float pressure;  // Pa
    float pressure_temperature;  // °C
    uint32_t timestamp;  // 时间戳，us
    
    // 构造函数
    IMUData_t() : gyroscope_x(0), gyroscope_y(0), gyroscope_z(0),
                  accelerometer_x(0), accelerometer_y(0), accelerometer_z(0),
                  magnetometer_x(0), magnetometer_y(0), magnetometer_z(0),
                  imu_temperature(0), pressure(0), pressure_temperature(0),
                  timestamp(0) {}
};


struct AHRSData_t {
    float roll_speed;
    float pitch_speed;
    float heading_speed;
    float roll;
    float pitch;
    float heading;
    float qw;
    float qx;
    float qy;
    float qz;
    uint64_t timestamp;
    
    AHRSData_t() : roll_speed(0), pitch_speed(0), heading_speed(0),
                   roll(0), pitch(0), heading(0),
                   qw(0), qx(0), qy(0), qz(0),
                   timestamp(0) {}
};


struct INSGPSData_t {
    double latitude;
    double longitude;
    double altitude;
    float north_velocity;
    float east_velocity;
    float ground_velocity;
    float azimuth;
    float pitch;
    float roll;
    uint64_t timestamp;
    
    INSGPSData_t() : latitude(0), longitude(0), altitude(0),
                     north_velocity(0), east_velocity(0), ground_velocity(0),
                     azimuth(0), pitch(0), roll(0),
                     timestamp(0) {}
};


/* 解析器统计信息 */
struct ParserInfo_t {
    uint64_t total_bytes;
    
    uint64_t total_frames;
    uint64_t imu_frames;
    uint64_t ahrs_frames;
    uint64_t insgps_frames;
    uint64_t error_frames;
    
    ParserInfo_t() : total_bytes(0), total_frames(0), 
                      imu_frames(0), ahrs_frames(0), insgps_frames(0),
                      error_frames(0) {}
};

}

#endif
