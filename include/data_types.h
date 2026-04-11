#ifndef __DATA_TYPES_H
#define __DATA_TYPES_H

#include <stdint.h>

#define FRAME_HEAD      0xFC
#define FRAME_END       0xFD

#define TYPE_IMU        0x40
#define TYPE_AHRS       0x41
#define TYPE_INSGPS     0x42
#define TYPE_GEODETIC_POS 0x5C
#define TYPE_GROUND     0xF0

#define IMU_LEN         0x38
#define AHRS_LEN        0x30
#define INSGPS_LEN      0x48
#define GEODETIC_POS_LEN 0x20

#define IMU_FRAME_SIZE  64
#define AHRS_FRAME_SIZE 56
#define INSGPS_FRAME_SIZE 80
#define GEODETIC_POS_FRAME_SIZE 40

typedef struct {
    float gyroscope_x;
    float gyroscope_y;
    float gyroscope_z;
    float accelerometer_x;
    float accelerometer_y;
    float accelerometer_z;
    float magnetometer_x;
    float magnetometer_y;
    float magnetometer_z;
    float imu_temperature;
    float pressure;
    float pressure_temperature;
    uint32_t timestamp;
} IMUData_Packet_t;

typedef struct {
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
} AHRSData_Packet_t;

typedef struct {
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
} INSGPSData_Packet_t;

typedef enum {
    DATA_TYPE_NONE = 0,
    DATA_TYPE_IMU,
    DATA_TYPE_AHRS,
    DATA_TYPE_INSGPS
} DataType_e;

typedef struct {
    DataType_e type;
    union {
        IMUData_Packet_t imu;
        AHRSData_Packet_t ahrs;
        INSGPSData_Packet_t insgps;
    } data;
} IMUDataFrame_t;

#endif
