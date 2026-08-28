#ifndef __TYPES_HPP__
#define __TYPES_HPP__

#include "imu_base/imu_base.hpp"

#include <cstddef>
#include <cstdint>

namespace imu {
//  起始标志、结束标志
constexpr uint8_t FRAME_HEAD = 0xFC;
constexpr uint8_t FRAME_END = 0xFD;

constexpr uint8_t TYPE_IMU = 0x40;
constexpr uint8_t TYPE_AHRS = 0x41;
constexpr uint8_t TYPE_UNKNOWN = 0xf0;
// 数据长度
constexpr uint8_t IMU_LEN  = 56;
constexpr uint8_t AHRS_LEN = 48;
// 帧大小
constexpr std::size_t IMU_FRAME_SIZE  = 64;
constexpr std::size_t AHRS_FRAME_SIZE = 56;

using Config_t = imu_base::ReaderConfig;
using IMUData_t = imu_base::IMUData;
using AHRSData_t = imu_base::AHRSData;
using ParserInfo_t = imu_base::ParserInfo;

}

#endif
