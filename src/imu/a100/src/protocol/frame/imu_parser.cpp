#include "imu_parser.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <iomanip>

namespace imu {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double wrap_to_pi(double angle)
{
    if (angle > kPi) {
        angle -= kTwoPi;
    } else if (angle < -kPi) {
        angle += kTwoPi;
    }
    return angle;
}

bool compute_projected_gravity(float roll,
                               float pitch,
                               float& projected_gravity_x,
                               float& projected_gravity_y,
                               float& projected_gravity_z)
{
    if (!std::isfinite(roll) || !std::isfinite(pitch)) {
        return false;
    }

    const double roll_rad = static_cast<double>(roll);
    const double pitch_rad = static_cast<double>(pitch);
    const double sin_roll = std::sin(roll_rad);
    const double cos_roll = std::cos(roll_rad);
    const double sin_pitch = std::sin(pitch_rad);
    const double cos_pitch = std::cos(pitch_rad);

    // IsaacLab projected gravity for body axes x-forward, y-left, z-up.
    // Use compensated AHRS Euler angles because the raw quaternion is still in the IMU mount frame.
    const double projected_x = sin_pitch;
    const double projected_y = -sin_roll * cos_pitch;
    const double projected_z = -cos_roll * cos_pitch;
    if (!std::isfinite(projected_x) ||
        !std::isfinite(projected_y) ||
        !std::isfinite(projected_z)) {
        return false;
    }

    projected_gravity_x = static_cast<float>(projected_x);
    projected_gravity_y = static_cast<float>(projected_y);
    projected_gravity_z = static_cast<float>(projected_z);
    return true;
}

}  // namespace

IMUParser::IMUParser() 
    : rx_index_(0),
      rx_buffer_(MAX_BUFFER_SIZE, 0),
      frame_length_(0),
      last_byte_(0),
      parsing_state_(false),
      frame_buffer_(MAX_BUFFER_SIZE, 0),
      imu_ready_(false),
      ahrs_ready_(false) {
}


void IMUParser::reset_info() {
    stats_ = ParserInfo_t();
}


void IMUParser::reset() {
    rx_index_ = 0;
    frame_length_ = 0;
    last_byte_ = 0;
    parsing_state_ = false;
    imu_ready_ = false;
    ahrs_ready_ = false;
    imu_data_ = IMUData_t();
    ahrs_data_ = AHRSData_t();
    stats_ = ParserInfo_t();
    std::fill(rx_buffer_.begin(), rx_buffer_.end(), 0);
    std::fill(frame_buffer_.begin(), frame_buffer_.end(), 0);
}



/* 数据入口 */
void IMUParser::feed(const uint8_t* data, int len) {
    if (data == nullptr || len <= 0) {
        return;
    }
    
    stats_.total_bytes += len;
    
    for (int i = 0; i < len; i++) {
        uint8_t cur_byte = data[i];
        
        /* 上一字节是帧尾当前字节是帧头，标记并开始接受一帧数据 */
        if (last_byte_ == FRAME_END && cur_byte == FRAME_HEAD) {
            rx_index_ = 0;
            rx_buffer_[rx_index_++] = cur_byte;  // 接受数据到缓冲区
            parsing_state_ = true;
        }
        /* 正在接受一帧数据 */
        else if (parsing_state_ && rx_index_ < MAX_BUFFER_SIZE - 1) {
            rx_buffer_[rx_index_++] = cur_byte;
            
            if (rx_index_ == 2) {
                uint8_t frame_type = rx_buffer_[1];
                
                switch (frame_type) {
                    case TYPE_IMU:  // IMU帧
                        frame_length_ = IMU_FRAME_SIZE;
                        payload_length_ = IMU_LEN;
                        break;
                    case TYPE_AHRS:  // AHRS帧
                        frame_length_ = AHRS_FRAME_SIZE;
                        payload_length_ = AHRS_LEN;
                        break;
                    case TYPE_UNKNOWN:  // 未知帧
                        parsing_state_ = false;
                        rx_index_ = 0;
                        break;
                    default:  // 错误的类型
                        parsing_state_ = false;
                        rx_index_ = 0;
                        stats_.error_frames++;
                        std::cerr << "[PROTOCOL WARNING] Unknown frame type: 0x" 
                                  << std::hex << (int)frame_type << std::dec << std::endl;
                        break;
                }
            }

            /* 进行CRC8校验 */
            if(rx_index_ == 5){
                std::vector<uint8_t> crc8_data(rx_buffer_.begin(), rx_buffer_.begin() + 4);
                uint8_t calculated_crc8 = crc8_table(crc8_data);
                uint8_t received_crc8 = rx_buffer_[4];

                if (calculated_crc8 != received_crc8) {
                    parsing_state_ = false;
                    rx_index_ = 0;
                    stats_.error_frames++;
                    std::cerr << "[PROTOCOL ERROR] CRC8 mismatch: calculated=0x" 
                              << std::hex << (int)calculated_crc8 
                              << ", received=0x" << (int)received_crc8 
                              << std::dec << std::endl;
                }           
            }

            /* 进行CRC16校验 */
            if (frame_length_ > 0 && rx_index_ == frame_length_ - 1) {
                std::vector<uint8_t> crc16_data(rx_buffer_.begin() + 7, rx_buffer_.begin() + 7 + payload_length_);
                uint16_t calculated_crc16 = crc16_table(crc16_data);
                uint16_t received_crc16 = rx_buffer_[5] << 8 | rx_buffer_[6];

                if (calculated_crc16 != received_crc16) {
                    parsing_state_ = false;
                    rx_index_ = 0;
                    stats_.error_frames++;
                    std::cerr << "[PROTOCOL ERROR] CRC16 mismatch: calculated=0x" 
                              << std::hex << (int)calculated_crc16 
                              << ", received=0x" << (int)received_crc16 
                              << std::dec << std::endl;
                }  
            }


            /* 接受到帧尾 */
            if (frame_length_ > 0 && rx_index_ >= frame_length_) {
                if (rx_buffer_[rx_index_ - 1] == FRAME_END) {
                    std::memcpy(frame_buffer_.data(), rx_buffer_.data(), rx_index_);
                    
                    switch (frame_buffer_[1]) {
                        case TYPE_IMU:
                            parse_imu_frame(frame_buffer_.data());
                            break;
                        case TYPE_AHRS:
                            parse_ahrs_frame(frame_buffer_.data());
                            break;
                    }
                    
                    stats_.total_frames++;
                } else {
                    stats_.error_frames++;
                    std::cerr << "[PROTOCOL ERROR] Frame end marker mismatch" << std::endl;
                }
                
                parsing_state_ = false;
                rx_index_ = 0;
                frame_length_ = 0;
            }
        }
        last_byte_ = cur_byte;
    }
}


/* 解读一帧的数据 */
bool IMUParser::parse_imu_frame(const uint8_t *data) {
    if (data[1] != TYPE_IMU) {
        return false;
    }
    
    imu_data_.gyroscope_x = data_to_float(data[7], data[8], data[9], data[10]);
    imu_data_.gyroscope_y = data_to_float(data[11], data[12], data[13], data[14]);
    imu_data_.gyroscope_z = data_to_float(data[15], data[16], data[17], data[18]);
    
    imu_data_.accelerometer_x = data_to_float(data[19], data[20], data[21], data[22]);
    imu_data_.accelerometer_y = data_to_float(data[23], data[24], data[25], data[26]);
    imu_data_.accelerometer_z = data_to_float(data[27], data[28], data[29], data[30]);
    
    imu_data_.magnetometer_x = data_to_float(data[31], data[32], data[33], data[34]);
    imu_data_.magnetometer_y = data_to_float(data[35], data[36], data[37], data[38]);
    imu_data_.magnetometer_z = data_to_float(data[39], data[40], data[41], data[42]);
    
    imu_data_.imu_temperature = data_to_float(data[43], data[44], data[45], data[46]);
    imu_data_.pressure = data_to_float(data[47], data[48], data[49], data[50]);
    imu_data_.pressure_temperature = data_to_float(data[51], data[52], data[53], data[54]);
    
    // imu_data_.timestamp = data_to_u32(data[55], data[56], data[57], data[58]);
    
    imu_ready_ = true;
    stats_.imu_frames++;
    

    if (imu_callback_) {
        imu_callback_(imu_data_);
    }
    
    return true;
}


bool IMUParser::parse_ahrs_frame(const uint8_t* data) {
    if (data[1] != TYPE_AHRS) {
        return false;
    }

    ahrs_data_.roll_speed    = data_to_float(data[7], data[8], data[9], data[10]);
    ahrs_data_.pitch_speed   = data_to_float(data[11], data[12], data[13], data[14]);
    ahrs_data_.heading_speed = -data_to_float(data[15], data[16], data[17], data[18]);

    double compensated_roll =
        static_cast<double>(data_to_float(data[19], data[20], data[21], data[22])) - kPi;
    const double compensated_heading =
        -static_cast<double>(data_to_float(data[27], data[28], data[29], data[30]));

    ahrs_data_.roll     = static_cast<float>(wrap_to_pi(compensated_roll));
    ahrs_data_.pitch    = -data_to_float(data[23], data[24], data[25], data[26]);
    ahrs_data_.heading  = static_cast<float>(wrap_to_pi(compensated_heading));

    ahrs_data_.qw = data_to_float(data[31], data[32], data[33], data[34]);
    ahrs_data_.qx = data_to_float(data[35], data[36], data[37], data[38]);
    ahrs_data_.qy = data_to_float(data[39], data[40], data[41], data[42]);
    ahrs_data_.qz = data_to_float(data[43], data[44], data[45], data[46]);

    ahrs_data_.projected_gravity_valid =
        compute_projected_gravity(ahrs_data_.roll,
                                  ahrs_data_.pitch,
                                  ahrs_data_.projected_gravity_x,
                                  ahrs_data_.projected_gravity_y,
                                  ahrs_data_.projected_gravity_z);

    ahrs_data_.timestamp =
        data_to_u64(data[47], data[48], data[49], data[50],
                    data[51], data[52], data[53], data[54]);

    ahrs_ready_ = true;
    stats_.ahrs_frames++;

    if (ahrs_callback_) {
        ahrs_callback_(ahrs_data_);
    }

    return true;
}


/* 暴露给外部获取解析好数据的接口 */
bool IMUParser::get_imu_data(IMUData_t& imu) {
    if (imu_ready_) {
        imu = imu_data_;
        imu_ready_ = false;
        return true;
    }
    return false;
}


bool IMUParser::get_ahrs_data(AHRSData_t& ahrs) {
    if (ahrs_ready_) {
        ahrs = ahrs_data_;
        ahrs_ready_ = false;
        return true;
    }
    return false;
}


void IMUParser::print_imu_data(const IMUData_t& imu) {
    std::cout << "========== IMU Data ==========" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Gyroscope (rad/s): X=" << imu.gyroscope_x 
              << ", Y=" << imu.gyroscope_y 
              << ", Z=" << imu.gyroscope_z << std::endl;
    std::cout << "Accelerometer (m/s²): X=" << imu.accelerometer_x 
              << ", Y=" << imu.accelerometer_y 
              << ", Z=" << imu.accelerometer_z << std::endl;
    std::cout << "Magnetometer (mG): X=" << imu.magnetometer_x 
              << ", Y=" << imu.magnetometer_y 
              << ", Z=" << imu.magnetometer_z << std::endl;
    std::cout << "IMU Temperature (°C): " << imu.imu_temperature << std::endl;
    std::cout << "Pressure (Pa): " << imu.pressure << std::endl;
    std::cout << "Pressure Temperature (°C): " << imu.pressure_temperature << std::endl;
    std::cout << "Timestamp: " << imu.timestamp << " us" << std::endl;
    std::cout << "==============================" << std::endl << std::endl;
}


void IMUParser::print_ahrs_data(const AHRSData_t& ahrs) {
    std::cout << "========= AHRS Data =========" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Roll/Pitch/Heading Speed (rad/s): ["
              << ahrs.roll_speed << ", "
              << ahrs.pitch_speed << ", "
              << ahrs.heading_speed << "]" << std::endl;
    std::cout << "Roll/Pitch/Heading (rad): ["
              << ahrs.roll << ", "
              << ahrs.pitch << ", "
              << ahrs.heading << "]" << std::endl;
    std::cout << "Quaternion [wxyz]: ["
              << ahrs.qw << ", "
              << ahrs.qx << ", "
              << ahrs.qy << ", "
              << ahrs.qz << "]" << std::endl;
    std::cout << "Projected gravity: ["
              << ahrs.projected_gravity_x << ", "
              << ahrs.projected_gravity_y << ", "
              << ahrs.projected_gravity_z << "]"
              << " valid=" << (ahrs.projected_gravity_valid ? "true" : "false")
              << std::endl;
    std::cout << "Timestamp: " << ahrs.timestamp << " us" << std::endl;
    std::cout << "=============================" << std::endl << std::endl;
}


float IMUParser::data_to_float(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4) {
    union {
        uint8_t bytes[4];
        float value;
    } converter;
    
    converter.bytes[0] = d1;
    converter.bytes[1] = d2;
    converter.bytes[2] = d3;
    converter.bytes[3] = d4;
    
    return converter.value;
}


double IMUParser::data_to_double(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4,
                                  uint8_t d5, uint8_t d6, uint8_t d7, uint8_t d8) {
    union {
        uint8_t bytes[8];
        double value;
    } converter;
    
    converter.bytes[0] = d1;
    converter.bytes[1] = d2;
    converter.bytes[2] = d3;
    converter.bytes[3] = d4;
    converter.bytes[4] = d5;
    converter.bytes[5] = d6;
    converter.bytes[6] = d7;
    converter.bytes[7] = d8;
    
    return converter.value;
}


uint32_t IMUParser::data_to_u32(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4) {
    return ((uint32_t)d4 << 24) | ((uint32_t)d3 << 16) | 
           ((uint32_t)d2 << 8) | (uint32_t)d1;
}


uint64_t IMUParser::data_to_u64(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4,
                                 uint8_t d5, uint8_t d6, uint8_t d7, uint8_t d8) {
    return ((uint64_t)d8 << 56) | ((uint64_t)d7 << 48) | 
           ((uint64_t)d6 << 40) | ((uint64_t)d5 << 32) |
           ((uint64_t)d4 << 24) | ((uint64_t)d3 << 16) | 
           ((uint64_t)d2 << 8) | (uint64_t)d1;
}


uint8_t IMUParser::crc8_table(const std::vector<uint8_t>& data) {
    uint8_t crc8 = 0x00;
    for (uint8_t value : data) {
        crc8 = CRC8Table[crc8 ^ value];
    }
    return crc8;
}

uint16_t IMUParser::crc16_table(const std::vector<uint8_t>& data)
{
    uint16_t crc16 = 0;
    for (uint8_t value : data) 
    {
        crc16 = CRC16Table[((crc16 >> 8) ^ value) & 0xff] ^ (crc16 << 8);
    }
    return crc16;
}

}
