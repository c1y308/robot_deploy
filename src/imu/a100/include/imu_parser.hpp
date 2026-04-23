#ifndef __IMU_PARSER_HPP__
#define __IMU_PARSER_HPP__

#include "types.hpp"
#include <functional>
#include <vector>

namespace imu {

constexpr size_t MAX_BUFFER_SIZE = 512;

class IMUParser {
public:
    using IMUCallback_t = std::function<void(const IMUData_t&)>;
    using AHRSCallback_t = std::function<void(const AHRSData_t&)>;
    
    IMUParser();
    ~IMUParser() = default;
    
    void feed(const uint8_t* data, int len);
    void reset();
    
    bool get_imu_data(IMUData_t& imu);
    bool get_ahrs_data(AHRSData_t& ahrs);
    
    /* 设置回调函数 */
    void set_imu_callback(IMUCallback_t callback) { imu_callback_ = callback; }
    void set_ahrs_callback(AHRSCallback_t callback) { ahrs_callback_ = callback; }
    
    const ParserInfo_t& get_info() const { return stats_; }
    void reset_info();
    
    static void print_imu_data(const IMUData_t& imu);
    static void print_ahrs_data(const AHRSData_t& ahrs);

private:
    uint8_t  CRC8_Table(const std::vector<uint8_t>& data);
    uint16_t CRC16_Table(const std::vector<uint8_t>& data);

    bool parse_imu_frame(const uint8_t* data);
    bool parse_ahrs_frame(const uint8_t* data);

    float  data_to_float(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4);
    double data_to_double(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4,
                          uint8_t d5, uint8_t d6, uint8_t d7, uint8_t d8);
    uint32_t data_to_u32(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4);
    uint64_t data_to_u64(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4,
                         uint8_t d5, uint8_t d6, uint8_t d7, uint8_t d8);

    
    /* 接受一帧数据的缓冲区 */
    size_t rx_index_;
    std::vector<uint8_t> rx_buffer_;
    size_t frame_length_;
    uint8_t last_byte_;
    bool parsing_state_;

    /* 存储正确数据的缓冲区 */
    std::vector<uint8_t> frame_buffer_;
    
    /* 存储解析好的数据 */
    IMUData_t imu_data_;
    AHRSData_t ahrs_data_;
    bool imu_ready_;
    bool ahrs_ready_;
    
    ParserInfo_t stats_;
    
    IMUCallback_t imu_callback_;
    AHRSCallback_t ahrs_callback_;
};


static const uint8_t CRC8Table[256] = {
    0,   94, 188, 226,  97,  63, 221, 131, 194, 156, 126,  32, 163, 253,  31,  65,
  157, 195,  33, 127, 252, 162,  64,  30,  95,   1, 227, 189,  62,  96, 130, 220,
   35, 125, 159, 193,  66,  28, 254, 160, 225, 191,  93,   3, 128, 222,  60,  98,
  190, 224,   2,  92, 223, 129,  99,  61, 124,  34, 192, 158,  29,  67, 161, 255,
   70,  24, 250, 164,  39, 121, 155, 197, 132, 218,  56, 102, 229, 187,  89,   7,
  219, 133, 103,  57, 186, 228,   6,  88,  25,  71, 165, 251, 120,  38, 196, 154,
  101,  59, 217, 135,   4,  90, 184, 230, 167, 249,  27,  69, 198, 152, 122,  36,
  248, 166,  68,  26, 153, 199,  37, 123,  58, 100, 134, 216,  91,   5, 231, 185,
  140, 210,  48, 110, 237, 179,  81,  15,  78,  16, 242, 172,  47, 113, 147, 205,
   17,  79, 173, 243, 112,  46, 204, 146, 211, 141, 111,  49, 178, 236,  14,  80,
  175, 241,  19,  77, 206, 144, 114,  44, 109,  51, 209, 143,  12,  82, 176, 238,
   50, 108, 142, 208,  83,  13, 239, 177, 240, 174,  76,  18, 145, 207,  45, 115,
  202, 148, 118,  40, 171, 245,  23,  73,   8,  86, 180, 234, 105,  55, 213, 139,
   87,   9, 235, 181,  54, 104, 138, 212, 149, 203,  41, 119, 244, 170,  72,  22,
  233, 183,  85,  11, 136, 214,  52, 106,  43, 117, 151, 201,  74,  20, 246, 168,
  116,  42, 200, 150,  21,  75, 169, 247, 182, 232,  10,  84, 215, 137, 107,  53
};



}

#endif
