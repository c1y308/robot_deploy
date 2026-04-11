#ifndef __IMU_PARSER_HPP__
#define __IMU_PARSER_HPP__

#include "types.hpp"
#include <functional>
#include <vector>

namespace imu {

constexpr size_t MAX_BUFFER_SIZE = 256;

class IMUParser {
public:
    using IMUCallback_t = std::function<void(const IMUData_t&)>;
    
    IMUParser();
    ~IMUParser() = default;
    
    void feed(const uint8_t* data, int len);
    
    bool get_imu_data(IMUData_t& imu);
    
    /* 设置回调函数 */
    void set_imu_callback(IMUCallback_t callback) { imu_callback_ = callback; }
    
    const ParserInfo_t& get_info() const { return stats_; }
    void reset_info();
    
    static void print_imu_data(const IMUData_t& imu);

private:
    float  data_to_float(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4);
    double data_to_double(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4,
                          uint8_t d5, uint8_t d6, uint8_t d7, uint8_t d8);
    uint32_t data_to_u32(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4);
    uint64_t data_to_u64(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4,
                         uint8_t d5, uint8_t d6, uint8_t d7, uint8_t d8);
    bool parse_imu_frame(const uint8_t* data);
    
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
    bool imu_ready_;
    
    ParserInfo_t stats_;
    
    IMUCallback_t imu_callback_;
};

}

#endif
