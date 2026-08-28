#ifndef IMU_DRIVER_A100_READER_HPP_
#define IMU_DRIVER_A100_READER_HPP_

#include "imu_base/imu_base.hpp"
#include "driver/serial_port.hpp"
#include "protocol/a100/imu_parser.hpp"
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

namespace imu {

class IMUReader : public imu_base::IMUReaderBase {
public:
    IMUReader();
    ~IMUReader() override;
    
    IMUReader(const IMUReader&) = delete;
    IMUReader& operator=(const IMUReader&) = delete;
    
    bool start(const Config_t& config) override;
    void stop() override;
    bool is_running() const override { return running_.load(); }
    
    const ParserInfo_t& get_info() const override;
    
    void set_imu_callback(imu_base::IMUReaderBase::IMUCallback callback) override;
    void set_ahrs_callback(imu_base::IMUReaderBase::AHRSCallback callback) override;

private:
    void read_loop();
    void print_configuration() const;
    void print_statistics() const;
    
    Config_t config_;
    std::unique_ptr<SerialPort> serial_port_;
    std::unique_ptr<IMUParser> parser_;
    std::thread worker_thread_;
    std::atomic<bool> running_;
    std::chrono::steady_clock::time_point start_time_;
    bool  has_yaw_offset_;
    float yaw_offset_;
};

}

#endif
