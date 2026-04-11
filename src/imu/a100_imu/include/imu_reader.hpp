#ifndef __IMU_READER_HPP__
#define __IMU_READER_HPP__

#include "serial_port.hpp"
#include "imu_parser.hpp"
#include <string>
#include <atomic>
#include <memory>

namespace imu {

struct Config_t {
    std::string device;
    int baudrate;
    bool print_imu;
    bool print_stats;
    
    Config_t() : device("/dev/ttyS1"), 
                 baudrate(921600),
                 print_imu(false),
                 print_stats(false) {}
};

class IMUReader {
public:
    IMUReader();
    ~IMUReader();
    
    IMUReader(const IMUReader&) = delete;
    IMUReader& operator=(const IMUReader&) = delete;
    
    bool initialize(const Config_t& config);
    void run();
    void stop();
    
    const ParserInfo_t& get_info() const;
    
    void set_imu_callback(IMUParser::IMUCallback_t callback);

private:
    void print_configuration() const;
    void print_statistics() const;
    
    Config_t config_;
    std::unique_ptr<SerialPort> serial_port_;
    std::unique_ptr<IMUParser> parser_;
    std::atomic<bool> running_;
};

}

#endif
