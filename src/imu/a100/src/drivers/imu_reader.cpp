#include "imu_reader.hpp"
#include <iostream>
#include <csignal>
#include <unistd.h>

namespace imu {

IMUReader::IMUReader() 
    : serial_port_(std::make_unique<SerialPort>()),
      parser_(std::make_unique<IMUParser>()),
      running_(false) {
}


IMUReader::~IMUReader() {
    stop();
}


bool IMUReader::initialize(const Config_t& config) {
    config_ = config;
    
    print_configuration();
    
    if (!serial_port_->open(config_.device, config_.baudrate)) {
        return false;
    }
    
    running_ = true;
    return true;
}


void IMUReader::run() {
    if (!running_) {
        std::cerr << "[ERROR] IMUReader not initialized" << std::endl;
        return;
    }
    
    constexpr size_t READ_BUFFER_SIZE = 256;
    uint8_t read_buffer[READ_BUFFER_SIZE];
    
    std::cout << "[INFO] Starting IMU data acquisition..." << std::endl;
    std::cout << "[INFO] Press Ctrl+C to exit." << std::endl << std::endl;
    
    uint64_t last_stats_frame = 0;
    
    while (running_) {
        int bytes_read = serial_port_->read(read_buffer, READ_BUFFER_SIZE);
        
        if (bytes_read > 0) {
            parser_->feed(read_buffer, bytes_read);
            
            IMUData_t imu_data;
            AHRSData_t ahrs_data;
            
            if (config_.print_imu && parser_->get_imu_data(imu_data)) {
                IMUParser::print_imu_data(imu_data);
            }
            if (config_.print_ahrs && parser_->get_ahrs_data(ahrs_data)) {
                IMUParser::print_ahrs_data(ahrs_data);
            }
        }
        
        if (config_.print_stats) {
            const auto& stats = parser_->get_info();
            if (stats.total_frames > 0 && 
                stats.total_frames % 100 == 0 &&
                stats.total_frames != last_stats_frame) {
                print_statistics();
                last_stats_frame = stats.total_frames;
            }
        }
        
        usleep(1000);
    }
    
    std::cout << std::endl << "[INFO] Final Statistics:" << std::endl;
    print_statistics();
}


void IMUReader::stop() {
    if (running_) {
        running_ = false;
        std::cout << "[INFO] Stopping IMU reader..." << std::endl;
    }
}


const ParserInfo_t& IMUReader::get_info() const {
    return parser_->get_info();
}


void IMUReader::set_imu_callback(IMUParser::IMUCallback_t callback) {
    parser_->set_imu_callback(callback);
}


void IMUReader::set_ahrs_callback(IMUParser::AHRSCallback_t callback) {
    parser_->set_ahrs_callback(callback);
}


void IMUReader::print_configuration() const {
    std::cout << "========================================" << std::endl;
    std::cout << "  RK3588 IMU Reader (C++ Version)" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Serial Device: " << config_.device << std::endl;
    std::cout << "Baud Rate: " << config_.baudrate << std::endl;
    std::cout << "Print IMU: " << (config_.print_imu ? "Yes" : "No") << std::endl;
    std::cout << "Print AHRS: " << (config_.print_ahrs ? "Yes" : "No") << std::endl;
    std::cout << "Print Stats: " << (config_.print_stats ? "Yes" : "No") << std::endl;
    std::cout << "========================================" << std::endl << std::endl;
}


void IMUReader::print_statistics() const {
    const auto& stats = parser_->get_info();
    std::cout << "--- Statistics ---" << std::endl;
    std::cout << "Total bytes: " << stats.total_bytes << std::endl;
    std::cout << "Total frames: " << stats.total_frames << std::endl;
    std::cout << "IMU frames: " << stats.imu_frames << std::endl;
    std::cout << "AHRS frames: " << stats.ahrs_frames << std::endl;
    std::cout << "Error frames: " << stats.error_frames << std::endl;
    std::cout << "------------------" << std::endl << std::endl;
}

}
