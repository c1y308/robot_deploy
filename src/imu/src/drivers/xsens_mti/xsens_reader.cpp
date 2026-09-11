#include "driver/xsens_mti/xsens_reader.hpp"

#include "driver/socket_can_config.hpp"

#include <iomanip>
#include <iostream>
#include <linux/can.h>

namespace imu {

XsensMtiCanReader::XsensMtiCanReader()
    : can_port_(std::make_unique<SocketCanPort>()),
      parser_(std::make_unique<XsensMtiCanParser>()),
      running_(false),
      start_time_(std::chrono::steady_clock::now())
{
}

XsensMtiCanReader::~XsensMtiCanReader()
{
    stop();
}

bool XsensMtiCanReader::start(const imu_base::ReaderConfig& config)
{
    if (running_.load()) {
        return true;
    }

    config_ = config;
    start_time_ = std::chrono::steady_clock::now();
    print_configuration();

    parser_->reset();
    if (config_.configure_can &&
        !configure_socket_can_interface(config_.device, config_.can_bitrate)) {
        return false;
    }
    if (!can_port_->open(config_.device)) {
        return false;
    }

    running_.store(true);
    std::string thread_error;
    if (!robot_base::start_configured_thread(
            worker_thread_, "imu_can_rx", config_.thread_options,
            [this] { read_loop(); }, thread_error)) {
        running_.store(false);
        can_port_->close();
        std::cerr << "[ERROR] Xsens MTi CAN reader thread setup failed: "
                  << thread_error << std::endl;
        return false;
    }
    std::cout << "[INFO] Xsens MTi CAN reader thread started." << std::endl;
    return true;
}

void XsensMtiCanReader::read_loop()
{
    std::cout << "[INFO] Starting Xsens MTi CAN data acquisition..."
              << std::endl;

    while (running_.load()) {
        const int ready = can_port_->wait_readable(10);
        if (ready < 0) {
            running_.store(false);
            break;
        }
        if (ready == 0) {
            continue;
        }

        while (running_.load()) {
            can_frame frame = {};
            std::int64_t receive_timestamp_ns = 0;
            const int read_result =
                can_port_->read_nonblocking(frame, &receive_timestamp_ns);
            if (read_result < 0) {
                running_.store(false);
                break;
            }
            if (read_result == 0) {
                break;
            }

            if ((frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) == 0) {
                parser_->feed(frame.can_id & CAN_SFF_MASK,
                              frame.data,
                              frame.len,
                              receive_timestamp_ns);

                imu_base::AHRSData ahrs_data;
                if (parser_->get_ahrs_data(ahrs_data) && config_.print_ahrs) {
                    XsensMtiCanParser::print_ahrs_data(ahrs_data);
                }
            }
        }
    }

    std::cout << "[INFO] Final Xsens MTi CAN statistics:" << std::endl;
    print_statistics();
}

void XsensMtiCanReader::stop()
{
    const bool was_running = running_.exchange(false);
    if (was_running) {
        std::cout << "[INFO] Stopping Xsens MTi CAN reader..." << std::endl;
    }
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
    if (can_port_->is_open()) {
        can_port_->close();
    }
}

const imu_base::ParserInfo& XsensMtiCanReader::get_info() const
{
    return parser_->get_info();
}

void XsensMtiCanReader::set_imu_callback(
    imu_base::IMUReaderBase::IMUCallback callback)
{
    parser_->set_imu_callback(callback);
}

void XsensMtiCanReader::set_ahrs_callback(
    imu_base::IMUReaderBase::AHRSCallback callback)
{
    parser_->set_ahrs_callback(callback);
}

void XsensMtiCanReader::print_configuration() const
{
    std::cout << "========================================" << std::endl;
    std::cout << "  Xsens MTi CAN Reader" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "CAN Interface: " << config_.device << std::endl;
    std::cout << "Configure CAN: "
              << (config_.configure_can ? "Yes" : "No") << std::endl;
    if (config_.configure_can) {
        std::cout << "CAN Bitrate: " << config_.can_bitrate << std::endl;
    }
    std::cout << "Print AHRS: " << (config_.print_ahrs ? "Yes" : "No")
              << std::endl;
    std::cout << "========================================" << std::endl << std::endl;
}

void XsensMtiCanReader::print_statistics() const
{
    const auto& stats = parser_->get_info();
    const auto runtime = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time_).count();

    std::cout << "--- Statistics ---" << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Runtime: " << runtime << " s" << std::endl;
    std::cout << "Total bytes: " << stats.total_bytes << std::endl;
    std::cout << "Total frames: " << stats.total_frames << std::endl;
    std::cout << "AHRS frames: " << stats.ahrs_frames << std::endl;
    std::cout << "Error frames: " << stats.error_frames << std::endl;
    std::cout << "------------------" << std::endl << std::endl;
}

}  // namespace imu
