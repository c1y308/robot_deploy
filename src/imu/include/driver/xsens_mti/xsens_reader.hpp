#pragma once

#include "driver/socket_can_port.hpp"
#include "imu_base/imu_base.hpp"
#include "protocol/xsens_mti/can_parser.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

namespace imu {

class XsensMtiCanReader : public imu_base::IMUReaderBase {
public:
    XsensMtiCanReader();
    ~XsensMtiCanReader() override;

    XsensMtiCanReader(const XsensMtiCanReader&) = delete;
    XsensMtiCanReader& operator=(const XsensMtiCanReader&) = delete;

    bool start(const imu_base::ReaderConfig& config) override;
    void stop() override;
    bool is_running() const override { return running_.load(); }

    const imu_base::ParserInfo& get_info() const override;

    void set_imu_callback(imu_base::IMUReaderBase::IMUCallback callback) override;
    void set_ahrs_callback(imu_base::IMUReaderBase::AHRSCallback callback) override;

private:
    void read_loop();
    void print_configuration() const;
    void print_statistics() const;

    imu_base::ReaderConfig config_;
    std::unique_ptr<SocketCanPort> can_port_;
    std::unique_ptr<XsensMtiCanParser> parser_;
    std::thread worker_thread_;
    std::atomic<bool> running_;
    std::chrono::steady_clock::time_point start_time_;
};

}  // namespace imu
