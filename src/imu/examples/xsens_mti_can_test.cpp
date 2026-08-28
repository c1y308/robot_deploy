#include "driver/xsens_mti/xsens_reader.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <thread>

namespace {

std::atomic<bool> g_running{true};

void signal_handler(int)
{
    g_running.store(false);
}

struct SharedState {
    std::mutex mutex;
    imu_base::AHRSData latest_ahrs;
    bool has_ahrs = false;
    std::uint64_t ahrs_updates = 0;
};

}  // namespace

int main()
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    imu_base::ReaderConfig cfg;
    cfg.type = imu_base::ReaderType::XSENS_MTI_CAN;
    cfg.device = "can0";
    cfg.print_ahrs = true;

    SharedState state;
    imu::XsensMtiCanReader reader;
    reader.set_ahrs_callback([&state](const imu_base::AHRSData& data) {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.latest_ahrs = data;
        state.has_ahrs = true;
        ++state.ahrs_updates;
    });

    if (!reader.start(cfg)) {
        std::cerr << "[XSENS_MTI_CAN_TEST] Failed to start IMU reader.\n";
        return -1;
    }

    std::cout << "[XSENS_MTI_CAN_TEST] Running. Press Ctrl+C to stop.\n";
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    reader.stop();
    std::cout << "[XSENS_MTI_CAN_TEST] IMU reader stopped.\n";
    return 0;
}
