#include "imu_reader.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <mutex>
#include <thread>

namespace {

std::atomic<bool> g_running{true};

void signal_handler(int) {
    g_running.store(false);
}

struct SharedState {
    std::mutex mutex;
    imu::IMUData_t  latest_imu;
    imu::AHRSData_t latest_ahrs;
    bool has_imu = false;
    bool has_ahrs = false;
    uint64_t imu_updates = 0;
    uint64_t ahrs_updates = 0;
};



void print_latest_summary(SharedState& state, const imu::ParserInfo_t& info) {
    std::lock_guard<std::mutex> lock(state.mutex);

    std::cout << "[A100_TEST] frames=" << info.total_frames
              << " imu=" << info.imu_frames
              << " ahrs=" << info.ahrs_frames
              << " err=" << info.error_frames << "\n";

    if (state.has_imu) {
        std::cout << "[A100_TEST] latest IMU updates=" << state.imu_updates << "\n";
        imu::IMUParser::print_imu_data(state.latest_imu);
    } else {
        std::cout << "  IMU waiting for data...\n";
    }

    if (state.has_ahrs) {
        std::cout << "[A100_TEST] latest AHRS updates=" << state.ahrs_updates << "\n";
        imu::IMUParser::print_ahrs_data(state.latest_ahrs);
    } else {
        std::cout << "  AHRS waiting for data...\n";
    }
}

}  // namespace

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    imu::Config_t cfg;
    SharedState state;


    imu::IMUReader reader;
    reader.set_imu_callback([&state](const imu::IMUData_t& data) {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.latest_imu = data;
        state.has_imu = true;
        ++state.imu_updates;
    });
    reader.set_ahrs_callback([&state](const imu::AHRSData_t& data) {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.latest_ahrs = data;
        state.has_ahrs = true;
        ++state.ahrs_updates;
    });

    if (!reader.start(cfg)) {
        std::cerr << "[A100_TEST] Failed to start IMU reader.\n";
        return -1;
    }

    std::cout << "[A100_TEST] Running. Press Ctrl+C to stop.\n";
    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        print_latest_summary(state, reader.get_info());
    }

    reader.stop();
    std::cout << "[A100_TEST] IMU reader stopped.\n";
    return 0;
}
