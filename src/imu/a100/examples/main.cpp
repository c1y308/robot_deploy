#include "imu_reader.hpp"
#include <iostream>
#include <csignal>
#include <getopt.h>
#include <cstdlib>

using namespace imu;

namespace {
    IMUReader* g_reader = nullptr;
    
    void signal_handler(int sig) {
        (void)sig;
        if (g_reader) {
            g_reader->stop();
        }
        std::cout << "\n[INFO] Received signal, exiting..." << std::endl;
    }
    
    void print_usage(const char* prog_name) {
        std::cout << "Usage: " << prog_name << " [options]" << std::endl;
        std::cout << "Options:" << std::endl;
        std::cout << "  -d, --device <path>    Serial device path (default: /dev/ttyUSB0)" << std::endl;
        std::cout << "  -b, --baud <rate>      Baud rate (default: 921600)" << std::endl;
        std::cout << "  -p, --print-imu        Print IMU data" << std::endl;
        std::cout << "  -s, --stats            Print statistics" << std::endl;
        std::cout << "  -h, --help             Show this help message" << std::endl;
        std::cout << std::endl;
        std::cout << "Example:" << std::endl;
        std::cout << "  " << prog_name << " -d /dev/ttyUSB0 -b 921600 -p" << std::endl;
    }
}

int main(int argc, char* argv[]) {
    Config_t config;
    
    static struct option long_options[] = {
        {"device",     required_argument, 0, 'd'},
        {"baud",       required_argument, 0, 'b'},
        {"print-imu",  no_argument,       0, 'p'},
        {"stats",      no_argument,       0, 's'},
        {"help",       no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };
    
    int opt;
    while ((opt = getopt_long(argc, argv, "d:b:psh", long_options, nullptr)) != -1) {
        switch (opt) {
            case 'd':
                config.device = optarg;
                break;
            case 'b':
                config.baudrate = std::atoi(optarg);
                break;
            case 'p':
                config.print_imu = true;
                break;
            case 's':
                config.print_stats = true;
                break;
            case 'h':
            default:
                print_usage(argv[0]);
                return (opt == 'h') ? 0 : 1;
        }
    }
    
    if (!config.print_imu && !config.print_stats) {
        config.print_imu = true;
    }
    
    IMUReader reader;
    g_reader = &reader;
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    if (!reader.initialize(config)) {
        std::cerr << "[ERROR] Failed to initialize IMU reader" << std::endl;
        return -1;
    }
    
    reader.run();
    
    std::cout << "[INFO] Program exited normally." << std::endl;
    return 0;
}
