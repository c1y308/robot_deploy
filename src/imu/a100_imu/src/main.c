#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>

#include "serial_port.h"
#include "imu_parser.h"
#include "data_types.h"

#define DEFAULT_SERIAL_DEVICE   "/dev/ttyS1"
#define DEFAULT_BAUDRATE        921600
#define READ_BUFFER_SIZE        256

static volatile int g_running = 1;

void signal_handler(int sig)
{
    (void)sig;
    g_running = 0;
    printf("\nReceived signal, exiting...\n");
}

void print_usage(const char *prog_name)
{
    printf("Usage: %s [options]\n", prog_name);
    printf("Options:\n");
    printf("  -d, --device <path>    Serial device path (default: %s)\n", DEFAULT_SERIAL_DEVICE);
    printf("  -b, --baud <rate>      Baud rate (default: %d)\n", DEFAULT_BAUDRATE);
    printf("  -p, --print-imu        Print IMU data\n");
    printf("  -a, --print-ahrs       Print AHRS data\n");
    printf("  -g, --print-insgps     Print INSGPS data\n");
    printf("  -s, --stats            Print statistics\n");
    printf("  -h, --help             Show this help message\n");
    printf("\nExample:\n");
    printf("  %s -d /dev/ttyS1 -b 921600 -p -a\n", prog_name);
}

int main(int argc, char *argv[])
{
    char *device = DEFAULT_SERIAL_DEVICE;
    int baudrate = DEFAULT_BAUDRATE;
    int print_imu = 0;
    int print_ahrs = 0;
    int print_insgps = 0;
    int print_stats = 0;
    
    static struct option long_options[] = {
        {"device",     required_argument, 0, 'd'},
        {"baud",       required_argument, 0, 'b'},
        {"print-imu",  no_argument,       0, 'p'},
        {"print-ahrs", no_argument,       0, 'a'},
        {"print-insgps", no_argument,     0, 'g'},
        {"stats",      no_argument,       0, 's'},
        {"help",       no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };
    
    int opt;
    while ((opt = getopt_long(argc, argv, "d:b:pagsh", long_options, NULL)) != -1) {
        switch (opt) {
            case 'd':
                device = optarg;
                break;
            case 'b':
                baudrate = atoi(optarg);
                break;
            case 'p':
                print_imu = 1;
                break;
            case 'a':
                print_ahrs = 1;
                break;
            case 'g':
                print_insgps = 1;
                break;
            case 's':
                print_stats = 1;
                break;
            case 'h':
            default:
                print_usage(argv[0]);
                return (opt == 'h') ? 0 : 1;
        }
    }
    
    if (!print_imu && !print_ahrs && !print_insgps && !print_stats) {
        print_imu = 1;
        print_ahrs = 1;
    }
    
    printf("========================================\n");
    printf("  RK3588 IMU Reader\n");
    printf("========================================\n");
    printf("Serial Device: %s\n", device);
    printf("Baud Rate: %d\n", baudrate);
    printf("Print IMU: %s\n", print_imu ? "Yes" : "No");
    printf("Print AHRS: %s\n", print_ahrs ? "Yes" : "No");
    printf("Print INSGPS: %s\n", print_insgps ? "Yes" : "No");
    printf("Print Stats: %s\n", print_stats ? "Yes" : "No");
    printf("========================================\n\n");
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    SerialPort_t serial;
    if (serial_open(&serial, device, baudrate) != 0) {
        fprintf(stderr, "Failed to open serial port: %s\n", device);
        fprintf(stderr, "\n[TODO] 请确认以下硬件配置:\n");
        fprintf(stderr, "  1. 惯导模块已正确连接到RK3588的串口\n");
        fprintf(stderr, "  2. 确认串口设备路径 (如 /dev/ttyS1, /dev/ttyUSB0 等)\n");
        fprintf(stderr, "  3. 确认波特率设置 (惯导模块默认: 921600)\n");
        fprintf(stderr, "  4. 检查用户是否有串口访问权限 (可能需要加入 dialout 组)\n");
        return -1;
    }
    
    IMUParser_t parser;
    imu_parser_init(&parser);
    
    uint8_t read_buffer[READ_BUFFER_SIZE];
    IMUData_Packet_t imu_data;
    AHRSData_Packet_t ahrs_data;
    INSGPSData_Packet_t insgps_data;
    
    printf("Starting IMU data acquisition...\n");
    printf("Press Ctrl+C to exit.\n\n");
    
    while (g_running) {
        int bytes_read = serial_read(&serial, read_buffer, READ_BUFFER_SIZE);
        
        if (bytes_read > 0) {
            imu_parser_feed(&parser, read_buffer, bytes_read);
            
            if (print_imu && imu_parser_get_imu(&parser, &imu_data)) {
                imu_print_data(&imu_data);
            }
            
            if (print_ahrs && imu_parser_get_ahrs(&parser, &ahrs_data)) {
                ahrs_print_data(&ahrs_data);
            }
            
            if (print_insgps && imu_parser_get_insgps(&parser, &insgps_data)) {
                insgps_print_data(&insgps_data);
            }
        }
        
        if (print_stats && parser.total_frames > 0 && 
            (parser.total_frames % 100 == 0)) {
            printf("--- Statistics ---\n");
            printf("Total bytes: %lu\n", parser.total_bytes);
            printf("Total frames: %lu\n", parser.total_frames);
            printf("IMU frames: %lu\n", parser.imu_frames);
            printf("AHRS frames: %lu\n", parser.ahrs_frames);
            printf("Error frames: %lu\n", parser.error_frames);
            printf("------------------\n\n");
        }
        
        usleep(1000);
    }
    
    printf("\nFinal Statistics:\n");
    printf("Total bytes received: %lu\n", parser.total_bytes);
    printf("Total frames: %lu\n", parser.total_frames);
    printf("IMU frames: %lu\n", parser.imu_frames);
    printf("AHRS frames: %lu\n", parser.ahrs_frames);
    printf("Error frames: %lu\n", parser.error_frames);
    
    serial_close(&serial);
    
    printf("Program exited normally.\n");
    return 0;
}
