#ifndef __IMU_PARSER_H
#define __IMU_PARSER_H

#include "data_types.h"

#define MAX_BUFFER_SIZE 256

typedef struct {
    uint8_t rx_buffer[MAX_BUFFER_SIZE];
    uint8_t frame_buffer[MAX_BUFFER_SIZE];
    int rx_index;
    int frame_length;
    uint8_t last_byte;
    uint8_t parsing_state;
    
    IMUData_Packet_t imu_data;
    AHRSData_Packet_t ahrs_data;
    INSGPSData_Packet_t insgps_data;
    
    int imu_ready;
    int ahrs_ready;
    int insgps_ready;
    
    uint64_t total_bytes;
    uint64_t total_frames;
    uint64_t imu_frames;
    uint64_t ahrs_frames;
    uint64_t error_frames;
} IMUParser_t;

void imu_parser_init(IMUParser_t *parser);
int imu_parser_feed(IMUParser_t *parser, const uint8_t *data, int len);
int imu_parser_get_imu(IMUParser_t *parser, IMUData_Packet_t *imu);
int imu_parser_get_ahrs(IMUParser_t *parser, AHRSData_Packet_t *ahrs);
int imu_parser_get_insgps(IMUParser_t *parser, INSGPSData_Packet_t *insgps);

void imu_print_data(const IMUData_Packet_t *imu);
void ahrs_print_data(const AHRSData_Packet_t *ahrs);
void insgps_print_data(const INSGPSData_Packet_t *insgps);

float data_trans_float(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4);
double data_trans_double(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4,
                         uint8_t d5, uint8_t d6, uint8_t d7, uint8_t d8);
uint32_t data_trans_u32(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4);
uint64_t data_trans_u64(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4,
                        uint8_t d5, uint8_t d6, uint8_t d7, uint8_t d8);

#endif
