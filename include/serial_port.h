#ifndef __SERIAL_PORT_H
#define __SERIAL_PORT_H

#include <stdint.h>
#include <termios.h>

#define DEFAULT_BAUDRATE   B921600
#define DEFAULT_DATABITS   8
#define DEFAULT_STOPBITS   1
#define DEFAULT_PARITY     'N'

typedef struct {
    int fd;
    char device_path[256];
    int baudrate;
    int databits;
    int stopbits;
    char parity;
    struct termios oldtio;
    struct termios newtio;
} SerialPort_t;

int serial_open(SerialPort_t *serial, const char *device, int baudrate);
int serial_close(SerialPort_t *serial);
int serial_read(SerialPort_t *serial, uint8_t *buffer, int max_len);
int serial_write(SerialPort_t *serial, const uint8_t *data, int len);
int serial_set_baudrate(SerialPort_t *serial, int baudrate);
void serial_flush(SerialPort_t *serial);

int baudrate_to_constant(int baudrate);

#endif
