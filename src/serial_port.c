#include "serial_port.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>

int baudrate_to_constant(int baudrate)
{
    switch (baudrate) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B921600;
    }
}

int serial_open(SerialPort_t *serial, const char *device, int baudrate)
{
    if (serial == NULL || device == NULL) {
        fprintf(stderr, "Error: Invalid parameters\n");
        return -1;
    }

    memset(serial, 0, sizeof(SerialPort_t));
    strncpy(serial->device_path, device, sizeof(serial->device_path) - 1);
    serial->baudrate = baudrate;
    serial->databits = DEFAULT_DATABITS;
    serial->stopbits = DEFAULT_STOPBITS;
    serial->parity = DEFAULT_PARITY;

    serial->fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial->fd < 0) {
        fprintf(stderr, "Error: Cannot open serial port %s: %s\n", 
                device, strerror(errno));
        return -1;
    }

    if (tcgetattr(serial->fd, &serial->oldtio) != 0) {
        fprintf(stderr, "Error: tcgetattr failed: %s\n", strerror(errno));
        close(serial->fd);
        return -1;
    }

    memset(&serial->newtio, 0, sizeof(serial->newtio));

    serial->newtio.c_cflag |= CLOCAL | CREAD;
    serial->newtio.c_cflag |= baudrate_to_constant(baudrate);

    serial->newtio.c_cflag &= ~CSIZE;
    serial->newtio.c_cflag |= CS8;

    serial->newtio.c_cflag &= ~PARENB;
    serial->newtio.c_cflag &= ~CSTOPB;

    serial->newtio.c_cflag &= ~CRTSCTS;

    serial->newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    serial->newtio.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | INLCR | ICRNL);

    serial->newtio.c_oflag &= ~OPOST;

    serial->newtio.c_cc[VTIME] = 0;
    serial->newtio.c_cc[VMIN] = 0;

    tcflush(serial->fd, TCIOFLUSH);

    if (tcsetattr(serial->fd, TCSANOW, &serial->newtio) != 0) {
        fprintf(stderr, "Error: tcsetattr failed: %s\n", strerror(errno));
        close(serial->fd);
        return -1;
    }

    printf("Serial port opened: %s @ %d bps\n", device, baudrate);
    return 0;
}

int serial_close(SerialPort_t *serial)
{
    if (serial == NULL || serial->fd < 0) {
        return -1;
    }

    tcsetattr(serial->fd, TCSANOW, &serial->oldtio);
    close(serial->fd);
    serial->fd = -1;
    
    printf("Serial port closed\n");
    return 0;
}

int serial_read(SerialPort_t *serial, uint8_t *buffer, int max_len)
{
    if (serial == NULL || buffer == NULL || serial->fd < 0) {
        return -1;
    }

    fd_set read_fds;
    struct timeval timeout;
    
    FD_ZERO(&read_fds);
    FD_SET(serial->fd, &read_fds);
    
    timeout.tv_sec = 0;
    timeout.tv_usec = 10000;

    int ret = select(serial->fd + 1, &read_fds, NULL, NULL, &timeout);
    if (ret < 0) {
        if (errno == EINTR) {
            return 0;
        }
        fprintf(stderr, "Error: select failed: %s\n", strerror(errno));
        return -1;
    }
    
    if (ret == 0) {
        return 0;
    }

    int bytes_read = read(serial->fd, buffer, max_len);
    if (bytes_read < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        fprintf(stderr, "Error: read failed: %s\n", strerror(errno));
        return -1;
    }

    return bytes_read;
}

int serial_write(SerialPort_t *serial, const uint8_t *data, int len)
{
    if (serial == NULL || data == NULL || serial->fd < 0) {
        return -1;
    }

    int bytes_written = write(serial->fd, data, len);
    if (bytes_written < 0) {
        fprintf(stderr, "Error: write failed: %s\n", strerror(errno));
        return -1;
    }

    tcdrain(serial->fd);
    return bytes_written;
}

int serial_set_baudrate(SerialPort_t *serial, int baudrate)
{
    if (serial == NULL || serial->fd < 0) {
        return -1;
    }

    cfsetispeed(&serial->newtio, baudrate_to_constant(baudrate));
    cfsetospeed(&serial->newtio, baudrate_to_constant(baudrate));

    if (tcsetattr(serial->fd, TCSANOW, &serial->newtio) != 0) {
        fprintf(stderr, "Error: tcsetattr failed: %s\n", strerror(errno));
        return -1;
    }

    serial->baudrate = baudrate;
    return 0;
}

void serial_flush(SerialPort_t *serial)
{
    if (serial != NULL && serial->fd >= 0) {
        tcflush(serial->fd, TCIOFLUSH);
    }
}
