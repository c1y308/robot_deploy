#ifndef __SERIAL_PORT_HPP__
#define __SERIAL_PORT_HPP__

#include <string>
#include <cstdint>
#include <termios.h>

namespace imu {

class SerialPort {
public:
    SerialPort();
    ~SerialPort();
    
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    SerialPort(SerialPort&&) = delete;
    SerialPort& operator=(SerialPort&&) = delete;
    
    bool open(const std::string& device, int baudrate);
    void close();
    bool is_open() const;
    
    int read(uint8_t* buffer, int max_len);
    int write(const uint8_t* data, int len);
    
    bool set_baudrate(int baudrate);
    void flush();
    
    const std::string& get_device_path() const { return device_path_; }
    int get_baudrate() const { return baudrate_; }
    int get_file_descriptor() const { return fd_; }

private:
    int baudrate_to_constant(int baudrate);
    
    int fd_;  // 对应文件描述符
    std::string device_path_;
    int baudrate_;  // 波特率
    struct termios oldtio_;
    struct termios newtio_;
};

}

#endif
