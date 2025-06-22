#include "iai_pcon_driver/pcon_driver.hpp"

bool PconDriver::openPort(std::string port, int baud){
    fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        return false; // Failed to open port
    }

    struct serial_struct serial_settings;
    if (ioctl(fd, TIOCGSERIAL, &serial_settings) < 0) 
        return false; // Failed to get serial settings

    serial_settings.flags |= ASYNC_LOW_LATENCY; // Set low latency mode

    if (ioctl(fd, TIOCSSERIAL, &serial_settings) < 0)
        return false; // Failed to set serial settings

    speed_t baud_rate = calculateBaudRate(baud);
    struct termios options;

    bzero( &options, sizeof( options ) ); 

    options.c_cflag = baud_rate | CS8 | CLOCAL | CREAD;
    options.c_iflag = 0;
    options.c_oflag = 0;
    options.c_lflag = 0;

    cfsetispeed(&options, baud_rate);
    cfsetospeed(&options, baud_rate);

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);
    return true;
}

void PconDriver::closePort() {
    if (fd >= 0) {
        close(fd);
        fd = -1; // Reset file descriptor
    }
}

speed_t PconDriver::calculateBaudRate(int baud){
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default: return B38400;
    }
}

void PconDriver::write(const char* data, size_t size) {
    if (fd >= 0) {
        ssize_t ret = ::write(fd, data, size);
        if (ret < 0) {
            // Handle write error
            std::cerr << "Write error: " << std::endl;
        }
    }
}

bool PconDriver::read(char* data, size_t size) {
    if (fd < 0) {
        return false; // Port not open
    }

    fd_set fdset;
    FD_ZERO(&fdset);
    FD_SET(fd, &fdset);

    struct timeval timeout;
    timeout.tv_sec  = 0;
    timeout.tv_usec = 10000;

    select(fd + 1, &fdset, NULL, NULL, &timeout);
    int ret = 0;

    if (FD_ISSET(fd, &fdset)) {
        int read_size = 0;
        ioctl(fd, FIONREAD, &read_size);

        if (read_size < size) {
            ret = ::read(fd, data, read_size);
        }
        else{
            ret = ::read(fd, data, size);
        }
    }

    if (ret < 0) {
        std::cerr << "Read error: " << std::endl;
        return false; // Read error
    }

    return true; // Read successful
}
