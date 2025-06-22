#include "iai_pcon_driver/pcon_driver.hpp"

static long getCurrentTime()
{
    struct timespec tv;
    clock_gettime(CLOCK_REALTIME, &tv);

    return tv.tv_nsec;
}

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

    baudRate = baud;
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

unsigned short PconDriver::calculateCRC(const char* data, size_t size) {
    unsigned char crcLo = 0xFF;
    unsigned char crcHi = 0xFF;

    for (size_t i = 0; i < size; ++i) {
        unsigned char index = crcLo ^ data[i];
        crcLo = crcHi ^ auchCRCHi[index];
        crcHi = auchCRCLo[index];
    }

    return (crcHi << 8) | crcLo;
}


void PconDriver::createReadRegisterMessage(char* message, int& response_size) {
    const uint8_t NUM_OF_REGISTERS = 16; // Number of registers to read
    const int MESSAGE_SIZE = 8; // 1 byte for slave address, 1 byte for function code, 2 bytes for starting address, 2 bytes for number of registers, and 2 bytes for CRC


    message[0] = 0x01; // Slave address
    message[1] = FunctionCode::READ; // Function code
    message[2] = 0x90; // Starting address high byte
    message[3] = 0x00; // Starting address low byte (0x9000)
    message[4] = 0x00; // Number of registers high byte
    message[5] = NUM_OF_REGISTERS; // Number of registers low byte (16 registers)
    
    unsigned short crc = calculateCRC(message, MESSAGE_SIZE -2);
    message[7] = (crc & 0xFF00) >> 8;
    message[6] = crc & 0x00FF;
    
    response_size = NUM_OF_REGISTERS * 2 + 5; // 2 bytes per register + 5 bytes for header and CRC
    std::cout << "Response size: " << response_size << std::endl;
}

bool PconDriver::sendMessage(char* message, int message_size, char* response, int response_size) {
    int rcvCnt = 0;
    int rcvSize = 0;

    for (int retry = 0; retry < NUM_RETRY; ++retry) {
        rcvCnt = transmitMessage(message, message_size, response, response_size);
        if (rcvCnt > 0) {
            unsigned short crc = calculateCRC(response, rcvCnt - 2);
            unsigned short received_crc = (response[rcvCnt - 2] << 8) | response[rcvCnt - 1];

            if (crc == received_crc) {
                return true; // Message sent and response received successfully
            }
            else{
                std::cerr << "CRC mismatch: calculated " << crc << ", received " << received_crc << std::endl;
            }
        }
        usleep(MIN_TRANSMIT_DELAY_MS * 1000); // Delay before retrying
    }
    return false; // Failed to send message or receive valid response
}

int PconDriver::transmitMessage(char* message, int message_size, char* response, int response_size) {
    static const unsigned long inf = 1000000000;

    long diffNs = 0;
    long startTime = 0;
    long endTime = 0;

    int rcvCnt = 0;
    int rcvSize = 0;

    usleep(1000);
    write(message, message_size);


    int expected_timeout_time = NINE_POSITION_DATA_IO_MS * SAFETY_FACTOR_DELAY + (message_size + 8) / (baudRate/100);

    if (response_size > 0)
    {
        startTime = getCurrentTime();

        while (rcvCnt < response_size)
        {
            rcvSize = read(response + rcvCnt, response_size - rcvCnt);
            rcvCnt += rcvSize;
            
            std::cout << "Required : " << response_size << ", received: " << rcvCnt << std::endl;
            //std::cout << "Received " << rcvSize << " bytes, total: " << rcvCnt << std::endl;


            endTime = getCurrentTime();
            diffNs = endTime - startTime;
            if (diffNs < 0)
            {
                diffNs = (inf - startTime) + endTime;
            }

            if (diffNs > expected_timeout_time * 1000000) // Convert ms to ns
            {
                return -1; // Timeout error
            }
        }
    }

    return rcvCnt;
}

void PconDriver::parseMessage(char* response, int response_size, PconStatus& status){
    if (response_size < 37){
        std::cerr << "Response size is too small." << std::endl;
        return; // Invalid response size
    }

    const int NONNEED_DATA = 3;

    long pos = 0;
    unsigned short alarm = 0;

    pos  = (response[NONNEED_DATA] << 24) & 0xFF000000;
    pos |= (response[NONNEED_DATA + 1] << 16) & 0x00FF0000;
    pos |= (response[NONNEED_DATA + 2] <<  8) & 0x0000FF00;
    pos |= (response[NONNEED_DATA + 3]      ) & 0x000000FF;

    alarm  = (response[NONNEED_DATA + 4] << 8) & 0xFF00;
    alarm |= (response[NONNEED_DATA + 5]     ) & 0x00FF;

    status.input_port_status = (response[NONNEED_DATA + 6] << 8) | response[NONNEED_DATA + 7];
    status.output_port_status = (response[NONNEED_DATA + 8] << 8) | response[NONNEED_DATA + 9];
    status.device_status_one = (response[NONNEED_DATA + 10] << 8) | response[NONNEED_DATA + 11];
    status.device_status_two = (response[NONNEED_DATA + 12] << 8) | response[NONNEED_DATA + 13];
    status.extended_device_status = (response[NONNEED_DATA + 14] << 8) | response[NONNEED_DATA + 15];
    status.system_status = (response[NONNEED_DATA + 16] << 24) | 
                           (response[NONNEED_DATA + 17] << 16) | 
                           (response[NONNEED_DATA + 18] << 8) | 
                           (response[NONNEED_DATA + 19]     );

    uint16_t speed_raw = (response[NONNEED_DATA + 20] << 8) | response[NONNEED_DATA + 21];
    int16_t speed_sign = static_cast<int16_t>(speed_raw);
    status.current_speed = static_cast<float>(speed_sign) / 100.f; //Convert to mm/s
    status.current_position = static_cast<float>(pos) / 100.f; //Convert to mm

    uint16_t current = (response[NONNEED_DATA + 22] << 8) | response[NONNEED_DATA + 23];
    status.current = static_cast<float>(current) / 1000.f; //Convert to A
    status.current_difference = (response[NONNEED_DATA + 24] << 8) | response[NONNEED_DATA + 25]; // pulse numbers
    
}