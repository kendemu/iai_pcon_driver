#include "iai_pcon_driver/pcon_driver.hpp"

static long getCurrentTimeNs()
{
    struct timespec tv;
    clock_gettime(CLOCK_REALTIME, &tv);
    return tv.tv_sec * 1000000000L + tv.tv_nsec;
}

bool PconDriver::openPort(const std::string& port, int baud) {
    fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "Error opening port " << port << ": " << strerror(errno) << std::endl;
        return false;
    }

    struct serial_struct serial_settings;
    if (ioctl(fd, TIOCGSERIAL, &serial_settings) < 0) {
        std::cerr << "Error getting serial settings: " << strerror(errno) << std::endl;
        return false;
    }

    serial_settings.flags |= ASYNC_LOW_LATENCY;
    if (ioctl(fd, TIOCSSERIAL, &serial_settings) < 0) {
        std::cerr << "Error setting low latency mode: " << strerror(errno) << std::endl;
        return false;
    }

    baudRate = baud;
    speed_t baud_rate_flag = calculateBaudRate(baud);
    struct termios options;
    bzero(&options, sizeof(options));

    options.c_cflag = baud_rate_flag | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR; // Ignore parity errors
    options.c_oflag = 0;
    options.c_lflag = 0;

    // Set read timeouts
    options.c_cc[VTIME] = 1; // 0.1 seconds timeout
    options.c_cc[VMIN] = 0;  // Non-blocking read

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);
    return true;
}

void PconDriver::closePort() {
    if (fd >= 0) {
        close(fd);
        fd = -1;
    }
}

DeviceStatus PconDriver::parseDeviceStatusOne(uint16_t device_status_one){
    DeviceStatus status;
    status.is_load_cell_calibrated = (device_status_one & DEVICE_STATUS_IS_LOAD_CELL_CALIB) != 0;
    status.is_load_cell_calib_command_done = (device_status_one & DEVICE_STATUS_IS_LOAD_CELL_CALIB_DONE) != 0;
    status.is_position_set = (device_status_one & DEVICE_STATUS_IS_POSITION_SET) != 0;
    status.is_home_pose = (device_status_one & DEVICE_STATUS_IS_HOME_POSE) != 0;
    status.is_pause = (device_status_one & DEVICE_STATUS_IS_PAUSE) != 0;
    status.is_break = (device_status_one & DEVICE_STATUS_BRKL) != 0;
    status.is_abs_encoder_err = (device_status_one & DEVICE_STATUS_ABS_ERR) != 0;
    status.is_light_err = (device_status_one & DEVICE_STATUS_LIGHT_ERR) != 0;
    status.is_heavy_err = (device_status_one & DEVICE_STATUS_HEAVY_ERR) != 0;
    status.is_press_not_contacted = (device_status_one & DEVICE_STATUS_IS_PRESS_NOT_CONTACTED) != 0;
    status.is_servo_on = (device_status_one & DEVICE_STATUS_IS_SERVO_ON) != 0;
    status.is_controller_ready = (device_status_one & DEVICE_STATUS_IS_CONTROLLER_READY) != 0;
    status.is_safety_activated = (device_status_one & DEVICE_STATUS_IS_SAFETY_ACTIVATED) != 0;
    status.is_emergency = (device_status_one & DEVICE_STATUS_IS_EMERGENCY) != 0;

    return status;
}


speed_t PconDriver::calculateBaudRate(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default: return B38400; // Default baud rate
    }
}

// Writes the content of a vector to the serial port
void PconDriver::writePort(const std::vector<uint8_t>& data) {
    if (fd >= 0) {
        ssize_t bytes_written = ::write(fd, data.data(), data.size());
        if (bytes_written < 0) {
            std::cerr << "Write error: " << strerror(errno) << std::endl;
        }
    }
}

// Reads from the serial port into a buffer, returns bytes read
ssize_t PconDriver::readPort(uint8_t* buffer, size_t size) {
    if (fd < 0) return -1;
    ssize_t bytes_read = ::read(fd, buffer, size);
    if (bytes_read < 0) {
        // EAGAIN is not a fatal error, just means no data was available
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
             std::cerr << "Read error: " << strerror(errno) << std::endl;
        }
    }
    return bytes_read;
}

unsigned short PconDriver::calculateCRC(const std::vector<uint8_t>& data, size_t size) const {
    unsigned char crcLo = 0xFF;
    unsigned char crcHi = 0xFF;
    
    // Ensure we don't read past the vector's end
    size_t len = std::min(size, data.size());

    for (size_t i = 0; i < len; ++i) {
        // With uint8_t, there's no need to cast data[i]
        unsigned char index = crcLo ^ data[i];
        crcLo = crcHi ^ auchCRCHi[index];
        crcHi = auchCRCLo[index];
    }
    return (static_cast<unsigned short>(crcHi) << 8) | crcLo;
}

std::vector<uint8_t> PconDriver::createReadRegisterMessage(int& response_size) {
    const uint8_t NUM_OF_REGISTERS = 16;
    const int MESSAGE_SIZE = 8;
    std::vector<uint8_t> message(MESSAGE_SIZE);

    message[0] = 0x01; // Slave address
    message[1] = FunctionCode::READ; // Function code (e.g., 0x03)
    message[2] = 0x90; // Starting address high byte
    message[3] = 0x00; // Starting address low byte (0x9000)
    message[4] = 0x00; // Number of registers high byte
    message[5] = NUM_OF_REGISTERS; // Number of registers low byte (16 registers)
    
    // Calculate CRC on the first 6 bytes
    unsigned short crc = calculateCRC(message, MESSAGE_SIZE - 2);
    message[6] = crc & 0x00FF;        // CRC LSB
    message[7] = (crc & 0xFF00) >> 8; // CRC MSB

    // Debug print with proper formatting
    /*
    std::cout << "Generated message: ";
    for(uint8_t byte : message) {
        std::cout << "0x" << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl; // Reset iostream to decimal
    */
    response_size = NUM_OF_REGISTERS * 2 + 5; // Expected response: 2 bytes/reg + addr, func, byte count, 2x crc
    return message;
}

std::vector<uint8_t> PconDriver::createDisableAlarmMessage(){
    const int MESSAGE_SIZE = 8;
    std::vector<uint8_t> message(MESSAGE_SIZE);

    message[0] = SLAVE_ADDR;
    message[1] = FunctionCode::WRITE;
    message[2] = 0x04; // address of alarm MSB
    message[3] = 0x07; // address of alarm LSB
    message[4] = 0xFF; // alarm reset code MSB
    message[5] = 0x00; // alarm reset code LSB
    unsigned short crc = calculateCRC(message, MESSAGE_SIZE - 2);
    message[6] = crc & 0x00FF;        // CRC LSB
    message[7] = (crc & 0xFF00) >> 8; // CRC MSB

    return message; //response size = query size
}

std::vector<uint8_t> PconDriver::createResetAlarmMessage() {
    const int MESSAGE_SIZE = 8;
    std::vector<uint8_t> message(MESSAGE_SIZE);

    message[0] = SLAVE_ADDR;
    message[1] = FunctionCode::WRITE;
    message[2] = 0x04; // address of alarm MSB
    message[3] = 0x07; // address of alarm LSB
    message[4] = 0x00; // alarm reset code MSB
    message[5] = 0x00; // alarm reset code LSB
    unsigned short crc = calculateCRC(message, MESSAGE_SIZE - 2);
    message[6] = crc & 0x00FF;        // CRC LSB
    message[7] = (crc & 0xFF00) >> 8; // CRC MSB

    return message; //response size = query size
}

std::vector<uint8_t> PconDriver::createServoOnMessage() {
    const int MESSAGE_SIZE = 8;
    std::vector<uint8_t> message(MESSAGE_SIZE);

    message[0] = SLAVE_ADDR;
    message[1] = FunctionCode::WRITE;
    message[2] = 0x04; // address of alarm MSB
    message[3] = 0x03; // address of alarm LSB
    message[4] = 0xFF; // alarm disable code MSB
    message[5] = 0x00; // alarm disable code LSB
    unsigned short crc = calculateCRC(message, MESSAGE_SIZE - 2);
    message[6] = crc & 0x00FF;        // CRC LSB
    message[7] = (crc & 0xFF00) >> 8; // CRC MSB

    return message; //response size = query size    
}

std::vector<uint8_t> PconDriver::createServoOffMessage() {
    const int MESSAGE_SIZE = 8;
    std::vector<uint8_t> message(MESSAGE_SIZE);

    message[0] = SLAVE_ADDR;
    message[1] = FunctionCode::WRITE;
    message[2] = 0x04; // address of servoi MSB
    message[3] = 0x03; // address of servo LSB
    message[4] = 0x00; // alarm reset code MSB
    message[5] = 0x00; // alarm reset code LSB
    unsigned short crc = calculateCRC(message, MESSAGE_SIZE - 2);
    message[6] = crc & 0x00FF;        // CRC LSB
    message[7] = (crc & 0xFF00) >> 8; // CRC MSB

    return message; //response size = query size    
}

std::vector<uint8_t> PconDriver::createGoHomeMessage(){
    const int MESSAGE_SIZE = 8;
    std::vector<uint8_t> message(MESSAGE_SIZE);

    message[0] = SLAVE_ADDR;
    message[1] = FunctionCode::WRITE;
    message[2] = 0x04; // address of go home MSB
    message[3] = 0x0B; // address of go home LSB
    message[4] = 0xFF; // go home MSB
    message[5] = 0x00; // go home LSB
    unsigned short crc = calculateCRC(message, MESSAGE_SIZE - 2);
    message[6] = crc & 0x00FF;        // CRC LSB
    message[7] = (crc & 0xFF00) >> 8; // CRC MSB

    return message; //response size = query size    
}

std::vector<uint8_t> PconDriver::createMotorMoveMessage(float pos_mm, float step_pos_mm, float speed_mm_s, float acc_g){
    const int MESSAGE_SIZE = 23;
    const uint8_t NUM_OF_REGISTERS = 7;
    std::vector<uint8_t> message(MESSAGE_SIZE, 0);

    message[0] = SLAVE_ADDR;
    message[1] = FunctionCode::WRITE_MULTIPLE;
    message[2] = 0x99; // address of position command MSB
    message[3] = 0x00; // address of postiion command LSB
    message[4] = 0x00; // number of resigsters
    message[5] = NUM_OF_REGISTERS; // number of registers
    message[6] = NUM_OF_REGISTERS * 2; // number of bytes

    uint32_t target_position = static_cast<uint32_t>(pos_mm * 100);
    message[7] = static_cast<uint8_t>((target_position >> 24) & 0xFF);   // target position MSB (Reg 9900) (4bytes) [xx 00 00 00]
    message[8] = static_cast<uint8_t>((target_position >> 16) & 0xFF);   // [00 xx 00 00]
    message[9] = static_cast<uint8_t>((target_position >> 8) & 0xFF);    // [00 00 xx 00]
    message[10] = static_cast<uint8_t>(target_position & 0xFF);          // [00 00 00 xx]

    uint32_t step_size = static_cast<uint32_t>(step_pos_mm * 100);
    message[11] = static_cast<uint8_t>((step_size >> 24) & 0xFF);        // step size MSB (Reg 9902) (4bytes) [xx 00 00 00]
    message[12] = static_cast<uint8_t>((step_size >> 16) & 0xFF);        // [00 xx 00 00]
    message[13] = static_cast<uint8_t>((step_size >> 8) & 0xFF);         // [00 00 xx 00]
    message[14] = static_cast<uint8_t>(step_size & 0xFF);                // [00 00 00 xx]

    uint32_t target_speed = static_cast<uint32_t>(speed_mm_s * 100);
    message[15] = static_cast<uint8_t>((target_speed >> 24) & 0xFF);     // target_speed MSB (Reg 9904) (4bytes) [xx 00 00 00]
    message[16] = static_cast<uint8_t>((target_speed >> 16) & 0xFF);     // [00 xx 00 00]
    message[17] = static_cast<uint8_t>((target_speed >> 8) & 0xFF);      // [00 00 xx 00]
    message[18] = static_cast<uint8_t>(target_speed & 0xFF);             // [00 00 00 xx]

    uint16_t target_g = static_cast<uint16_t>(acc_g * 100);
    message[19] = static_cast<uint8_t>(target_g >> 8);                   // target g MSB (Reg 9906) (2bytes) [xx 00]
    message[20] = static_cast<uint8_t>(target_g & 0x00FF);               // target g LSB [00 xx]

    unsigned short crc = calculateCRC(message, MESSAGE_SIZE - 2);
    message[21] = crc & 0x00FF;        // CRC LSB
    message[22] = (crc & 0xFF00) >> 8; // CRC MSB

    return message;
}

bool PconDriver::sendMessage(const std::vector<uint8_t>& message, std::vector<uint8_t>& response, int response_size) {
    for (int retry = 0; retry < NUM_RETRY; ++retry) {
        int rcvCnt = transmitMessage(message, response, response_size);

        if (rcvCnt > 0) {
            // Trim response to actual number of bytes received
            response.resize(rcvCnt); 
            
            // Need at least 3 bytes for a minimal valid response (addr, func, crc)
            if (rcvCnt < 3) {
                std::cerr << "Response too short to have a CRC." << std::endl;
                continue; // Retry
            }
            
            unsigned short crc_calculated = calculateCRC(response, rcvCnt - 2);
            uint8_t crc_calc_lsb = crc_calculated & 0x00FF;
            uint8_t crc_calc_msb = (crc_calculated & 0xFF00) >> 8;

            uint8_t crc_recv_lsb = response[rcvCnt - 2];
            uint8_t crc_recv_msb = response[rcvCnt - 1];

            if (crc_recv_lsb == crc_calc_lsb && crc_recv_msb == crc_calc_msb) {
                return true; // Success!
            } else {
                std::cerr << "CRC mismatch: calculated 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(crc_calc_msb) << " 0x" << static_cast<int>(crc_calc_lsb)
                          << ", received 0x" << static_cast<int>(crc_recv_msb) << " 0x" << static_cast<int>(crc_recv_lsb) << std::dec << std::endl;
            }
        }
        usleep(MIN_TRANSMIT_DELAY_MS * 1000);
    }
    return false;
}

int PconDriver::transmitMessage(const std::vector<uint8_t>& message, std::vector<uint8_t>& response, int response_size) {
    tcflush(fd, TCIOFLUSH); // Flush both input and output buffers before transaction
    
    writePort(message);
    
    int rcvCnt = 0;
    if (response_size > 0) {
        response.assign(response_size, 0); // Pre-allocate and zero-fill the vector

        long startTime = getCurrentTimeNs();
        // Calculate timeout based on baud rate and response size
        long timeoutNs = (1000000000L * (response_size + 10) * 10) / baudRate + 20000000; // 20ms base + time for data

        while (rcvCnt < response_size) {
            ssize_t rcvSize = readPort(response.data() + rcvCnt, response_size - rcvCnt);
            
            if (rcvSize > 0) {
                rcvCnt += rcvSize;
                startTime = getCurrentTimeNs(); // Reset timer on new data
            }

            long diffNs = getCurrentTimeNs() - startTime;
            if (diffNs > timeoutNs) {
                if (rcvCnt > 0) {
                    // Timeout, but we received something. Return what we got.
                    break; 
                }
                return -1; // Timeout with no data
            }
        }
    }
    return rcvCnt;
}

void PconDriver::parseMessage(const std::vector<uint8_t>& response, PconStatus& status) {
    if (response.size() < 37) { // Minimum size for the data we need
        std::cerr << "Response size is too small for parsing." << std::endl;
        return;
    }

    // Byte 0: Slave Address
    // Byte 1: Function Code
    // Byte 2: Byte Count
    const int DATA_START_INDEX = 3;

    // With uint8_t, these bitwise operations are now safe from sign extension.
    // The uint8_t values are promoted to unsigned int, which is what we want.
    uint32_t pos_raw = (static_cast<uint32_t>(response[DATA_START_INDEX + 0]) << 24) |
                       (static_cast<uint32_t>(response[DATA_START_INDEX + 1]) << 16) |
                       (static_cast<uint32_t>(response[DATA_START_INDEX + 2]) << 8)  |
                       (static_cast<uint32_t>(response[DATA_START_INDEX + 3]));
    status.current_position = static_cast<float>(static_cast<int32_t>(pos_raw)) / 100.0f; // Convert from 1/100mm to mm

    status.current_alarm = (static_cast<uint16_t>(response[DATA_START_INDEX + 4]) << 8) | response[DATA_START_INDEX + 5];

    status.input_port_status = (static_cast<uint16_t>(response[DATA_START_INDEX + 6]) << 8) | response[DATA_START_INDEX + 7];
    status.output_port_status = (static_cast<uint16_t>(response[DATA_START_INDEX + 8]) << 8) | response[DATA_START_INDEX + 9];
    status.device_status_one = (static_cast<uint16_t>(response[DATA_START_INDEX + 10]) << 8) | response[DATA_START_INDEX + 11];
    status.device_status_two = (static_cast<uint16_t>(response[DATA_START_INDEX + 12]) << 8) | response[DATA_START_INDEX + 13];
    status.extended_device_status = (static_cast<uint16_t>(response[DATA_START_INDEX + 14]) << 8) | response[DATA_START_INDEX + 15];
    
    uint32_t system_status_raw = (static_cast<uint32_t>(response[DATA_START_INDEX + 16]) << 24) |
                                 (static_cast<uint32_t>(response[DATA_START_INDEX + 17]) << 16) |
                                 (static_cast<uint32_t>(response[DATA_START_INDEX + 18]) << 8)  |
                                 (static_cast<uint32_t>(response[DATA_START_INDEX + 19]));
    status.system_status = system_status_raw;

    uint32_t speed_raw = (static_cast<uint32_t>(response[DATA_START_INDEX + 20]) << 24) | 
                         (static_cast<uint32_t>(response[DATA_START_INDEX + 21]) << 16) |
                         (static_cast<uint32_t>(response[DATA_START_INDEX + 22]) << 8) |
                         (static_cast<uint32_t>(response[DATA_START_INDEX + 23]));
    status.current_speed = static_cast<float>(static_cast<int32_t>(speed_raw)) / 100.0f; // Convert from 1/100mm/s to mm/s

    uint32_t current_raw = (static_cast<uint32_t>(response[DATA_START_INDEX + 24]) << 24) | 
                           (static_cast<uint32_t>(response[DATA_START_INDEX + 25]) << 16) |
                           (static_cast<uint32_t>(response[DATA_START_INDEX + 26]) << 8) |
                           (static_cast<uint32_t>(response[DATA_START_INDEX + 27]));
    status.current_load = static_cast<float>(static_cast<int32_t>(current_raw)) / 1000.0f; // Convert from mA to A

    uint32_t encoder_diff_raw = (static_cast<uint32_t>(response[DATA_START_INDEX + 28]) << 24) |
                                (static_cast<uint32_t>(response[DATA_START_INDEX + 29]) << 16) |
                                (static_cast<uint32_t>(response[DATA_START_INDEX + 30]) << 8) |
                                (static_cast<uint32_t>(response[DATA_START_INDEX + 31]));
    status.encoder_difference = static_cast<int32_t>(encoder_diff_raw);
}