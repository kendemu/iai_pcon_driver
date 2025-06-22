#pragma once

#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/serial.h>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>


// Constants for Timeouts and delays, written in p.35 of the IAI PCON Modbus Protocol Manual
constexpr int NUM_RETRY = 3;
constexpr int MIN_TRANSMIT_DELAY_MS = 5;
constexpr int FAST_MEMORY_IO_MS = 1;
constexpr int ONE_POSITION_DATA_READ_MS = 4;
constexpr int ONE_POSITION_DATA_WRITE_MS = 15;
constexpr int ONE_POSITION_DATA_IO_MS = 18;
constexpr int NINE_POSITION_DATA_READ_MS = 9;
constexpr int NINE_POSITION_DATA_WRITE_MS = 90;
constexpr int NINE_POSITION_DATA_IO_MS = 98;
constexpr int SAFETY_FACTOR_DELAY = 3;

class PconDriver{
    public:
      bool openPort(std::string port, int baud);
      void closePort();

    private:

      void write(const char* data, size_t size);
      bool read(char* data, size_t size);
      int fd;
      int baudRate;

      speed_t calculateBaudRate(int baud);
};