#include "iai_pcon_driver/pcon_driver.hpp"

int main(int argc, char** argv) {
    PconDriver driver;

    // Example usage: Open a port with a specific baud rate
    std::string port = "/dev/ttyUSB0"; // Replace with your actual port
    int baudRate = 38400; // Replace with your desired baud rate

    if (driver.openPort(port, baudRate)) {
        std::cout << "Port opened successfully." << std::endl;
        
        // Perform operations with the driver...

        driver.closePort();
        std::cout << "Port closed." << std::endl;
    } else {
        std::cerr << "Failed to open port." << std::endl;
    }

    return 0;
}