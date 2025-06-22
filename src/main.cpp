#include "iai_pcon_driver/pcon_driver.hpp"

int main(int argc, char** argv) {
    PconDriver driver;

    // Example usage: Open a port with a specific baud rate
    std::string port = "/dev/ttyUSB0"; // Replace with your actual port
    int baudRate = 38400; // Replace with your desired baud rate

    if (driver.openPort(port, baudRate)) {
        std::cout << "Port opened successfully." << std::endl;
        
        char query_message[256];
        int response_size = 0;
        driver.createReadRegisterMessage(query_message, response_size);

        char response[256];
        if (driver.sendMessage(query_message, sizeof(query_message), response, sizeof(response))) {
            PconStatus status;
            driver.parseMessage(response, response_size, status);
            std::cout << "Current Position: " << status.current_position << std::endl;
            std::cout << "Current Speed: " << status.current_speed << std::endl;
            // Print other status fields as needed
        } else {
            std::cerr << "Failed to send message or receive response." << std::endl;
        }
        driver.closePort();
        std::cout << "Port closed." << std::endl;
    } else {
        std::cerr << "Failed to open port." << std::endl;
    }

    return 0;
}