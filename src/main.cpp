#include "iai_pcon_driver/pcon_driver.hpp"

int main(int argc, char** argv) {
    PconDriver driver;

    // Example usage: Open a port with a specific baud rate
    std::string port = "/dev/ttyUSB0"; // Replace with your actual port
    int baudRate = 38400; // Replace with your desired baud rate

    if (driver.openPort(port, baudRate)) {
        std::cout << "Port opened successfully." << std::endl;
        
        int response_size = 0;
        std::vector<uint8_t> query_message = driver.createReadRegisterMessage(response_size);
        std::vector<uint8_t> response_message(response_size, 0); 

        if (driver.sendMessage(query_message, response_message, response_size)) {
            std::cout << "Message sent successfully." << std::endl;
            PconStatus status;
            driver.parseMessage(response_message, status);
            std::cout << "Current Position: " << status.current_position << std::endl;
            std::cout << "Current Speed: " << status.current_speed << std::endl;
            std::cout << "Current Load: " << status.current_load << std::endl;
            std::cout << "Current Alarm: " << std::hex << status.current_alarm << std::dec << std::endl;
            std::cout << "encoder difference: " << status.encoder_difference << std::endl;
            
            query_message = driver.createDisableAlarmMessage();
            response_size = query_message.size();
            response_message = std::vector<uint8_t>(response_size, 0);
            if (driver.sendMessage(query_message, response_message, response_size)){
                std::cout << "Successfully Disabled the alarm." << std::endl;
            }

            usleep(3000000);
            query_message = driver.createResetAlarmMessage();
            response_size = query_message.size();
            response_message = std::vector<uint8_t>(response_size, 0);
            if (driver.sendMessage(query_message, response_message, response_size)){
                std::cout << "Successfully resetted the alarm." << std::endl;
            }

            usleep(3000000);
            query_message = driver.createServoOnMessage();
            response_size = query_message.size();
            response_message = std::vector<uint8_t>(response_size, 0);
            if (driver.sendMessage(query_message, response_message, response_size)){
                std::cout << "Enabled the servo." << std::endl;
            }

            usleep(3000000);
            query_message = driver.createGoHomeMessage();
            response_size = query_message.size();
            response_message = std::vector<uint8_t>(response_size, 0);
            if (driver.sendMessage(query_message, response_message, response_size)){
                std::cout << "Going Home." << std::endl;
            }

            usleep(10000000);

            query_message = driver.createMotorMoveMessage(200.0, 0.1, 50.0, 0.3);
            response_size = MOTOR_MOVE_RESPONSE_SIZE;
            response_message = std::vector<uint8_t>(response_size, 0);

            for(int i = 0; i < query_message.size(); i++){
                std::cout << "0x" << std::hex << static_cast<int>(query_message[i]) << " ";
            }
            std::cout << std::dec << std::endl;
    
            if (driver.sendMessage(query_message, response_message, response_size)){
                std::cout << "Moving the motor." << std::endl;
            }
            usleep(10000000);
        
            /*
            query_message = driver.createServoOffMessage();
            response_size = query_message.size();


            
            response_message = std::vector<uint8_t>(response_size, 0);
            if (driver.sendMessage(query_message, response_message, response_size)){
                std::cout << "Disabled the servo." << std::endl;
            }
            usleep(1000000);
            
            */

        } else {
            std::cerr << "Failed to send message." << std::endl;
            driver.closePort();
            std::cout << "Port closed." << std::endl;
            return 1;
        }


        

    } else {
        std::cerr << "Failed to open port." << std::endl;
    }

    return 0;
}