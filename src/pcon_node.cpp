#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "iai_pcon_driver/pcon_driver.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class PconROS2Driver : public rclcpp::Node
{
public:
    PconROS2Driver() : Node("pcon_ros2_driver")
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 230400);
        this->declare_parameter<float>("speed_mms", 50.0);
        this->declare_parameter<float>("acceleration_g", 0.3);
        this->declare_parameter<float>("step_position_mm", 0.1);
        this->declare_parameter<float>("publish_rate_hz", 50.0);

        port_ = this->get_parameter("port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();
        speed_mms_ = this->get_parameter("speed_mms").as_double();
        acceleration_g_ = this->get_parameter("acceleration_g").as_double();
        step_position_mm_ = this->get_parameter("step_position_mm").as_double();
        double publish_rate = this->get_parameter("publish_rate_hz").as_double();

        // Initialize the PCON driver
        driver_ = std::make_unique<PconDriver>();
        if (!driver_->openPort(port_, baudrate_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open port %s at %d baud.", port_.c_str(), baudrate_);
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully opened port %s", port_.c_str());


        initialize();

        // Create publisher for joint state
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // Create subscriber for target pose
        target_pose_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "target_pose", 10, std::bind(&PconROS2Driver::targetPoseCallback, this, std::placeholders::_1));

        // Create a timer to periodically read status and publish
        
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
            std::bind(&PconROS2Driver::publishStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "IAI PCON ROS 2 Driver node started successfully.");
    }

    ~PconROS2Driver()
    {
        if (driver_)
        {
            driver_->closePort();
        }
    }

private:
    void initialize(){
        //follow the PCON CB state machine
        int response_size = 0;
        std::vector<uint8_t> query_message = driver_->createReadRegisterMessage(response_size);
        std::vector<uint8_t> response_message(response_size, 0); 

        if (driver_->sendMessage(query_message, response_message, response_size)) {
            PconStatus status;
            driver_->parseMessage(response_message, status);
            DeviceStatus device_status = driver_->parseDeviceStatusOne(status.device_status_one);


            query_message = driver_->createDisableAlarmMessage();
            response_size = query_message.size();
            response_message = std::vector<uint8_t>(response_size, 0);
            if (driver_->sendMessage(query_message, response_message, response_size)){
                RCLCPP_INFO(this->get_logger(), "Successfully Disabled the alarm.");
            }

            usleep(10000);

            query_message = driver_->createResetAlarmMessage();
            response_size = query_message.size();
            response_message = std::vector<uint8_t>(response_size, 0);
            if (driver_->sendMessage(query_message, response_message, response_size)){
                RCLCPP_INFO(this->get_logger(), "Successfully Resetted the alarm.");
            }

            usleep(10000);

            query_message = driver_->createServoOnMessage();
            response_size = query_message.size();
            response_message = std::vector<uint8_t>(response_size, 0);
            if (driver_->sendMessage(query_message, response_message, response_size)){
                RCLCPP_INFO(this->get_logger(), "Enabled the servo.");
            }

            usleep(100000); // wait since it is a motor toggle command

            if(!device_status.is_home_pose){
                RCLCPP_INFO(this->get_logger(), "Device not in home pose, going to home.");
                query_message = driver_->createGoHomeMessage();
                response_size = query_message.size();
                response_message = std::vector<uint8_t>(response_size, 0);
                if (driver_->sendMessage(query_message, response_message, response_size)){
                    RCLCPP_INFO(this->get_logger(), "Going Home");
                }
                usleep(10000000); // long sleep to secure time for go home
            } else {
                RCLCPP_INFO(this->get_logger(), "Device already in home pose");
            }

            RCLCPP_INFO(this->get_logger(), "Initialization done.");

        }
    }


    void targetPoseCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        float target_pos = static_cast<float>(msg->data);

        std::cout << "speed" << speed_mms_ << std::endl;
        RCLCPP_INFO(this->get_logger(), "Received target pose: %.2f mm, step : %.2f, speed :  %.2f mm/s, acc : %.2f G", target_pos, step_position_mm_, speed_mms_, acceleration_g_);
        
        std::vector<uint8_t> move_message = driver_->createMotorMoveMessage(
            target_pos,
            step_position_mm_,
            speed_mms_,
            acceleration_g_
        );

        // For a write command, the response is typically small (8 bytes)
        std::vector<uint8_t> response;
        int expected_response_size = 8;

        if (!driver_->sendMessage(move_message, response, expected_response_size)) {
            RCLCPP_WARN(this->get_logger(), "Failed to send move command or get valid response.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully sent move command.");
        }
    }

    void publishStatus()
    {
        int expected_response_size = 0;
        std::vector<uint8_t> read_message = driver_->createReadRegisterMessage(expected_response_size);
        std::vector<uint8_t> response;

        if (driver_->sendMessage(read_message, response, expected_response_size))
        {
            PconStatus status;
            driver_->parseMessage(response, status);

            // Create and publish the JointState message
            auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
            joint_state_msg->header.stamp = this->get_clock()->now();
            joint_state_msg->name.push_back("linear_actuator_joint");
            joint_state_msg->position.push_back(status.current_position / 1000.0); // Convert mm to meters
            joint_state_msg->velocity.push_back(status.current_speed / 1000.0);   // Convert mm/s to m/s

            joint_state_publisher_->publish(std::move(joint_state_msg));
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to read status from device.");
        }
    }

    // ROS 2 components
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_pose_subscriber_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // Driver and parameters
    std::unique_ptr<PconDriver> driver_;
    std::string port_;
    int baudrate_;
    double speed_mms_;
    double acceleration_g_;
    double step_position_mm_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PconROS2Driver>());
    rclcpp::shutdown();
    return 0;
}