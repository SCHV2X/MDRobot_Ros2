#include <rclcpp/rclcpp.hpp>
#include "md_robot_node/global.hpp"
#include "md_robot_node/main.hpp"
#include "md_robot_node/com.hpp"
#include "md/md_robot_msg1.hpp"
#include "md/md_robot_msg2.hpp"
#include <serial/serial.h>

#define ENABLE_MD_MESSAGE
#define VELOCITY_CONSTANT_VALUE 9.5492743

using namespace std::chrono_literals;

class RobotCommunicationNode : public rclcpp::Node
{
public:
    RobotCommunicationNode() : Node("robot_communication_node")
    {
        // Initialize serial communication
        if (InitSerialComm() == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial communication.");
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(), "Serial communication initialized successfully.");
        }

        // Publishers
        md_robot_message1_pub_ = this->create_publisher<md::msg::MdRobotMsg1>("md_robot_message1", 10);
        md_robot_message2_pub_ = this->create_publisher<md::msg::MdRobotMsg2>("md_robot_message2", 10);

        // Timer for serial data receiving
        serial_receive_timer_ = this->create_wall_timer(10ms, std::bind(&RobotCommunicationNode::ReceiveSerialData, this));
    }

private:
    int InitSerialComm()
    {
        std::string port;
        this->get_parameter_or("md_robot_node/serial_port", port, std::string("/dev/ttyUSB0"));
        int baudrate;
        this->get_parameter_or("md_robot_node/serial_baudrate", baudrate, 115200);

        try {
            ser_.setPort(port);
            ser_.setBaudrate(baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(to);
            ser_.open();
        } catch (serial::IOException &e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", port.c_str());
            return -1;
        }

        if (ser_.isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Serial port open: %s", port.c_str());
            return 1;
        } else {
            return -1;
        }
    }

    void ReceiveSerialData()
    {
        if (ser_.available()) {
            uint8_t buffer[256];
            size_t bytes_read = ser_.read(buffer, sizeof(buffer));
            AnalyzeReceivedData(buffer, bytes_read);
        }
    }

    void AnalyzeReceivedData(uint8_t *data, size_t length)
    {
        // Implementation of analyzing data and publishing relevant messages
        // For example:
        RCLCPP_INFO(this->get_logger(), "Received data of length: %zu", length);
    }

    // Publishers
    rclcpp::Publisher<md::msg::MdRobotMsg1>::SharedPtr md_robot_message1_pub_;
    rclcpp::Publisher<md::msg::MdRobotMsg2>::SharedPtr md_robot_message2_pub_;

    // Timer for receiving serial data
    rclcpp::TimerBase::SharedPtr serial_receive_timer_;

    // Serial object
    serial::Serial ser_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotCommunicationNode>());
    rclcpp::shutdown();
    return 0;
}