#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include "md_robot_node/global.hpp"
#include "md_robot_node/com.hpp"
#include "md/MdRobotMsg1.hpp"
#include "md/MdRobotMsg2.hpp"
#include <queue>

#define MAX_CONNECTION_CHECK_COUNT 10

using namespace std::chrono_literals;

class MdRobotNode : public rclcpp::Node
{
public:
    MdRobotNode() : Node("md_robot_node")
    {
        // Publishers
        md_robot_message1_pub_ = this->create_publisher<md::msg::MdRobotMsg1>("md_robot_message1", 10);
        md_robot_message2_pub_ = this->create_publisher<md::msg::MdRobotMsg2>("md_robot_message2", 10);

        // Subscribers
        keyboard_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MdRobotNode::cmdVelCallBack, this, std::placeholders::_1));
        reset_position_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "reset_position", 10, std::bind(&MdRobotNode::resetPositionCallBack, this, std::placeholders::_1));
        reset_alarm_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "reset_alarm", 10, std::bind(&MdRobotNode::resetAlarmCallBack, this, std::placeholders::_1));

        // Timer for velocity command timeout
        vel_cmd_rcv_timeout_ = this->create_wall_timer(1s, std::bind(&MdRobotNode::VelCmdRcvTimeoutCallback, this));

        // Initialize parameters
        this->declare_parameter("md_robot_node/use_MDUI", 0);
        this->declare_parameter("md_robot_node/serial_port", std::string("/dev/ttyUSB0"));
        this->declare_parameter("md_robot_node/serial_baudrate", 115200);
        this->declare_parameter("md_robot_node/reverse_direction", 0);
        this->declare_parameter("md_robot_node/maxrpm", 100);
        this->declare_parameter("md_robot_node/enable_encoder", 0);
        this->declare_parameter("md_robot_node/slow_start", 0);
        this->declare_parameter("md_robot_node/slow_down", 0);
        this->declare_parameter("md_robot_node/wheel_length", 0.0);
        this->declare_parameter("md_robot_node/reduction", 1);
        this->declare_parameter("md_robot_node/wheel_radius", 0.1);
        this->declare_parameter("md_robot_node/encoder_PPR", 0);

        // Load parameters
        this->get_parameter("md_robot_node/use_MDUI", robotParamData.use_MDUI);
        this->get_parameter("md_robot_node/serial_port", serial_port_);
        this->get_parameter("md_robot_node/serial_baudrate", robotParamData.nBaudrate);
        this->get_parameter("md_robot_node/reverse_direction", robotParamData.reverse_direction);
        this->get_parameter("md_robot_node/maxrpm", robotParamData.nMaxRPM);
        this->get_parameter("md_robot_node/enable_encoder", robotParamData.enable_encoder);
        this->get_parameter("md_robot_node/slow_start", robotParamData.nSlowstart);
        this->get_parameter("md_robot_node/slow_down", robotParamData.nSlowdown);
        this->get_parameter("md_robot_node/wheel_length", robotParamData.nWheelLength);
        this->get_parameter("md_robot_node/reduction", robotParamData.nGearRatio);
        this->get_parameter("md_robot_node/wheel_radius", robotParamData.wheel_radius);
        this->get_parameter("md_robot_node/encoder_PPR", robotParamData.encoder_PPR);

        RCLCPP_INFO(this->get_logger(), "Node initialized with serial port: %s", serial_port_.c_str());
    }

private:
    // Callback functions
    void cmdVelCallBack(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (fgInitsetting == INIT_SETTING_STATE_OK) {
            velCmdRcvCount++;
            velCmdUpdateCount++;

            goal_cmd_speed = msg->linear.x;
            goal_cmd_ang_speed = msg->angular.z;
        }
    }

    void resetPositionCallBack(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data == true) {
            RCLCPP_INFO(this->get_logger(), "Reset Position");
            reset_pos_flag = true;
        }
    }

    void resetAlarmCallBack(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data == true) {
            RCLCPP_INFO(this->get_logger(), "Reset Alarm");
            reset_alarm_flag = true;
        }
    }

    void VelCmdRcvTimeoutCallback()
    {
        static uint32_t old_velCmdRcvCount;
        if (velCmdRcvCount == old_velCmdRcvCount) {
            goal_cmd_speed = 0;
            goal_cmd_ang_speed = 0;
            if (remote_pc_connection_state == true) {
                velCmdUpdateCount++;
                remote_pc_connection_state = false;
            }
        } else {
            old_velCmdRcvCount = velCmdRcvCount;
            if (remote_pc_connection_state == false) {
                remote_pc_connection_state = true;
            }
        }
    }

    // ROS2 publishers, subscribers, and timers
    rclcpp::Publisher<md::msg::MdRobotMsg1>::SharedPtr md_robot_message1_pub_;
    rclcpp::Publisher<md::msg::MdRobotMsg2>::SharedPtr md_robot_message2_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr keyboard_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_alarm_sub_;
    rclcpp::TimerBase::SharedPtr vel_cmd_rcv_timeout_;

    // Parameters
    std::string serial_port_;
    double goal_cmd_speed;
    double goal_cmd_ang_speed;
    bool reset_pos_flag;
    bool reset_alarm_flag;
    volatile bool remote_pc_connection_state;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MdRobotNode>());
    rclcpp::shutdown();
    return 0;
}