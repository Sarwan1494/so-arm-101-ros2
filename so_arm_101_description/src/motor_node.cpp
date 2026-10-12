#include <chrono>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "SCServo.h"

#define TORQUE_DISABLE 0
#define TORQUE_ENABLE 1

using namespace std::chrono_literals;

class Motor_node : public rclcpp::Node
{
public:
    Motor_node() : Node("motor_node")
    {

        auto topic_callback = [this](std_msgs::msg::String::UniquePtr msg) -> void
        {
            if (msg->data == jointPos)
            {
                for (int i = 1; i < 7; i++)
                {
                    pos = servo.ReadPos(i);
                    RCLCPP_INFO(this->get_logger(), "Servo '%d': '%d'\n", i, pos);
                    usleep(30);
                }
            }
            else if (msg->data.substr(0, moveCommand.length()) == moveCommand) // starts with() returns bool
            {
                std::string payload = msg->data.substr(moveCommand.length());
                servoID = payload[0] - '0';

                value = std::stoi(payload.substr(1));
                RCLCPP_INFO(this->get_logger(), "Servo ID: '%d', Angle: '%d'\n", servoID, value);
                servo.WritePosEx(servoID, value, 2000, 10);
            }
            else
            {
                /* code */
                RCLCPP_INFO(this->get_logger(), "Changing the min position value of the servo");
                value = std::stoi(msg->data);
                for (int i = 1; i < 7; i++)
                {
                    servo.writeWord(i, SMS_STS_MIN_ANGLE_LIMIT_L, (uint16_t)value);
                    usleep(15);
                }
                RCLCPP_INFO(this->get_logger(), "Min angle position sent to servos");
            }

            // pos = servo.Ping(1);
            // RCLCPP_INFO(this->get_logger(), "Received command: '%d'", pos);

            // pos = servo.WritePosEx(1, 3000, 2000, 10);
            // RCLCPP_INFO(this->get_logger(), "Output: '%d'", pos);
        };
        subscription_ = this->create_subscription<std_msgs::msg::String>("orientation", 10, topic_callback);

        // pos = servo.ReadPos(1);
        // servo.setBaudRate(115200);
        servo.begin(1000000, serialPort);

        RCLCPP_INFO(this->get_logger(), "Disabling the torque for all the servos");
        for (int i = 1; i < 7; i++)
        {
            servo.EnableTorque(i, TORQUE_DISABLE);
            usleep(10);
        }
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    SMS_STS servo;
    int pos, value, servoID;
    const char *serialPort = "/dev/ttyACM0";
    const char *jointPos = "Position";
    std::string moveCommand = "move";
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Motor_node>());
    rclcpp::shutdown();
    return 0;
}