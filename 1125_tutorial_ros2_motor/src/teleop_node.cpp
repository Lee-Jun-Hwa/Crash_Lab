#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>

using namespace std::chrono_literals;

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode()
    : Node("teleop_node"), linear_speed_(0.0), angular_speed_(0.0)
    {
        // cmd_vel_control 토픽에 퍼블리셔 생성
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_control", 10);
        RCLCPP_INFO(this->get_logger(), "Teleop Node Started. Use 'WAXD' to move, 'S' to stop, 'E' to exit.");
    }

    void spin()
    {
        char key;
        auto message = geometry_msgs::msg::Twist();

        while (rclcpp::ok()) {
            key = getKey();
            
            // 'WASD' 키에 따라 로봇 속도 설정
            switch (key) {
                case 'w':  // Forward (Increase linear speed)
                    linear_speed_ += 0.02;
                    break;
                case 'X':  // Backward (Decrease linear speed)
                    linear_speed_ -= 0.02;
                    break;
                case 'a':  // Turn left (Increase angular speed)
                    angular_speed_ += 0.02;
                    break;
                case 'd':  // Turn right (Decrease angular speed)
                    angular_speed_ -= 0.02;
                    break;
                case 'S':  // Stop
                    linear_speed_ = 0.0;
                    angular_speed_ = 0.0;
                    break;
                case 'e':  // Exit
                    RCLCPP_INFO(this->get_logger(), "Exiting Teleop Node.");
                    return;
                default:
                    continue;
            }

            // 메시지에 현재 속도 설정
            message.linear.x = linear_speed_;
            message.angular.z = angular_speed_;

            RCLCPP_INFO(this->get_logger(), "Publishing: linear.x = %.2f, angular.z = %.2f",
                        message.linear.x, message.angular.z);

            // 메시지 퍼블리시
            publisher_->publish(message);
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }

private:
    // 키보드 입력을 받아오는 함수
    char getKey()
    {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(STDIN_FILENO, &old) < 0)
            perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &old) < 0)
            perror("tcsetattr ICANON");
        if (read(STDIN_FILENO, &buf, 1) < 0)
            perror("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(STDIN_FILENO, TCSADRAIN, &old) < 0)
            perror("tcsetattr ~ICANON");
        return buf;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    double linear_speed_;
    double angular_speed_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto teleop_node = std::make_shared<TeleopNode>();
    teleop_node->spin();
    rclcpp::shutdown();
    return 0;
}
