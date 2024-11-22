#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class TwistPublisher : public rclcpp::Node
{
public:
    TwistPublisher()
    : Node("twist_publisher")
    {
        // cmd_vel 토픽에 Twist 메시지 퍼블리셔 생성
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_control", 10);

        // 타이머를 설정하여 주기적으로 publish
        timer_ = this->create_wall_timer(
            500ms, std::bind(&TwistPublisher::publish_twist_message, this));
    }

private:
    void publish_twist_message()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.5;   // 직진 속도 (m/s)
        message.angular.z = 0.0;  // 회전 속도 (rad/s)

        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x = %.2f, angular.z = %.2f",
                    message.linear.x, message.angular.z);

        // 메시지 퍼블리시
        publisher_->publish(message);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistPublisher>());
    rclcpp::shutdown();
    return 0;
}
