#include <memory>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
   public:
    MinimalSubscriber() : Node("minimal_subscriber") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

        ackermann_subscription_ =
            this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
                "drive", 10, std::bind(&MinimalSubscriber::ackermann_callback, this, _1));

        publisher_ =
            this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 10);
    }

   private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    void ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) const {
        auto new_message = ackermann_msgs::msg::AckermannDriveStamped();
        new_message.drive.speed = msg->drive.speed * 3;
        new_message.drive.steering_angle = msg->drive.steering_angle * 3;

        RCLCPP_INFO(this->get_logger(), "Received: speed=%f, steering_angle=%f", msg->drive.speed,
                    msg->drive.steering_angle);
        RCLCPP_INFO(this->get_logger(), "Publishing: speed=%f, steering_angle=%f",
                    new_message.drive.speed, new_message.drive.steering_angle);

        publisher_->publish(new_message);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
        ackermann_subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
