#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include <cmath>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

enum class MethodType { Method1, Method2 };

class TimeNode : public rclcpp::Node {
   public:
    TimeNode() : Node("time_node") { clock_ = this->get_clock(); }
    double getCurrentTime() { return clock_->now().seconds(); }

   private:
    rclcpp::Clock::SharedPtr clock_;
};

class Safety : public rclcpp::Node {
    // The class that handles emergency braking

   public:
    Safety() : Node("safety_node") {
        /*
        You should also subscribe to the /scan topic to get the sensor_msgs/LaserScan messages and the /ego_racecar/odom
        topic to get the nav_msgs/Odometry messages
        The subscribers should use the provided odom_callback and scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// TODO: create ROS subscribers and publishers
        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10,
                                                                          std::bind(&Safety::scan_callback, this, _1));
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10,
                                                                      std::bind(&Safety::drive_callback, this, _1));

        ackermann_drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    }

   private:
    double speed = 0.0;
    double TTC_1 = 0.0, TTC_2 = 0.0;
    double curr_t = 0.0, prev_t = 0.0, dt = 0.0;
    std::vector<float> prev_ranges;
    std::shared_ptr<TimeNode> node = std::make_shared<TimeNode>();

    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_drive_pub;

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        /// TODO: update current speed
        speed = msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        if (speed == 0.0) return;

        /** @ref: https://answers.ros.org/question/198843/need-explanation-on-sensor_msgslaserscanmsg/ */
        double curr_t = (double)scan_msg->header.stamp.nanosec * (double)10e-9 + (double)scan_msg->header.stamp.sec;
        if (prev_t != 0.0) {
            dt = curr_t - prev_t;
            if (dt < 0.1) return;
        }
        prev_t = curr_t;
        // RCLCPP_INFO(this->get_logger(), "dt = %f", dt);
        MethodType method = MethodType::Method2;

        /// TODO: calculate TTC
        for (int i = 0; i < (int)scan_msg->ranges.size(); i++) {
            double TTC_threshold_1 = 0.1f;
            double TTC_threshold_2 = 1.6f;

            auto ackermann_drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            ackermann_drive_msg.drive.speed = 0.0;

            if (method == MethodType::Method1) {
                /** =====================================================================================================
                 *      This is method 1 to do AEB, with poor performace, as it's hard to tune the threshold
                 *          when the timestamp interval has sharp influence on range rate (some FP)
                 *      Also. during rotation, scan_msg->ranges: hard to measure the range in the same angle. (tf2?)
                 *  =====================================================================================================
                 */
                if (prev_ranges.empty() || dt == 0.0) {
                    break;
                }
                if (std::isnan(scan_msg->ranges[i]) || scan_msg->ranges[i] > scan_msg->range_max ||
                    scan_msg->ranges[i] < scan_msg->range_min) {
                    continue;
                }

                float range_rate = (scan_msg->ranges[i] - prev_ranges[i]) / dt;
                range_rate = fmax(-range_rate, 0.0);
                TTC_1 = scan_msg->ranges[i] / range_rate;

                if (TTC_1 <= TTC_threshold_1) {
                    ackermann_drive_pub->publish(ackermann_drive_msg);
                    RCLCPP_INFO(this->get_logger(), "Speed = %f, dt = %f, range_rate = %f, the range left = %f, iTTC = %f: causing emergency braking...",
                                this->speed, dt, range_rate, scan_msg->ranges[i], TTC_1);
                    break;
                }

            } else if (method == MethodType::Method2) {
                /** =====================================================================================================
                 *      This is the second method to do AEB. but only considering self-absolute velocity
                 *  =====================================================================================================
                 */
                float speed_projection = speed * cos(scan_msg->angle_min + scan_msg->angle_increment * i);
                if (speed_projection <= 0.0) {
                    continue;
                }
                TTC_2 = scan_msg->ranges.at(i) / speed_projection;

                /// TODO: publish drive/brake message
                if (TTC_2 <= TTC_threshold_2) {
                    ackermann_drive_pub->publish(ackermann_drive_msg);
                    RCLCPP_INFO(this->get_logger(), "Speed = %f, iTTC = %f: causing emergency braking...", this->speed,
                                TTC_2);
                    break;
                }
            }
        }
        prev_ranges = scan_msg->ranges;
        prev_t = curr_t;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
