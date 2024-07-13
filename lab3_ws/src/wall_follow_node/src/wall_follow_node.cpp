#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

#include <cmath>
#define HALF_PI (M_PI / 2.0)

class TimeNode : public rclcpp::Node {
   public:
    TimeNode() : Node("time_node") { clock_ = this->get_clock(); }

    auto GetCurrentTime() -> double {
        current_time = clock_->now().seconds();
        prev_time = current_time;
        return current_time;
    }
    auto GetPrevTime() -> double { return prev_time; }

   private:
    rclcpp::Clock::SharedPtr clock_;
    double current_time = 0.0;
    double prev_time = 0.0;
};

class PIDController : public rclcpp::Node {
   public:
    PIDController(double kp, double ki, double kd)
        : Node("pid_controller_node"), kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {
        /**
         * https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller
         */
        timer_node_->GetCurrentTime();
    }
    /**
     * Give a steering angle for the VESC
     *
     * u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt
     * where u(t) is the steering angle we want the car to drive at;
     * The error term e(t) is the difference between the set point and the parameter we want to maintain around that
     * set point.
     */
    auto GetSteeringAngle(double error) -> double {
        double prev_time = timer_node_->GetPrevTime();
        double current_time = timer_node_->GetCurrentTime();
        double dt = current_time - prev_time;

        integral_ = integral_ + error * dt;
        double derivative = (error - prev_error_) / dt;

        prev_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    auto GetSpeed(double steering_angle) -> double {
        double abs_angle = std::abs(steering_angle);
        if (abs_angle >= 0.0 && abs_angle <= 10.0) {
            return 1.5;
        }
        if (abs_angle > 10.0 && abs_angle <= 20.0) {
            return 1.0;
        }
        return 0.5;
    }

   private:
    double kp_ = 0.0;  // Propotional gain: heavy lifting
    double ki_ = 0.0;  // Integral gain: reduce steady state residual error
    double kd_ = 0.0;  // Differential (Derivative) gain: reduce overshoot
    double prev_error_ = 0.0;
    double integral_ = 0.0;
    std::shared_ptr<TimeNode> timer_node_ = std::make_shared<TimeNode>();
};

class WallFollow : public rclcpp::Node {
   public:
    WallFollow() : Node("wall_follow_node") {
        // TODO: create ROS subscribers and publishers
        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, _1));

        ackermann_drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    }

   private:
    // PID CONTROL PARAMS
    // TODO: double kp =
    // TODO: double kd =
    // TODO: double ki =
    double theta = HALF_PI / 2;
    double alpha = 0.0;
    double AB = 0.0;
    double lookahead_distance = 0.8;
    double CD = 0.0;

    double error = 0.0;
    double integral = 0.0;
    double prev_error = 0.0;

    sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan_msg;

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_drive_pub;

    std::shared_ptr<PIDController> pid_controller = std::make_shared<PIDController>(1.5, 0.25, 0.05);

    double get_range(const float* range_data, double angle) {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs
        and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        // TODO: implement
        int index = (angle - laser_scan_msg->angle_min) / laser_scan_msg->angle_increment;
        if (index >= 0 && index < (int)laser_scan_msg->ranges.size()) {
            if (std::isfinite(range_data[index])) {
                return range_data[index];
            }
        }
        return std::numeric_limits<double>::infinity();
    }

    double get_error(const float* range_data, double dist) {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You
        potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */
        /** @ref: https://f1tenth-coursekit.readthedocs.io/en/latest/assignments/labs/lab3.html */
        double b_distance = get_range(range_data, HALF_PI);
        double a_distance = get_range(range_data, HALF_PI - theta);
        if (std::isnan(a_distance) || std::isnan(b_distance)) return 0.0;

        // TODO:implement
        alpha = std::atan((a_distance * std::cos(theta) - b_distance) / (a_distance * std::sin(theta)));
        AB = b_distance * std::cos(alpha);
        /**
         * the car will be traveling at a high speed and
         * therefore will have a non-instantaneous response to whatever speed and servo control we give to it
         */
        CD = AB + lookahead_distance * std::sin(alpha);
        return dist - CD;
    }

    void pid_control(double error, double velocity) {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        // TODO: Use kp, ki & kd to implement a PID controller
        double steering_angle = -1 * pid_controller->GetSteeringAngle(error);
        velocity = pid_controller->GetSpeed(steering_angle);  // TODO: calculate desired car velocity based on error

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

        // TODO: fill in drive message and publish
        drive_msg.drive.speed = velocity;
        drive_msg.drive.steering_angle = steering_angle;

        RCLCPP_INFO(this->get_logger(), "Speed = %f, Steering Angle = %f, Published...", velocity, steering_angle);
        ackermann_drive_pub->publish(drive_msg);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        laser_scan_msg = scan_msg;
        double error =
            get_error(laser_scan_msg->ranges.data(), 1);  // TODO: replace with error calculated by get_error()
        double velocity = 0.0;                            // TODO: calculate desired car velocity based on error
        pid_control(error, velocity);                     // TODO: actuate the car with PID
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}