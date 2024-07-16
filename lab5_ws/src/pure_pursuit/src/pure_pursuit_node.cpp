#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cassert>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
/// CHECK: include needed ROS msg type headers and libraries

#include <utility>

using namespace std;

class WaypointsReader {
   public:
    WaypointsReader(std::vector<std::pair<double, double>>& waypoints) {
        std::string file_path = "./src/pure_pursuit/data/waypoints.csv";
        read_waypoints_csv(file_path, waypoints);
    }

   private:
    void read_waypoints_csv(const std::string& file_path, std::vector<std::pair<double, double>>& waypoints) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            std::cout << "Could not open file: " << file_path.c_str() << std::endl;
            return;
        }
        std::string line;
        while (std::getline(file, line)) {
            auto waypoint = parse_csv_line(line);
            waypoints.push_back(waypoint);
            // std::cout << "Waypoint: x = " << waypoint.first << ", y = " << waypoint.second << std::endl;
        }
        file.close();
    }

    std::pair<double, double> parse_csv_line(const std::string& line) {
        std::stringstream ss(line);
        std::string token;
        std::pair<double, double> waypoint;
        if (std::getline(ss, token, ',')) {
            waypoint.first = std::stod(token);  // x-coordinate
        }
        if (std::getline(ss, token, ',')) {
            waypoint.second = std::stod(token);  // y-coordinate
        }
        return waypoint;
    }
};

/**
 * A geometric path tracking controller which relies on kinematic vehicle model for selecting commnads
 */
class PurePursuit : public rclcpp::Node {
    // Implement PurePursuit
    // This is just a template, you are free to implement your own node!

   private:
    ackermann_msgs::msg::AckermannDriveStamped drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    std::string odom_topic = "/ego_racecar/odom";
    std::string drive_topic = "/drive";

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_drive_pub;

    std::vector<std::pair<double, double>> waypoints;
    const double lookahead_distance = 1.0;

    /**
     * @brief Interpolate between two waypoints
     *  |==|----|        |
        |==|----|--------|
     */
    auto interpolate_waypoint(const pair<double, double>& inner, const pair<double, double>& outer, double t)
        -> pair<double, double> {
        double x = inner.first + t * (outer.first - inner.first);
        double y = inner.second + t * (outer.second - inner.second);
        return make_pair(x, y);
    }

    auto inline euclidean_distance(const pair<double, double>& p1, const pair<double, double>& p2) -> double {
        return sqrt(pow(p1.first - p2.first, 2) + pow(p1.second - p2.second, 2));
    }

    /**
     * @brief Get the current waypoint to track
     *
     * How to get this current waypoint:
     *  1. pick the closest waypoint.
     *  2. Go up to the waypoint until there is a waypoint L away from the robot
     *  3. Use that as the current waypoint
     *
     * Simplified:
     *  Try to find the farthest point in the L-radius circle
     */
    auto get_current_waypoint(const std::shared_ptr<const nav_msgs::msg::Odometry>& curr_pose)
        -> std::pair<double, double> {
        std::pair<double, double> curr_robot = std::make_pair(curr_pose->pose.pose.position.x, curr_pose->pose.pose.position.y);
        if (waypoints.empty()) {
            std::cout << "No waypoints found" << std::endl;
            return curr_robot;
        }

        std::pair<double, double> prev_wp;
        std::pair<double, double> curr_wp = waypoints.front();
        waypoints.erase(waypoints.begin());

        if (waypoints.size() == 1) {
            return curr_wp;
        }

        std::cout << "Current robot: " << curr_robot.first << ", " << curr_robot.second << std::endl;
        while (!waypoints.empty()) {
            prev_wp = curr_wp;
            curr_wp = waypoints.front();
            double prev_dist = euclidean_distance(curr_robot, prev_wp);
            double curr_dist = euclidean_distance(curr_robot, curr_wp);
            double t;

            if (prev_dist < lookahead_distance && curr_dist < lookahead_distance) {
                waypoints.erase(waypoints.begin());
                continue;

            } else if (prev_dist < lookahead_distance && curr_dist > lookahead_distance) {
                t = (lookahead_distance - prev_dist) / (curr_dist - prev_dist);
                return interpolate_waypoint(prev_wp, curr_wp, t);

            } else if (prev_dist > lookahead_distance && curr_dist < lookahead_distance) {
                t = (lookahead_distance - curr_dist) / (prev_dist - curr_dist);
                waypoints.insert(waypoints.begin(), prev_wp);
                return interpolate_waypoint(prev_wp, curr_wp, t);

            } else if (prev_dist > lookahead_distance && curr_dist > lookahead_distance) {
                t = lookahead_distance / prev_dist;
                return interpolate_waypoint(curr_robot, prev_wp, t);

            } else {
                waypoints.erase(waypoints.begin());
                return prev_dist == lookahead_distance ? prev_wp : curr_wp;
            }
        }
        return curr_wp;
    }

    auto GetSpeed(double steering_angle) -> double {
        double abs_angle = std::abs(steering_angle);
        if (abs_angle >= 0.0 * (M_PI / 180.0) && abs_angle <= 10.0 * (M_PI / 180.0)) {
            return 3;
        }
        if (abs_angle > 10.0 * (M_PI / 180.0) && abs_angle <= 20.0 * (M_PI / 180.0)) {
            return 2;
        }
        return 1;
    }

   public:
    PurePursuit() : Node("pure_pursuit_node") {
        WaypointsReader waypoints_reader(waypoints);
        std::cout << "Waypoints loaded: " << waypoints.size() << std::endl;

        // TODO: create ROS subscribers and publishers
        pose_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, std::bind(&PurePursuit::pose_callback, this, placeholders::_1));

        ackermann_drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    }

    void pose_callback(const std::shared_ptr<const nav_msgs::msg::Odometry>& pose_msg) {
        std::cout << "Pose callback" << std::endl;

        // TODO: find the current waypoint to track using methods mentioned in lecture
        std::pair<double, double> curr_waypoint = get_current_waypoint(pose_msg);

        // TODO: transform goal point to vehicle frame of reference
        geometry_msgs::msg::Pose current_pose = pose_msg->pose.pose;
        double dx = curr_waypoint.first - current_pose.position.x;
        double dy = curr_waypoint.second - current_pose.position.y;

        tf2::Quaternion q(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
                          current_pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        /** @ref:
         * https://www.formula1-dictionary.net/motions_of_f1_car.html
         *  https://shuffleai.blog/blog/Three_Methods_of_Vehicle_Lateral_Control.html
         */
        m.getRPY(roll, pitch, yaw);

        // double x_vehicle_frame = cos(yaw) * dx + sin(yaw) * dy;
        double y_vehicle_frame = -sin(yaw) * dx + cos(yaw) * dy;

        // TODO: calculate curvature/steering angle
        double curvature = 2 * y_vehicle_frame / (lookahead_distance * lookahead_distance);
        double steering_angle = curvature * 0.5;
        if (steering_angle > M_PI / 2) {
            steering_angle = M_PI / 2;
        } else if (steering_angle < -M_PI / 2) {
            steering_angle = -M_PI / 2;
        }

        // TODO: publish drive message, don't forget to limit the steering angle.
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = GetSpeed(steering_angle);

        ackermann_drive_pub->publish(drive_msg);
    }

    ~PurePursuit() {}
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}