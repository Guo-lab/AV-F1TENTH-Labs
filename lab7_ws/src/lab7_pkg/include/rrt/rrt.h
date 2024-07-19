// RRT assignment

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <nav_msgs/msg/path.hpp>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

class TimeNode : public rclcpp::Node {
   public:
    TimeNode() : Node("time_node") { clock_ = this->get_clock(); }

    auto GetCurrentTime() -> double {
        current_time = clock_->now().seconds();
        prev_time = current_time;
        return current_time;
    }
    auto Timeout() -> bool { return clock_->now().seconds() - prev_time > 3.0; }

   private:
    rclcpp::Clock::SharedPtr clock_;
    double current_time = 0.0;
    double prev_time = 0.0;
};

class WaypointsReader {
   public:
    WaypointsReader(std::vector<std::pair<double, double>>& waypoints) {
        std::string file_path = "./src/lab7_pkg/data/waypoints.csv";
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

// Struct defining the RRT_Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct RRT_Node {
    double x, y;
    double cost;  // only used for RRT*
    int parent;   // index of parent node in the tree vector
    bool is_root = false;
} RRT_Node;

class WaypointVisualizer {
    /**
     * Message Filter dropping message: frame 'base_link' at time 1721112901.771 for reason 'discarding message because
     * the queue is full' To solve in the future.
     */
   public:
    WaypointVisualizer() {}
    void update_visualization(const std::vector<RRT_Node>& path_in_tree) {
        this->path_to_visualize = path_in_tree;
        // std::cout << "size: " << this->path_to_visualize.size() << std::endl;

        visualize_waypoints();
    }
    nav_msgs::msg::Path path_msg;

   private:
    std::vector<RRT_Node> path_to_visualize;

    void visualize_waypoints() {
        if (path_to_visualize.empty()) {
            return;
        }

        path_msg.poses.clear();

        for (auto each : path_to_visualize) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = rclcpp::Clock().now();
            pose.header.frame_id = "map";
            pose.pose.position.x = each.x;
            pose.pose.position.y = each.y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        path_msg.header = path_msg.poses[0].header;
    }
};

class RRT : public rclcpp::Node {
   public:
    RRT();
    virtual ~RRT();

   private:
    // TODO: add the publishers and subscribers you need

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_drive_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    std::shared_ptr<TimeNode> timer_node_ = std::make_shared<TimeNode>();

    string pose_topic = "/ego_racecar/odom";
    string scan_topic = "/scan";
    string drive_topic = "/drive";
    string occupancy_grid_topic = "/local_occupancy_grid";
    string path_topic = "/path";

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;

    nav_msgs::msg::OccupancyGrid occupancy_grid_;
    int occupancy_grid_width_ = 400;
    int occupancy_grid_height_ = 400; // 4 meters for the height
    double occupancy_grid_resolution_ = 0.01; // 1 cm per cell, 100 cells in a meter
    int x_offset_ = 0; // -1 * occupancy_grid_width_ * occupancy_grid_resolution_ * 1/4;
    int y_offset_ = -1 * occupancy_grid_height_ * occupancy_grid_resolution_ * 1/2;

    geometry_msgs::msg::Pose robot_pose_;
    nav_msgs::msg::Odometry robot_odom_;

    std::vector<std::pair<double, double>> waypoints;
    WaypointVisualizer waypoint_visualizer;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // RRT variables
    bool initialized_ = false;
    bool no_waypoints_ = true;
    double max_expansion_dist_ = 1.0f;
    double goal_threshold_ = 0.15f;

    // callbacks
    // where rrt actually happens
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    // RRT methods
    std::vector<double> sample();
    int nearest(std::vector<RRT_Node>& tree, std::vector<double>& sampled_point);
    RRT_Node steer(RRT_Node& nearest_node, std::vector<double>& sampled_point);

    bool check_collision(RRT_Node& nearest_node, RRT_Node& new_node);
    bool line_of_sight(int max_iter, int x0, int y0, int x1, int y1, int err, int dx, int dy, int sx, int sy);
    bool is_goal(RRT_Node& latest_added_node, double goal_x, double goal_y);
    std::vector<RRT_Node> find_path(std::vector<RRT_Node>& tree, RRT_Node& latest_added_node);

    auto get_speed_based_on_angle(double steering_angle) -> double;
    // void actuator(std::vector<RRT_Node>& path);

    // RRT* methods
    double cost(std::vector<RRT_Node>& tree, RRT_Node& node);
    double line_cost(RRT_Node& n1, RRT_Node& n2);
    std::vector<int> near(std::vector<RRT_Node>& tree, RRT_Node& node);
};
