// RRT assignment

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <image_transport/image_transport.hpp>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

/**
 * @brief This class is a reader for reading CSV files containing the waypoints
 *  of the path to follow.
 *
 * The waypoints will be extracted to the vector `waypoints`
 */
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

/**
 * @brief This class is used for visualizing the waypoints, goal point, the RRT algorithm's tree and path.
 *  It maintains all messages for the RRT to call publisher to publish.
 */
class WaypointVisualizer {
   public:
    WaypointVisualizer() {}
    /**
     * @brief: Update the visualization of the waypoints and the tree, and the path.
     *  Then, wait RRT node to publish these visualization message
     */
    void update_visualization(const std::vector<RRT_Node>& path_in_tree, const std::vector<RRT_Node>& tree) {
        this->path_to_visualize = path_in_tree;
        this->tree_to_visualize = tree;
        visualize_waypoints();
        visualize_tree();
    }

    nav_msgs::msg::Path path_msg;
    visualization_msgs::msg::MarkerArray tree_msg;
    visualization_msgs::msg::Marker current_waypoint;
    visualization_msgs::msg::Marker goal_waypoint;

    std::vector<RRT_Node> path_to_visualize;
    std::vector<RRT_Node> tree_to_visualize;

    /**
     * @brief: visualize_goal is used for visualizing the goal waypoint.
     *  This function only called once each time the RRT is finding the tree and path, when there are still waypoints.
     * 
     * @param: goal_x, which is the x of the last point among all waypoints
     * @param: goal_y, which is the y (in the global reference frame)
     */
    void visualize_goal(double goal_x, double goal_y) {
        goal_waypoint.lifetime = rclcpp::Duration::from_seconds(0.5);
        goal_waypoint.type = visualization_msgs::msg::Marker::CYLINDER;
        goal_waypoint.action = visualization_msgs::msg::Marker::ADD;
        goal_waypoint.id = 0;
        goal_waypoint.scale.x = 0.2;
        goal_waypoint.scale.y = 0.2;
        goal_waypoint.scale.z = 0.5;
        goal_waypoint.color.g = 0.0f;
        goal_waypoint.color.a = 1.0;  // Don't forget to set the alpha!
        goal_waypoint.color.r = 1.0;
        goal_waypoint.color.b = 0.0;
        goal_waypoint.header.frame_id = "map";
        goal_waypoint.pose.position.z = 0;
        goal_waypoint.pose.position.x = goal_x;  // waypoints.back().first;
        goal_waypoint.pose.position.y = goal_y;  // waypoints.back().second;
    }

   private:
    void visualize_waypoints() {
        if (path_to_visualize.empty()) return;

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

        current_waypoint.lifetime = rclcpp::Duration::from_seconds(0.5);
        current_waypoint.type = visualization_msgs::msg::Marker::SPHERE;
        current_waypoint.action = visualization_msgs::msg::Marker::ADD;
        current_waypoint.id = 0;
        current_waypoint.scale.x = 0.2;
        current_waypoint.scale.y = 0.2;
        current_waypoint.scale.z = 0.2;
        current_waypoint.color.g = 1.0f;
        current_waypoint.color.a = 1.0;  // Don't forget to set the alpha!
        current_waypoint.color.r = 0.0;
        current_waypoint.color.b = 0.0;
        current_waypoint.header.frame_id = "map";
        current_waypoint.pose.position.x = path_to_visualize.back().x;
        current_waypoint.pose.position.y = path_to_visualize.back().y;
        current_waypoint.pose.position.z = 0;
        current_waypoint.header.stamp = rclcpp::Clock().now();
    }

    void visualize_tree() {
        if (tree_to_visualize.empty()) return;

        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker nodes_marker;
        nodes_marker.header.frame_id = "map";
        nodes_marker.header.stamp = rclcpp::Clock().now();
        nodes_marker.ns = "nodes";
        nodes_marker.id = 0;
        nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        nodes_marker.action = visualization_msgs::msg::Marker::ADD;
        nodes_marker.pose.orientation.w = 1.0;
        nodes_marker.scale.x = 0.01;
        nodes_marker.scale.y = 0.01;
        nodes_marker.scale.z = 0.01;
        nodes_marker.color.a = 1.0;
        nodes_marker.color.r = 0.0;
        nodes_marker.color.g = 0.0;
        nodes_marker.color.b = 1.0;

        visualization_msgs::msg::Marker edges_marker;
        edges_marker.header.frame_id = "map";
        edges_marker.header.stamp = rclcpp::Clock().now();
        edges_marker.ns = "edges";
        edges_marker.id = 1;
        edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::msg::Marker::ADD;
        edges_marker.pose.orientation.w = 1.0;
        edges_marker.scale.x = 0.01;
        edges_marker.color.a = 1.0;
        edges_marker.color.r = 0.0;
        edges_marker.color.g = 0.0;
        edges_marker.color.b = 1.0;

        for (const auto& node : tree_to_visualize) {
            geometry_msgs::msg::Point p;
            p.x = node.x;
            p.y = node.y;
            p.z = 0.0;
            nodes_marker.points.push_back(p);
            if (!node.is_root) {
                geometry_msgs::msg::Point p2;
                p2.x = tree_to_visualize[node.parent].x;
                p2.y = tree_to_visualize[node.parent].y;
                p2.z = 0.0;
                edges_marker.points.push_back(p);
                edges_marker.points.push_back(p2);
            }
        }
        marker_array.markers.push_back(nodes_marker);
        marker_array.markers.push_back(edges_marker);

        tree_msg = marker_array;
    }
};

class RRT : public rclcpp::Node {
   public:
    RRT();
    virtual ~RRT();

   private:
    ackermann_msgs::msg::AckermannDriveStamped drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

    // TODO: add the publishers and subscribers you need

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_drive_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub;

    string pose_topic = "/ego_racecar/odom";
    string scan_topic = "/scan";
    string drive_topic = "/drive";
    string occupancy_grid_topic = "/local_occupancy_grid";
    string path_topic = "/path";
    string tree_topic = "/tree";
    string visualization_marker_topic = "/visualization_marker";
    string goal_marker_topic = "/goal_marker";

    string goal_frame_id = "map";
    string robot_frame_id = "ego_racecar/base_link";
    string laser_frame_id = "ego_racecar/laser";

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;

    nav_msgs::msg::OccupancyGrid occupancy_grid_;
    int occupancy_grid_width_ = 100;
    int occupancy_grid_height_ = 100;          // 4 meters for the height
    double occupancy_grid_resolution_ = 0.04;  // 4 cm per cell, 25 cells in a meter
    int x_offset_ = 0;  // -1 * occupancy_grid_width_ * occupancy_grid_resolution_ * 1 / 2;                         //
                        // -1 * occupancy_grid_width_ * occupancy_grid_resolution_ * 1/4;
    int y_offset_ = -1 * occupancy_grid_height_ * occupancy_grid_resolution_ * 1 / 2;
    int robot_radius_ = 5;
    int8_t obstacle_free_cost_ = 0;
    int8_t obstacle_cost_ = 100;

    geometry_msgs::msg::Pose robot_pose_;
    nav_msgs::msg::Odometry robot_odom_;

    std::vector<std::pair<double, double>> waypoints;
    WaypointVisualizer waypoint_visualizer;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // RRT variables
    double max_expansion_dist_ = 1.0f;
    double goal_threshold_ = 0.15f;
    double goal_reached_threshold_ = 0.6f;
    
    int max_tree_size_ = 300000;


    // callbacks
    // where rrt actually happens
    void brake_vehicle();
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);

    void dilate_to_configuration_space();
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
    void actuator(std::vector<RRT_Node>& path);

    // RRT* methods
    double cost(std::vector<RRT_Node>& tree, RRT_Node& node);
    double line_cost(RRT_Node& n1, RRT_Node& n2);
    std::vector<int> near(std::vector<RRT_Node>& tree, RRT_Node& node);
};
