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
#include <nav_msgs/msg/path.hpp>
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
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

class TimeNode : public rclcpp::Node {
   public:
    TimeNode() : Node("time_node") { clock_ = this->get_clock(); }

    void SetCurrentTime() {
        current_time = clock_->now().seconds();
        prev_time = current_time;
    }
    auto Timeout() -> bool { return clock_->now().seconds() - prev_time > 10.0; }

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
   public:
    WaypointVisualizer() {}
    void update_visualization(const std::vector<RRT_Node>& path_in_tree, const std::vector<RRT_Node>& tree) {
        this->path_to_visualize = path_in_tree;
        this->tree_to_visualize = tree;
        visualize_waypoints();
        visualize_tree();
    }

    nav_msgs::msg::Path path_msg;
    visualization_msgs::msg::MarkerArray tree_msg;
    visualization_msgs::msg::Marker current_waypoint;

   private:
    std::vector<RRT_Node> path_to_visualize;
    std::vector<RRT_Node> tree_to_visualize;

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
        if (tree_to_visualize.empty()) {
            std::cout << "Tree is empty" << std::endl;
            return;
        }
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

// class RRT : public rclcpp::Node {
//    public:
//     RRT();
//     virtual ~RRT();

//    private:
//     ackermann_msgs::msg::AckermannDriveStamped drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

//     // TODO: add the publishers and subscribers you need

//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

//     rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_drive_pub;
//     rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
//     rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
//     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pub_;
//     rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
//     rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub;

//     std::shared_ptr<TimeNode> timer_node_ = std::make_shared<TimeNode>();

//     string pose_topic = "/ego_racecar/odom";
//     string scan_topic = "/scan";
//     string drive_topic = "/drive";
//     string occupancy_grid_topic = "/local_occupancy_grid";
//     string path_topic = "/path";
//     string tree_topic = "/tree";

//     // random generator, use this
//     std::mt19937 gen;
//     std::uniform_real_distribution<> x_dist;
//     std::uniform_real_distribution<> y_dist;

//     nav_msgs::msg::OccupancyGrid occupancy_grid_;
//     int occupancy_grid_width_ = 80;
//     int occupancy_grid_height_ = 80;           // 4 meters for the height
//     double occupancy_grid_resolution_ = 0.05;  // 5 cm per cell, 100 cells in a meter
//     int x_offset_ = 0;                         // -1 * occupancy_grid_width_ * occupancy_grid_resolution_ * 1/4;
//     int y_offset_ = -1 * occupancy_grid_height_ * occupancy_grid_resolution_ * 1 / 2;

//     geometry_msgs::msg::Pose robot_pose_;
//     nav_msgs::msg::Odometry robot_odom_;

//     std::vector<std::pair<double, double>> waypoints;
//     WaypointVisualizer waypoint_visualizer;

//     std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

//     // RRT variables
//     bool no_waypoints_ = true;
//     double max_expansion_dist_ = 0.5f;
//     double goal_threshold_ = 0.15f;

//     // callbacks
//     // where rrt actually happens
//     void brake_vehicle();
//     void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);

//     void dilate_to_configuration_space();
//     // updates occupancy grid
//     void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

//     // RRT methods
//     std::vector<double> sample();
//     int nearest(std::vector<RRT_Node>& tree, std::vector<double>& sampled_point);
//     RRT_Node steer(RRT_Node& nearest_node, std::vector<double>& sampled_point);

//     bool check_collision(RRT_Node& nearest_node, RRT_Node& new_node);
//     bool line_of_sight(int max_iter, int x0, int y0, int x1, int y1, int err, int dx, int dy, int sx, int sy);
//     bool is_goal(RRT_Node& latest_added_node, double goal_x, double goal_y);
//     std::vector<RRT_Node> find_path(std::vector<RRT_Node>& tree, RRT_Node& latest_added_node);

//     auto get_speed_based_on_angle(double steering_angle) -> double;
//     void actuator(std::vector<RRT_Node>& path);

//     // RRT* methods
//     double cost(std::vector<RRT_Node>& tree, RRT_Node& node);
//     double line_cost(RRT_Node& n1, RRT_Node& n2);
//     std::vector<int> near(std::vector<RRT_Node>& tree, RRT_Node& node);
// };

class RRT : public rclcpp::Node {
   public:
    RRT();
    virtual ~RRT();

   private:
    // add the publishers and subscribers you need

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // add drive publisher
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    // add path publisher for debug
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // add point publisher for debug
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;

    // add processed laser publisher for debug
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr processed_scan_pub_;

    // add occupancy grid publisher for debug
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;

    // add visualization marker array publisher for debug
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_marker_array_pub_;

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> sample_type;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;

    // constants for laser properties
    const double PI = 3.1415926536;
    const double angle_min = -2.3499999046325684;
    const double angle_max = 2.3499999046325684;
    const double angle_increment = 0.004351851996034384;

    // constants for tree expansion status
    const int TRAPPED = 0;
    const int ADVANCED = 1;
    const int REACHED = 2;

    // parameters for RRT
    int num_of_samples;
    double look_ahead_dist;
    double step_size;
    double goal_sample_rate;
    double angle_sample_range;
    double safety_padding;
    double disparity_extend_range;
    std::string waypoint_file_path;
    std::string waypoint_frame_id;
    std::string pose_to_listen;

    // define transform from/to frames and waypoint index
    std::string fromFrame;
    std::string toFrame;
    int last_waypoint_index;

    // define last heading angle
    double last_heading;

    // listener to listen updates on pose_to_listen
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // data structure to store
    // - path file
    // - path
    // - occupancy grid
    // - rrt visualization
    ifstream waypoint_file;
    nav_msgs::msg::Path path_msg;
    int path_length;
    nav_msgs::msg::OccupancyGrid rrt_grid;
    visualization_msgs::msg::MarkerArray rrt_marks;
    int marker_id;

    // parameters used for occupancy grid
    int grid_width;
    int grid_height;
    double grid_resolution;

    // vector for processed laser scan
    std::vector<float> processed_scan;

    // callbacks
    // where rrt actually happens
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    // RRT methods
    std::vector<double> sample(std::vector<double>& goal, bool goal_status);
    int nearest(std::vector<RRT_Node>& tree, std::vector<double>& sampled_point);
    int extend(std::vector<RRT_Node>& tree, int nearest_node_index, std::vector<double>& sampled_point,
               std::vector<double>& goal_point, bool goal_status);
    bool check_collision(RRT_Node& nearest_node, RRT_Node& new_node);
    bool is_goal(RRT_Node& latest_added_node, std::vector<double>& goal_point, bool goal_status);
    std::vector<RRT_Node> find_path(std::vector<RRT_Node>& tree, RRT_Node& latest_added_node);
    // RRT* methods
    double cost(std::vector<RRT_Node>& tree, RRT_Node& node);
    double line_cost(RRT_Node& n1, RRT_Node& n2);
    std::vector<int> near(std::vector<RRT_Node>& tree, RRT_Node& node);

    // helper functions
    void log_waypoints();
    void find_goal_point(std::vector<double>& goal_point);
    void interpolate_points(double x1, double y1, double x2, double y2, std::vector<double>& goal_point);
    double euclidean_dist(double x, double y);
    void process_scan(std::vector<float>& scan);
    void init_grid();
};