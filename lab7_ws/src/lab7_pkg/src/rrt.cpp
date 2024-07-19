// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT() : rclcpp::Node("rrt_node"), gen((std::random_device())()) {
    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need

    ackermann_drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(occupancy_grid_topic, 1);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic, 1);

    // ROS subscribers
    // TODO: create subscribers as you need
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));

    // TODO: create a occupancy grid
    occupancy_grid_.header.frame_id = "map";
    occupancy_grid_.header.stamp = this->now();

    occupancy_grid_.info.width = 800;  // 8 meters for the width
    occupancy_grid_.info.height = 800;
    occupancy_grid_.info.resolution = 0.05;  // 5 cm per cell, 10 cells in a meter

    occupancy_grid_.info.origin.position.x = -5.0;
    occupancy_grid_.info.origin.position.y = -5.0;
    occupancy_grid_.info.origin.position.z = 0.0;

    occupancy_grid_.info.origin.orientation.x = 0.0;
    occupancy_grid_.info.origin.orientation.y = 0.0;
    occupancy_grid_.info.origin.orientation.z = 0.0;
    occupancy_grid_.info.origin.orientation.w = 1.0;

    std::vector<int8_t> data(occupancy_grid_.info.width * occupancy_grid_.info.height, 50);
    occupancy_grid_.data = data;

    occupancy_grid_pub_->publish(occupancy_grid_);

    // Load waypoints
    WaypointsReader waypoints_reader(waypoints);
    if (!waypoints.empty()) {
        no_waypoints_ = false;
    }
    std::cout << "Waypoints loaded: " << waypoints.size() << std::endl;

    // Prepare for the coordination transformation
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");
}

/**
 * @brief Callback function for the laser scan message, which will be used to update the occupancy grid.
 *  Occupancy grid should attach to the robot. Thus, here there is also a transformation from the the robot odom frame
 *      to the map frame. Finally, the grid will be together with the robot.
 *  Traverse all laser points. For each scan laser point, convert the them from their laser frame to the global map
 *      frame.
 *
 * @param scan_msg The incoming laser scan message
 */
void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //

    /**
     * occupancy_grid_.header.frame_id, robot_odom_.child_frame_id, 
     *  and scan_msg->header.frame_id might be empty?
     */

    // Get the robot's pose in the map frame (actually the same here since robot odom has reference frame: map)
    geometry_msgs::msg::TransformStamped base_to_map_transform;
    try {
        base_to_map_transform =
            tf_buffer_->lookupTransform("map", "ego_racecar/base_link", rclcpp::Time(0), tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform from base_link to map: %s", ex.what());
        return;
    }
    occupancy_grid_.header.stamp = this->now();
    occupancy_grid_.info.origin.position.x = base_to_map_transform.transform.translation.x -
                                             (occupancy_grid_.info.width * occupancy_grid_.info.resolution) / 2;
    occupancy_grid_.info.origin.position.y = base_to_map_transform.transform.translation.y -
                                             (occupancy_grid_.info.height * occupancy_grid_.info.resolution) / 2;

    std::vector<int8_t> data(occupancy_grid_.info.width * occupancy_grid_.info.height, 50);
    occupancy_grid_.data = data;

    geometry_msgs::msg::TransformStamped laser_to_map_transform;
    // std::cout << "Occupancy Grid Frame: " << occupancy_grid_.header.frame_id << std::endl;
    try {
        laser_to_map_transform =
            tf_buffer_->lookupTransform("map", "ego_racecar/laser", rclcpp::Time(0), tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform from laser to map: %s", ex.what());
        return;
    }

    auto scan_ranges = scan_msg->ranges;

    // TODO: update your occupancy grid
    for (int i = 0; i < (int)scan_ranges.size(); i++) {
        if (isnan(scan_ranges[i]) || isinf(scan_ranges[i])) {
            continue;
        }
        double each_angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        double each_range = scan_msg->ranges[i];

        double laser_x = each_range * std::cos(each_angle);
        double laser_y = each_range * std::sin(each_angle);

        geometry_msgs::msg::PointStamped laser_point;
        laser_point.header.frame_id = scan_msg->header.frame_id;
        laser_point.point.x = laser_x;
        laser_point.point.y = laser_y;
        laser_point.point.z = 0.0;

        geometry_msgs::msg::PointStamped map_point;
        try {
            tf2::doTransform(laser_point, map_point, laser_to_map_transform);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform point from laser to map: %s", ex.what());
            continue;
        }

        int grid_x = (map_point.point.x - occupancy_grid_.info.origin.position.x) / occupancy_grid_.info.resolution;
        int grid_y = (map_point.point.y - occupancy_grid_.info.origin.position.y) / occupancy_grid_.info.resolution;

        if (grid_x >= 0 && grid_x < (int)occupancy_grid_.info.width && grid_y >= 0 &&
            grid_y < (int)occupancy_grid_.info.height) {
            int index = grid_y * occupancy_grid_.info.width + grid_x;
            if (each_range < scan_msg->range_max && each_range > scan_msg->range_min) {
                occupancy_grid_.data[index] = 100;
            } else {
                occupancy_grid_.data[index] = 0;
            }
        }
    }

    occupancy_grid_pub_->publish(occupancy_grid_);
    std::cout << "Occupancy grid updated" << std::endl;
}

void RRT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //
    robot_pose_ = pose_msg->pose.pose;
    robot_odom_ = *pose_msg;

    /** =============== Waypoints Fetching ============== */
    if (no_waypoints_) return;

    // tree as std::vector
    std::vector<RRT_Node> tree;
    tree.clear();

    // if (!initialized_) {
    //     initialized_ = true;
    // }
    std::cout << "Robot pose: " << robot_pose_.position.x << ", " << robot_pose_.position.y << std::endl;

    /** =================  RRT Building  ================= */
    RRT_Node root{robot_pose_.position.x, robot_pose_.position.y, 0, -1, true};
    tree.push_back(root);

    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Root node added");

    // TODO: fill in the RRT main loop
    bool if_path_found = false;
    timer_node_->GetCurrentTime();

    while (!if_path_found) {

        if (timer_node_->Timeout()) {
            RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Time out, no path found.");
            break;
        }
        std::vector<double> sampled_points = sample();

        int nearest_node_idx = nearest(tree, sampled_points);
        RRT_Node nearest_node = tree[nearest_node_idx];

        RRT_Node new_node = steer(nearest_node, sampled_points);

        if (!check_collision(nearest_node, new_node)) {
            tree.push_back(new_node);

            if (is_goal(new_node, waypoints[0].first, waypoints[0].second)) {
                
                timer_node_->GetCurrentTime();
                if_path_found = true;
                std::vector<RRT_Node> path = find_path(tree, new_node);

                // actuator(path);
                
                // RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Path found");
                std::cout << path.size() << std::endl;
                
                // waypoint_visualizer.update_visualization(path);
                // path_pub_->publish(waypoint_visualizer.path_msg);
            }
        }

        // std::cout << "Tree size: " << tree.size() << std::endl;
    }

    // path found as Path message
}

/**
 * ================================================================================================
 *                                     RRT methods
 * ================================================================================================
 */
std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    // RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Sampling point");

    std::vector<double> sampled_point;
    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)

    double range_x_lower = robot_pose_.position.x - 5.0;
    double range_x_upper = robot_pose_.position.x + 5.0;
    double range_y_lower = robot_pose_.position.y - 5.0;
    double range_y_upper = robot_pose_.position.y + 5.0;

    x_dist = std::uniform_real_distribution<>(range_x_lower, range_x_upper);
    y_dist = std::uniform_real_distribution<>(range_y_lower, range_y_upper);

    sampled_point.push_back(x_dist(gen));
    sampled_point.push_back(y_dist(gen));

    return sampled_point;
}

int RRT::nearest(std::vector<RRT_Node>& tree, std::vector<double>& sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<RRT_Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    double min_dist = std::numeric_limits<double>::max();

    // TODO: fill in this method
    for (int i = 0; i < (int)tree.size(); i++) {
        double distance =
            std::sqrt(std::pow(tree[i].x - sampled_point[0], 2) + std::pow(tree[i].y - sampled_point[1], 2));
        if (distance < min_dist) {
            nearest_node = i;
            min_dist = distance;
        }
    }

    return nearest_node;
}

RRT_Node RRT::steer(RRT_Node& nearest_node, std::vector<double>& sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” to y than x is.
    // The point z returned by the function steer will be such that z minimizes ||z−y|| while at the same time
    // maintaining ||z−x|| <= max_expansion_dist,
    // for a prespecified max_expansion_dist > 0 basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (RRT_Node): new node created from steering

    // RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Steering");

    RRT_Node new_node;

    // TODO: fill in this method
    double dx = sampled_point[0] - nearest_node.x;
    double dy = sampled_point[1] - nearest_node.y;
    double dist = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

    if (dist > max_expansion_dist_) {
        dx = dx * max_expansion_dist_ / dist;
        dy = dy * max_expansion_dist_ / dist;
    }

    new_node.x = nearest_node.x + dx;
    new_node.y = nearest_node.y + dy;
    new_node.is_root = false;

    return new_node;
}

bool RRT::line_of_sight(int max_iter, int x0, int y0, int x1, int y1, int err, int dx, int dy, int sx, int sy) {
    for (int i = 0; i < max_iter; i++) {
        if (occupancy_grid_.data[y0 * occupancy_grid_.info.width + x0] == 100) {
            return false;
        }
        if (x0 == x1 && y0 == y1) return true;
        int e2 = 2 * err;
        if (e2 > -dy) err -= dy;
        if (e2 > -dy) x0 += sx;
        if (e2 < dx) err += dx;
        if (e2 < dx) y0 += sy;
    }
    return false;
}

bool RRT::check_collision(RRT_Node& nearest_node, RRT_Node& new_node) {
    // This method returns a boolean indicating if the path between the nearest node and the new node created from
    // steering is collision free
    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    new_node (RRT_Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;
    // TODO: fill in this method
    int x0 = (nearest_node.x - occupancy_grid_.info.origin.position.x) / occupancy_grid_.info.resolution;
    int y0 = (nearest_node.y - occupancy_grid_.info.origin.position.y) / occupancy_grid_.info.resolution;
    int x1 = (new_node.x - occupancy_grid_.info.origin.position.x) / occupancy_grid_.info.resolution;
    int y1 = (new_node.y - occupancy_grid_.info.origin.position.y) / occupancy_grid_.info.resolution;
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int max_iter_step = std::ceil(max_expansion_dist_ / occupancy_grid_.info.resolution);

    /** Direction indicator */
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;

    collision = !line_of_sight(max_iter_step, x0, y0, x1, y1, err, dx, dy, sx, sy);

    // RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", collision ? "Collision" : "No collision");
    return collision;
}

bool RRT::is_goal(RRT_Node& latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close enough (defined by goal_threshold) to the goal
    // so we can terminate the search and find a path
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TODO: fill in this method
    double distance = std::sqrt(std::pow(latest_added_node.x - goal_x, 2) + std::pow(latest_added_node.y - goal_y, 2));
    // RCLCPP_INFO(rclcpp::get_logger("RRT"), "Distance to goal: %f\n", distance);

    if (distance < goal_threshold_) {
        close_enough = true;
    }
    return close_enough;
}

std::vector<RRT_Node> RRT::find_path(std::vector<RRT_Node>& tree, RRT_Node& latest_added_node) {
    // This method traverses the tree from the node that has been determined as goal
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<RRT_Node>): the vector that represents the order of
    //      of the nodes traversed as the found path

    std::vector<RRT_Node> found_path;

    // TODO: fill in this method
    RRT_Node curr_node = latest_added_node;

    while (!curr_node.is_root) {
        found_path.push_back(curr_node);
        curr_node = tree[curr_node.parent];
    }

    found_path.push_back(curr_node);
    std::reverse(found_path.begin(), found_path.end());

    return found_path;
}

auto RRT::get_speed_based_on_angle(double steering_angle) -> double {
    double abs_angle = std::abs(steering_angle);
    if (abs_angle >= 0.0 * (M_PI / 180.0) && abs_angle <= 10.0 * (M_PI / 180.0)) {
        return 2;
    }
    if (abs_angle > 10.0 * (M_PI / 180.0) && abs_angle <= 20.0 * (M_PI / 180.0)) {
        return 1;
    }
    return 0.5;
}

// void RRT::actuator(std::vector<RRT_Node>& path) {
//     // This method publishes the path as a series of waypoints to the path topic
//     // Args:
//     //   path (std::vector<RRT_Node>): the vector that represents the order of
//     //      of the nodes traversed as the found path
//     // Returns:
//     //
// }

/**
 *  ================================================================================================
 *                                      RRT* methods
 *  ================================================================================================
 */
// double RRT::cost(std::vector<RRT_Node>& tree, RRT_Node& node) {
//     // This method returns the cost associated with a node
//     // Args:
//     //    tree (std::vector<RRT_Node>): the current tree
//     //    node (RRT_Node): the node the cost is calculated for
//     // Returns:
//     //    cost (double): the cost value associated with the node

//     double cost = 0;
//     // TODO: fill in this method

//     return cost;
// }

// double RRT::line_cost(RRT_Node& n1, RRT_Node& n2) {
//     // This method returns the cost of the straight line path between two nodes
//     // Args:
//     //    n1 (RRT_Node): the RRT_Node at one end of the path
//     //    n2 (RRT_Node): the RRT_Node at the other end of the path
//     // Returns:
//     //    cost (double): the cost value associated with the path

//     double cost = 0;
//     // TODO: fill in this method

//     return cost;
// }

// std::vector<int> RRT::near(std::vector<RRT_Node>& tree, RRT_Node& node) {
//     // This method returns the set of Nodes in the neighborhood of a
//     // node.
//     // Args:
//     //   tree (std::vector<RRT_Node>): the current tree
//     //   node (RRT_Node): the node to find the neighborhood for
//     // Returns:
//     //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

//     std::vector<int> neighborhood;
//     // TODO:: fill in this method

//     return neighborhood;
// }