// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

/**
 * @brief helper function to calculate the euclidean distance between two points
 *
 * @param p1 point in a pair containing x and y position
 * @param p2 the second point in a pair containing x and y position
 */
auto inline euclidean_distance(const pair<double, double>& p1, const pair<double, double>& p2) -> double {
    return sqrt(pow(p1.first - p2.first, 2) + pow(p1.second - p2.second, 2));
}

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
    tree_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(tree_topic, 1);

    marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(visualization_marker_topic, 1);
    goal_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(goal_marker_topic, 1);

    // ROS subscribers
    // TODO: create subscribers as you need
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));

    // TODO: create a occupancy grid
    occupancy_grid_.header.frame_id = robot_frame_id;
    occupancy_grid_.header.stamp = this->now();

    occupancy_grid_.info.width = occupancy_grid_width_;
    occupancy_grid_.info.height = occupancy_grid_height_;
    occupancy_grid_.info.resolution = occupancy_grid_resolution_;
    occupancy_grid_.info.map_load_time = this->now();

    occupancy_grid_.info.origin.position.x = x_offset_;
    occupancy_grid_.info.origin.position.y = y_offset_;
    occupancy_grid_.info.origin.position.z = 0.0;
    occupancy_grid_.info.origin.orientation.x = 0.0;
    occupancy_grid_.info.origin.orientation.y = 0.0;
    occupancy_grid_.info.origin.orientation.z = 0.0;
    occupancy_grid_.info.origin.orientation.w = 1.0;

    std::vector<int8_t> data(occupancy_grid_.info.width * occupancy_grid_.info.height, obstacle_free_cost_);
    occupancy_grid_.data = data;

    occupancy_grid_pub_->publish(occupancy_grid_);

    // Load waypoints
    WaypointsReader waypoints_reader(waypoints);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("RRT"), "Loaded " << waypoints.size() << " waypoints.");

    // Prepare for the coordination transformation
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");
}

/**
 * @brief dilate the occupancy grid from the workspace to configuration space
 *  With opencv2, thicken the obstacles
 */
void RRT::dilate_to_configuration_space() {
    cv::Mat map_mat =
        cv::Mat(this->occupancy_grid_height_, this->occupancy_grid_width_, CV_8U, occupancy_grid_.data.data()).clone();

    // Perform dilation or any other desired operation to thicken obstacles
    cv::Mat dilated_map;

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * robot_radius_ + 1, 2 * robot_radius_ + 1));
    cv::dilate(map_mat, dilated_map, element);

    // Convert back to std::vector<int8_t>
    std::vector<int8_t> vector_map(dilated_map.data, dilated_map.data + dilated_map.total());
    this->occupancy_grid_.data = vector_map;
}

/**
 * @brief Callback function for the laser scan message, which will be used to update the occupancy grid.
 *  Occupancy grid should attach to the robot. Thus, the grid will be together with the robot in the robot's reference
 * frame.
 *  Traverse all laser points. For each scan laser point, convert the them from their laser frame to the global
 * map frame. Based on these scna points, finding the obstacles and updating the local planner's map (occupancy grid)
 *
 * @param scan_msg The incoming laser scan message
 */
void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //
    occupancy_grid_.header.stamp = this->now();

    occupancy_grid_.info.origin.position.x = x_offset_;
    occupancy_grid_.info.origin.position.y = y_offset_;
    occupancy_grid_.info.origin.position.z = 0.0;
    occupancy_grid_.info.origin.orientation.x = 0.0;
    occupancy_grid_.info.origin.orientation.y = 0.0;
    occupancy_grid_.info.origin.orientation.z = 0.0;
    occupancy_grid_.info.origin.orientation.w = 1.0;

    occupancy_grid_.data.assign(occupancy_grid_width_ * occupancy_grid_height_, obstacle_free_cost_);

    geometry_msgs::msg::TransformStamped laser_to_map_transform;
    try {
        /**
         * occupancy_grid_.header.frame_id, robot_odom_.child_frame_id, & scan_msg->header.frame_id might be empty?
         * so here use the static parameters
         */
        laser_to_map_transform =
            tf_buffer_->lookupTransform(robot_frame_id, laser_frame_id, rclcpp::Time(0), tf2::durationFromSec(0.25));
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
        double each_range = scan_msg->ranges[i];

        double laser_x = each_range * std::cos(scan_msg->angle_min + i * scan_msg->angle_increment);
        double laser_y = each_range * std::sin(scan_msg->angle_min + i * scan_msg->angle_increment);

        geometry_msgs::msg::PointStamped laser_point;
        laser_point.header.frame_id = laser_frame_id;
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

        int width = occupancy_grid_.info.width;
        int height = occupancy_grid_.info.height;

        if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
            int index = grid_y * occupancy_grid_width_ + grid_x;
            if (index >= (int)occupancy_grid_.data.size()) {
                continue;
            }
            if (each_range < scan_msg->range_max && each_range > scan_msg->range_min) {
                occupancy_grid_.data[index] = obstacle_cost_;
            }
        }
    }
    dilate_to_configuration_space();
    occupancy_grid_pub_->publish(occupancy_grid_);
}

/**
 * @brief Brake the vehicle by setting the speed and steering angle to zero
 *  As a helper function, it only be called as AEB, rrt ending, or rrt timeout
 */
void RRT::brake_vehicle() {
    drive_msg.drive.steering_angle = 0.0;
    drive_msg.drive.speed = 0.0;

    ackermann_drive_pub->publish(drive_msg);
}

/**
 * @brief Callback function for the pose message, which will be used to update the robot's pose and run the RRT
 * algorithm
 *
 * In the RRT, before timeout, or path not found; rrt will sample the points around the robot, find the nearest point on
 * the tree to the sampled point, and steer the tree towards the sampled point if that sampled points is accessible.
 *
 * When finding a path on the tree to the waypoint, the robot will visualize the tree and path, and actuator will use
 * pure pursuit to drive the robot based on the path provided by the RRT.
 *
 * @param pose_msg The incoming pose message
 */
void RRT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //
    robot_pose_ = pose_msg->pose.pose;
    robot_odom_ = *pose_msg;

    /** ==================== Waypoints Fetching =================== */
    if (waypoints.empty()) {
        brake_vehicle();
        return;
    }

    /** ======== Checking if the robot reaches any waypoints ======= */
    std::pair<double, double> curr_wp = waypoints.front();
    while (euclidean_distance(curr_wp, std::make_pair(robot_pose_.position.x, robot_pose_.position.y)) <
           goal_reached_threshold_) {
        RCLCPP_INFO(rclcpp::get_logger("RRT"), "Reached waypoint: %f, %f\n", curr_wp.first, curr_wp.second);

        waypoints.erase(waypoints.begin());
        if (waypoints.empty()) {
            brake_vehicle();
            return;
        }
        curr_wp = waypoints.front();
    }

    waypoint_visualizer.visualize_goal(waypoints.back().first, waypoints.back().second);
    goal_marker_pub->publish(waypoint_visualizer.goal_waypoint);

    // tree as std::vector
    std::vector<RRT_Node> tree;
    tree.clear();

    /** ========================  RRT Building  ===================== */
    RRT_Node root{robot_pose_.position.x, robot_pose_.position.y, 0, -1, true};
    tree.push_back(root);

    // TODO: fill in the RRT main loop
    bool if_path_found = false;
    while (!if_path_found) {
        if (tree.size() > max_tree_size_) {  // including the timeout cases
            RCLCPP_WARN(rclcpp::get_logger("RRT"), "%s\n", "Tree size exceeds the maximum size.");
            brake_vehicle();
            break;
        }

        std::vector<double> sampled_points = sample();

        int nearest_node_idx = nearest(tree, sampled_points);
        RRT_Node nearest_node = tree[nearest_node_idx];
        RRT_Node new_node = steer(nearest_node, sampled_points);

        if (!check_collision(nearest_node, new_node)) {
            new_node.parent = nearest_node_idx;
            tree.push_back(new_node);

            if (is_goal(new_node, curr_wp.first, curr_wp.second)) {
                if_path_found = true;

                std::vector<RRT_Node> path = find_path(tree, new_node);

                waypoint_visualizer.update_visualization(path, tree);
                path_pub_->publish(waypoint_visualizer.path_msg);

                tree_pub_->publish(waypoint_visualizer.tree_msg);
                marker_pub->publish(waypoint_visualizer.current_waypoint);

                actuator(path);
            }
        } else {
            // RRT's tree growth
            waypoint_visualizer.update_visualization(waypoint_visualizer.path_to_visualize, tree);
            tree_pub_->publish(waypoint_visualizer.tree_msg);
        }
    }

    // path found as Path message
}

/**
 * ================================================================================================
 *                                     RRT methods
 * ================================================================================================
 */

/**
 * @brief Sample a point in the free space.
 *
 * What's noteworthy is that the sample point should be within a small region of interest around the car's current
 * position. Besides, its in the global map frame.
 */
std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;

    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)

    double hori = occupancy_grid_width_ * occupancy_grid_resolution_ / 2;
    double vert = occupancy_grid_height_ * occupancy_grid_resolution_ / 2;

    double range_x_lower = -hori;
    double range_x_upper = hori;
    double range_y_lower = -vert;
    double range_y_upper = vert;

    x_dist = std::uniform_real_distribution<>(range_x_lower, range_x_upper);
    y_dist = std::uniform_real_distribution<>(range_y_lower, range_y_upper);

    sampled_point.push_back(robot_pose_.position.x + x_dist(gen));
    sampled_point.push_back(robot_pose_.position.y + y_dist(gen));

    return sampled_point;
}

/**
 * @brief Find the nearest node on the tree to the sampled point
 */
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
        double distance = euclidean_distance(std::make_pair(tree[i].x, tree[i].y),
                                             std::make_pair(sampled_point[0], sampled_point[1]));
        if (distance < min_dist) {
            nearest_node = i;
            min_dist = distance;
        }
    }

    return nearest_node;
}

/**
 * @brief Steer the tree towards the sampled point:
 *  truncate the edge from tree to the sampled node in the tree to the max expansion distance if this edge is too long.
 */
RRT_Node RRT::steer(RRT_Node& nearest_node, std::vector<double>& sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” to y than x is.
    // The point z returned by the function steer will be such that z minimizes ||z−y|| while at the same time
    // maintaining ||z−x|| <= max_expansion_dist,
    // for a prespecified max_expansion_dist > 0 basically, expand the tree towards the sample point
    // (within a max dist)

    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (RRT_Node): new node created from steering

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

/**
 * @brief helper function to check if there are obstacle on the line from sampled point to the tree.
 *  Using Bresenham's line algorithm.
 */
bool RRT::line_of_sight(int max_iter, int x0, int y0, int x1, int y1, int err, int dx, int dy, int sx, int sy) {
    for (int i = 0; i < max_iter; i++) {
        int tmp_idx = y0 * (int)occupancy_grid_.info.width + x0;
        if (tmp_idx >= (int)occupancy_grid_.data.size()) {
            return false;
        }

        if (occupancy_grid_.data[tmp_idx] == obstacle_cost_) {
            return false;
        }

        if (x0 == x1 && y0 == y1) {
            return true;
        }

        int e2 = 2 * err;
        if (e2 > -dy) err -= dy;
        if (e2 > -dy) x0 += sx;
        if (e2 < dx) err += dx;
        if (e2 < dx) y0 += sy;
    }

    return false;
}

/**
 * @brief check_collision:
 *  Check if the path between the nearest node and the new node created from steering is collision free
 *  The points based on the global map frame should be transformed to the base_link robot's frame as the occupancy grid
 * is in that frame.'
 *
 * If not doing so. mismatching the reference frame cause the robot crashing. (several hours to fix this bug)
 */
bool RRT::check_collision(RRT_Node& nearest_node, RRT_Node& new_node) {
    // This method returns a boolean indicating if the path between the nearest node and the new node created from
    // steering is collision free
    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    new_node (RRT_Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;

    geometry_msgs::msg::TransformStamped map_to_robot_transform;
    try {
        map_to_robot_transform =
            tf_buffer_->lookupTransform(robot_frame_id, goal_frame_id, rclcpp::Time(0), tf2::durationFromSec(0.25));
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        return false;
    }
    geometry_msgs::msg::PointStamped rrt_point;
    rrt_point.header.frame_id = goal_frame_id;
    rrt_point.point.x = nearest_node.x;
    rrt_point.point.y = nearest_node.y;
    rrt_point.point.z = 0.0;

    geometry_msgs::msg::PointStamped nearest_point;
    try {
        tf2::doTransform(rrt_point, nearest_point, map_to_robot_transform);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform point: %s", ex.what());
        return false;
    }

    rrt_point.point.x = new_node.x;
    rrt_point.point.y = new_node.y;

    geometry_msgs::msg::PointStamped new_point;
    try {
        tf2::doTransform(rrt_point, new_point, map_to_robot_transform);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform point: %s", ex.what());
        return false;
    }

    // TODO: fill in this method
    int x0 = (nearest_point.point.x - occupancy_grid_.info.origin.position.x) / occupancy_grid_.info.resolution;
    int y0 = (nearest_point.point.y - occupancy_grid_.info.origin.position.y) / occupancy_grid_.info.resolution;
    int x1 = (new_point.point.x - occupancy_grid_.info.origin.position.x) / occupancy_grid_.info.resolution;
    int y1 = (new_point.point.y - occupancy_grid_.info.origin.position.y) / occupancy_grid_.info.resolution;
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int max_iter_step = std::ceil(max_expansion_dist_ / occupancy_grid_.info.resolution);

    /** Direction indicator */
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;

    collision = !line_of_sight(max_iter_step, x0, y0, x1, y1, err, dx, dy, sx, sy);

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
    double distance =
        euclidean_distance(std::make_pair(latest_added_node.x, latest_added_node.y), std::make_pair(goal_x, goal_y));

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

/**
 * @brief get_speed_based_on_angle:
 *  The speed is based on the steering angle. The larger the steering angle, the slower the speed.
 */
auto RRT::get_speed_based_on_angle(double steering_angle) -> double {
    double abs_angle = std::abs(steering_angle);
    if (abs_angle >= 0.0 * (M_PI / 180.0) && abs_angle <= 5.0 * (M_PI / 180.0)) {
        return 3.0;
    }
    if (abs_angle > 5.0 * (M_PI / 180.0) && abs_angle <= 10.0 * (M_PI / 180.0)) {
        return 1.0;
    }
    if (abs_angle > 10.0 * (M_PI / 180.0) && abs_angle <= 20.0 * (M_PI / 180.0)) {
        return 0.5;
    }
    return 0.3;
}

/**
 * @brief actuator: which will use pure pursuit to drive the robot based on the path provided by the RRT
 *  Here just using the first sub waypoint givne by the path.
 */
void RRT::actuator(std::vector<RRT_Node>& path) {
    // This method publishes the path as a series of waypoints to the path topic
    // Args:
    //   path (std::vector<RRT_Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    // Returns:
    //

    double next_rrt_node_dist = 0.0;
    double dx = 0.0, dy = 0.0;
    dx = path[1].x - robot_pose_.position.x;
    dy = path[1].y - robot_pose_.position.y;
    next_rrt_node_dist = euclidean_distance(std::make_pair(path[1].x, path[1].y),
                                            std::make_pair(robot_pose_.position.x, robot_pose_.position.y));

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf2::Quaternion(robot_pose_.orientation.x, robot_pose_.orientation.y, robot_pose_.orientation.z,
                                   robot_pose_.orientation.w))
        .getRPY(roll, pitch, yaw);

    double y_vehicle_frame = -sin(yaw) * dx + cos(yaw) * dy;

    double curvature = 2 * y_vehicle_frame / (next_rrt_node_dist * next_rrt_node_dist);
    double steering_angle = curvature * 0.5;

    // postprocess the steering angle
    steering_angle = fmod(steering_angle, M_PI);

    double clip_angle = 90 * M_PI / 180;
    if (steering_angle > clip_angle) {
        steering_angle = clip_angle;

    } else if (steering_angle < -clip_angle) {
        steering_angle = -clip_angle;
    }

    drive_msg.drive.steering_angle = steering_angle;
    drive_msg.drive.speed = get_speed_based_on_angle(steering_angle);

    ackermann_drive_pub->publish(drive_msg);
}

/**
 *  ================================================================================================
 *                                      RRT* methods
 *  ================================================================================================
 */
double RRT::cost(std::vector<RRT_Node>& tree, RRT_Node& node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<RRT_Node>): the current tree
    //    node (RRT_Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

double RRT::line_cost(RRT_Node& n1, RRT_Node& n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (RRT_Node): the RRT_Node at one end of the path
    //    n2 (RRT_Node): the RRT_Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

std::vector<int> RRT::near(std::vector<RRT_Node>& tree, RRT_Node& node) {
    // This method returns the set of Nodes in the neighborhood of a
    // node.
    // Args:
    //   tree (std::vector<RRT_Node>): the current tree
    //   node (RRT_Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method

    return neighborhood;
}
