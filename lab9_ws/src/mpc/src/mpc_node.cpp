#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <Eigen/Sparse>
#include <osqp/osqp.h>
#include <OsqpEigen/OsqpEigen.h>

/// CHECK: include needed ROS msg type headers and libraries

/**
 * @ref F1Tenth L12 - Model Predictive Control
 *  Many solvers available: CVXGen, OSQP, QuadProg, Casadi (for non-convex optimization), Multi-Parametric Toolbox
 * (MPT3) ...
 * 
 *  Recommended OSQP: nice EIGEN interface
 * '@ref: https://osqp.org/docs/get_started/sources.html
 *  @ref: https://github.com/robotology/osqp-eigen
 */

using namespace std;
using namespace Eigen;

/**
 * @brief This class is a reader for reading CSV files containing the waypoints
 *  of the path to follow.
 *
 * The waypoints will be extracted to the vector `waypoints`
 */
class WaypointsReader {
   public:
    WaypointsReader(std::vector<std::pair<double, double>>& waypoints) {
        std::string file_path = "./src/mpc/data/waypoints.csv";
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

/**
 * @brief MPC class for implementing the MPC controller
 */
class MPC : public rclcpp::Node {
    // Implement MPC
    // This is just a template, you are free to implement your own node!

   private:
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    std::vector<std::pair<double, double>> waypoints;

   public:
    MPC() : Node("mpc_node") {
        // TODO: create ROS subscribers and publishers
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 1, std::bind(&MPC::pose_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        WaypointsReader waypoints_reader(waypoints);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("MPC"), "Loaded " << waypoints.size() << " waypoints.");
    }

    /**
     * @brief get_speed_based_on_angle:
     *  The speed is based on the steering angle. The larger the steering angle, the slower the speed.
     */
    auto get_speed_based_on_angle(double steering_angle) -> double {
        double abs_angle = std::abs(steering_angle);
        if (abs_angle >= 0.0 * (M_PI / 180.0) && abs_angle <= 10.0 * (M_PI / 180.0)) {
            return 3.0;
        }
        if (abs_angle > 10.0 * (M_PI / 180.0) && abs_angle <= 20.0 * (M_PI / 180.0)) {
            return 2.0;
        }
        return 1.0;
    }

    // /**
    //  * @brief solve_mpc: Solves the MPC optimization problem
    //  *  Define optimization problem here using OsqpEigen or other solvers
    //  */
    // Eigen::VectorXd solve_mpc(const Eigen::VectorXd& state, const Eigen::VectorXd& ref_trajectory) {
    //     int state_dim = 3;    // [x, y, yaw]
    //     int control_dim = 2;  // [speed, steering_angle]
    //     int horizon = 10;     // Prediction horizon

    //     Eigen::MatrixXd A(state_dim, state_dim);    // State transition matrix
    //     Eigen::MatrixXd B(state_dim, control_dim);  // Control matrix
    //     Eigen::VectorXd Q(state_dim);               // State cost matrix
    //     Eigen::VectorXd R(control_dim);             // Control cost matrix

    //     // Fill A, B, Q, R according to your model

    //     OsqpEigen::Solver solver;

    //     // Define your optimization problem
    //     // Set the problem dimensions, matrices and vectors

    //     solver.solve();
    //     Eigen::VectorXd control_input(control_dim);

    //     return control_input;
    // }

    /**
     * @brief pose_callback: Callback function for pose subscriber. This function is called whenever a new pose message
     * is received.
     */
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& pose_msg) {
        auto position = pose_msg->pose.pose.position;
        auto orientation = pose_msg->pose.pose.orientation;

        double roll, pitch, yaw;
        tf2::Matrix3x3(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w))
            .getRPY(roll, pitch, yaw);

        VectorXd state(3);
        state << position.x, position.y, yaw;

        // Define the reference trajectory (can be set dynamically)
        VectorXd ref_trajectory(3);
        ref_trajectory << 5.0, 5.0, 0.0;

        // // Solve the MPC problem to get the optimal control input
        // Eigen::VectorXd control_input = solve_mpc(state, ref_trajectory);

        // Apply control input
        drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

        // drive_msg.drive.speed = control_input[0];
        // drive_msg.drive.steering_angle = control_input[1];

        drive_pub_->publish(drive_msg);
    }

    ~MPC() {}
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPC>());
    rclcpp::shutdown();
    return 0;
}