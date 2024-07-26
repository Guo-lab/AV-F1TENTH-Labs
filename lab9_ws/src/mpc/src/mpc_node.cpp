#include <OsqpEigen/OsqpEigen.h>
#include <osqp/osqp.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
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

// to be moved to hpp file

/**
 * @ref F1Tenth L12 - Model Predictive Control
 *  Many solvers available: CVXGen, OSQP, QuadProg, Casadi (for non-convex optimization), Multi-Parametric Toolbox
 * (MPT3) ...
 *
 *  Recommended OSQP: nice EIGEN interface
 * '@ref: https://osqp.org/docs/get_started/sources.html
 *  @ref: https://github.com/robotology/osqp-eigen
 *
 * @ref:
 *  https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html
 *  https://math.stackexchange.com/questions/3177528/how-to-linearize-a-kinematic-bicycle-model
 *
 * @ref: https://github.com/JunshengFu/Model-Predictive-Control
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

struct BicycleModelState {
    double position_x;
    double position_y;
    double psi;
    double velocity;
    double cross_track_error;
    double heading_error;
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
    double steering_angle_ = 0.0;
    double throttle_ = 0.0;

    double latency_ = 0.1;  // 100 ms
    double Lf_ = 0.33 / 2;  // the distance between the front axle and its center of mass is half of the wheelbase

   public:
    MPC() : Node("mpc_node") {
        // TODO: create ROS subscribers and publishers
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 1, std::bind(&MPC::pose_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        WaypointsReader waypoints_reader(waypoints);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("MPC"), "Loaded " << waypoints.size() << " waypoints.");

        steering_angle_ = 0.0;
        throttle_ = 0.0;
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

    /**
     * @brief solve_mpc: Solves the MPC optimization problem
     *  Define optimization problem here using OsqpEigen or other solvers
     */
    VectorXd solve_mpc(const VectorXd& state, const VectorXd& ref_trajectory) {
        // assert(state.size() == ref_trajectory.size());
        int state_dim = state.size();  // [x, y, theta]
        int control_dim = 2;           // [v, delta], [speed, steering_angle]
        int horizon = 10;              // Prediction horizon

        Eigen::MatrixXd A(state_dim, state_dim);    // State transition matrix
        Eigen::MatrixXd B(state_dim, control_dim);  // Control matrix

        // Define cost matrices
        MatrixXd Q = MatrixXd::Identity(state_dim, state_dim);      // State cost matrix
        MatrixXd R = MatrixXd::Identity(control_dim, control_dim);  // Control cost matrix

        // Define constraints
        VectorXd lower_bound(state_dim * horizon + control_dim * (horizon - 1));
        VectorXd upper_bound(state_dim * horizon + control_dim * (horizon - 1));

        // Initialize OSQP solver
        OsqpEigen::Solver solver;
        solver.settings()->setWarmStart(true);
        solver.settings()->setVerbosity(false);

        // Define the optimization problem
        SparseMatrix<double> H;           // Hessian matrix
        VectorXd g;                       // Gradient vector
        SparseMatrix<double> constraint;  // Constraint matrix
        VectorXd b;                       // Constraint vector

        // Fill the matrices and vectors with problem-specific data
        // This will involve setting up the kinematic bicycle model dynamics
        // and constraints based on the ref_trajectory_points

        solver.data()->setNumberOfVariables(state_dim * horizon + control_dim * (horizon - 1));
        solver.data()->setNumberOfConstraints(state_dim * horizon + control_dim * (horizon - 1));

        if (!solver.data()->setHessianMatrix(H)) return VectorXd::Zero(control_dim);
        if (!solver.data()->setGradient(g)) return VectorXd::Zero(control_dim);
        // if (!solver.data()->setLinearConstraintsMatrix(A)) return VectorXd::Zero(control_dim);
        if (!solver.data()->setLowerBound(lower_bound)) return VectorXd::Zero(control_dim);
        if (!solver.data()->setUpperBound(upper_bound)) return VectorXd::Zero(control_dim);

        // Solve the problem
        // Solve the problem
        if (!solver.initSolver()) return VectorXd::Zero(control_dim);
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return VectorXd::Zero(control_dim);

        // Extract the control input
        VectorXd control_input = solver.getSolution().head(control_dim);

        return control_input;
    }

    /**
     * @brief convert_map_coordinates_to_car_coordinates: Converts the map coordinates to car coordinates
     */
    void convert_map_coordinates_to_car_coordinates(double px, double py, double psi, VectorXd& ptsx, VectorXd& ptsy) {
        for (size_t i = 0; i < waypoints.size(); i++) {
            double shift_x = waypoints[i].first - px;
            double shift_y = waypoints[i].second - py;

            ptsx[i] = shift_x * std::cos(-psi) - shift_y * std::sin(-psi);
            ptsy[i] = shift_x * std::sin(-psi) + shift_y * std::cos(-psi);
        }
    }

    VectorXd polyfit(VectorXd xvals, VectorXd yvals, int order) {
        assert(xvals.size() == yvals.size());
        assert(order >= 1 && order <= xvals.size() - 1);

        MatrixXd A(xvals.size(), order + 1);
        for (int i = 0; i < xvals.size(); i++) {
            A(i, 0) = 1.0;
        }

        for (int i = 0; i < xvals.size(); i++) {
            for (int j = 0; j < order; j++) {
                A(i, j + 1) = A(i, j) * xvals(i);
            }
        }

        auto Q = A.householderQr();
        auto result = Q.solve(yvals);

        return result;
    }

    /**
     * @brief pose_callback: Callback function for pose subscriber. This function is called whenever a new pose message
     * is received.
     */
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& pose_msg) {
        // waypoints: vector of pairs of x, y coordinates: ptsx, ptsy
        //
        auto position = pose_msg->pose.pose.position;
        // position: x, y
        auto orientation = pose_msg->pose.pose.orientation;

        double roll, pitch, yaw;
        tf2::Matrix3x3(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w))
            .getRPY(roll, pitch, yaw);

        // Extract linear velocity components
        double linear_x = pose_msg->twist.twist.linear.x;
        double linear_y = pose_msg->twist.twist.linear.y;
        double linear_z = pose_msg->twist.twist.linear.z;

        // Calculate speed
        double speed = std::sqrt(linear_x * linear_x + linear_y * linear_y + linear_z * linear_z);

        VectorXd ptsx_car(waypoints.size());
        VectorXd ptsy_car(waypoints.size());
        convert_map_coordinates_to_car_coordinates(position.x, position.y, yaw, ptsx_car, ptsy_car);

        // Fit a 3-order polynomial for the waypoints
        VectorXd coeffs = polyfit(ptsx_car, ptsy_car, 3);

        VectorXd state(6);  // [x, y, psi, v, cte, epsi]
        state << position.x, position.y, yaw, speed;

        // Define the reference trajectory (can be set dynamically)
        VectorXd ref_trajectory(3 * waypoints.size());
        for (size_t i = 0; i < waypoints.size(); ++i) {
            ref_trajectory(3 * i) = waypoints[i].first;
            ref_trajectory(3 * i + 1) = waypoints[i].second;
            ref_trajectory(3 * i + 2) = 0.0;
        }

        // // Solve the MPC problem to get the optimal control input
        VectorXd control_input = solve_mpc(state, ref_trajectory);

        // Apply control input
        drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

        drive_msg.drive.speed = control_input[0];
        drive_msg.drive.steering_angle = control_input[1];

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