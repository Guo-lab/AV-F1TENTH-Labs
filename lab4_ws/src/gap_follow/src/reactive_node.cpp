#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
/// CHECK: include needed ROS msg type headers and libraries

using std::placeholders::_1;

#include <cmath>
#define HALF_PI (M_PI / 2.0)

/** Disparity Extender @ref: PJmin2 */
#define FULL_RANGE 1080
#define RANGE_MIN 300
#define RANGE_MAX 800
#define MAX_DIST 3.5

/**
 * ========================== Helper Function ========================
 * smooth_ranges_with_mean: Smooth the LiDAR ranges using a mean filter
 * @param ranges: vector of LiDAR ranges; which is also the @return
 * @param window_size: size of the window for the mean filter
 */
void smooth_ranges_with_mean(std::vector<float>& ranges, int window_size) {
    std::vector<float> smoothed_ranges(ranges);
    for (int i = 0; i < (int)ranges.size(); i++) {
        float sum = 0.0;
        int count = 0;
        for (int j = -window_size / 2; j <= window_size / 2; j++) {
            if (i + j >= 0 && i + j < (int)ranges.size()) {
                sum += ranges[i + j];
                count++;
            }
        }
        smoothed_ranges[i] = sum / count;
    }
    // std::cout << "Smoothed Ranges: " << smoothed_ranges.size() << std::endl;
    ranges = smoothed_ranges;
}

/** ===================== Corner Case Helper Function ==================
 * @ref: dhanvi97
 * @brief: These two functions are used to help robot pass the corner
 */
int corner_checker(const std::vector<float>& preprocessed_ranges, int left_side_idx, int right_side_idx) {
    for (int i = 0; i < left_side_idx; i++)
        if (preprocessed_ranges.at(i) < 0.1) return -1;
    for (int i = right_side_idx; i < (int)preprocessed_ranges.size(); i++)
        if (preprocessed_ranges.at(i) < 0.1) return 1;
    return 0;
}

bool steering_override_at_corner(int corner_coll, double best_steering_angle) {
    if (corner_coll == 0) {
        return false;
    } else {
        return (corner_coll * best_steering_angle) > 0;
    }
}

class TimeNode : public rclcpp::Node {
   public:
    TimeNode() : Node("time_node") { clock_ = this->get_clock(); }

    auto GetCurrentTime() -> double {
        current_time = clock_->now().seconds();
        prev_time = current_time;
        return current_time;
    }
    auto GetPrevTime() -> double { return prev_time; }
    auto IfLogging() -> bool { return clock_->now().seconds() - prev_time > 0.1; }

   private:
    rclcpp::Clock::SharedPtr clock_;
    double current_time = 0.0;
    double prev_time = 0.0;
};

class ReactiveFollowGap : public rclcpp::Node {
    // Implement Reactive Follow Gap on the car
    // This is just a template, you are free to implement your own node!

   public:
    ReactiveFollowGap() : Node("reactive_node") {
        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, _1));

        ackermann_drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

        timer_node_->GetCurrentTime();
    }

   private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_drive_pub;

    std::vector<float> preprocessed_ranges;
    ackermann_msgs::msg::AckermannDriveStamped drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    std::shared_ptr<TimeNode> timer_node_ = std::make_shared<TimeNode>();

    double fov_start_idx = 0.0;
    double fov_end_idx = 0.0;

    void preprocess_lidar(const std::vector<float>& ranges) {
        // Preprocess the LiDAR scan array. Expert implementation includes:

        // 1.Setting each value to the mean over some window
        const int window_size = 5;

        // 2.Rejecting high values (eg. > 3m)

        preprocessed_ranges.clear();
        for (size_t i = 0; i < ranges.size(); i++) {
            if (i < RANGE_MIN || i > RANGE_MAX) {
                preprocessed_ranges.push_back(0.0);
                continue;
            }
            if (std::isnan(ranges[i]) || !std::isfinite(ranges[i]) || ranges[i] > MAX_DIST) {
                preprocessed_ranges.push_back(MAX_DIST);
                continue;
            }
            preprocessed_ranges.push_back(ranges[i]);
        }
        smooth_ranges_with_mean(preprocessed_ranges, window_size);

        // Disparity extender (might not be that helpful here)
        const float disparity_threshold = 2;
        for (size_t i = 1; i < preprocessed_ranges.size(); i++) {
            double disparity = std::abs(preprocessed_ranges[i] - preprocessed_ranges[i - 1]);
            if (disparity > disparity_threshold) {
                for (size_t j = i; j < preprocessed_ranges.size() && disparity > disparity_threshold; j++) {
                    preprocessed_ranges[j] = preprocessed_ranges[j - 1];
                }
            }
        }
        return;
    }

    auto find_max_gap(const std::vector<float>& ranges) -> std::pair<int, int> {
        // Return the start index & end index of the max gap in free_space_ranges
        int max_start = 0, max_end = 0, current_start = 0;
        bool in_gap = false;

        for (size_t i = RANGE_MIN; i <= RANGE_MAX; i++) {
            if (ranges[i] > 1.5 && !in_gap) {  // 1.5 also an important parameter
                current_start = i;
                in_gap = true;
            } else if (ranges[i] <= 1.5 && in_gap) {
                if ((int)i - current_start > max_end - max_start) {
                    max_start = current_start;
                    max_end = i;
                }
                in_gap = false;
            }
        }
        if (in_gap && RANGE_MAX - current_start > max_end - max_start) {
            max_start = current_start;
            max_end = RANGE_MAX;
        }
        return {max_start, max_end};
    }

    int find_best_point(const std::vector<float>& ranges, int start_idx, int end_idx) {
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
        // Naive: Choose the furthest point within ranges and go there

        int best_point = start_idx;
        for (int i = start_idx; i < end_idx; i++) {
            if (ranges[i] > ranges[best_point]) {
                best_point = i;
            }
        }
        // return best_point;

        return (start_idx + end_idx) / 2;
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

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        preprocess_lidar(scan_msg->ranges);

        fov_start_idx = ((-M_PI / 2) - scan_msg->angle_min) / scan_msg->angle_increment;
        fov_end_idx = ((M_PI / 2) - scan_msg->angle_min) / scan_msg->angle_increment;

        /// TODO:
        // Find closest point to LiDAR
        auto min_it =
            std::min_element(preprocessed_ranges.begin() + RANGE_MIN, preprocessed_ranges.begin() + RANGE_MAX);
        int closest_point_idx = std::distance(preprocessed_ranges.begin(), min_it);

        // Eliminate all points inside 'bubble' (set them to zero)
        if (scan_msg->ranges.at(closest_point_idx) < MAX_DIST) {
            const int bubble_radius = 8;
            int left_idx = std::max(RANGE_MIN, closest_point_idx - bubble_radius);
            int right_idx = std::min(RANGE_MAX, closest_point_idx + bubble_radius);
            for (int i = left_idx; i <= right_idx; i++) {
                preprocessed_ranges[i] = 0.0;
            }
        }

        // Find max length gap
        auto [start_i, end_i] = find_max_gap(preprocessed_ranges);

        // Find the best point in the gap
        int best_point = find_best_point(scan_msg->ranges, start_i, end_i);

        // Logging Data
        if (timer_node_->IfLogging()) {
            timer_node_->GetCurrentTime();
            RCLCPP_INFO(this->get_logger(), "Range Size: %ld", scan_msg->ranges.size());
            RCLCPP_INFO(this->get_logger(), "Gap: [%d, %d]", start_i, end_i);
            RCLCPP_INFO(this->get_logger(), "Best point: %d", best_point);
            RCLCPP_INFO(this->get_logger(), "angle: %f degrees",
                        (scan_msg->angle_min + best_point * scan_msg->angle_increment) * (180.0 / M_PI));
            RCLCPP_INFO(this->get_logger(), "Angle Increment: %f", scan_msg->angle_increment);
        }

        // Publish Drive message
        double angle = scan_msg->angle_min + best_point * scan_msg->angle_increment;
        drive_msg.drive.steering_angle = angle;
        if (scan_msg->ranges.at(best_point) < 1) {
            angle = 0.0;
        }

        drive_msg.drive.speed = GetSpeed(angle);

        int corner_collision = corner_checker(preprocessed_ranges, fov_start_idx, fov_end_idx);
        if (steering_override_at_corner(corner_collision, angle)) {
            drive_msg.drive.steering_angle = 0.0;
            drive_msg.drive.speed = 1.0;
        }

        ackermann_drive_pub->publish(drive_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
