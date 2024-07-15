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
#define FULL_RANGE 1080
#define RANGE_MIN 200
#define RANGE_MAX 900

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

    void preprocess_lidar(const std::vector<float>& ranges) {
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        const int window_size = 5;
        // 2.Rejecting high values (eg. > 3m)

        preprocessed_ranges.clear();
        for (size_t i = 0; i < ranges.size(); i++) {
            // if (i < RANGE_MIN || i > RANGE_MAX) {
            //     preprocessed_ranges.push_back(0.0);
            //     continue;
            // }
            if (std::isnan(ranges[i]) || !std::isfinite(ranges[i]) || ranges[i] > 3.0) {
                preprocessed_ranges.push_back(3.0);
                continue;
            }
            preprocessed_ranges.push_back(ranges[i]);
        }
        // std::cout << "Original Ranges: " << preprocessed_ranges.size() << std::endl;
        smooth_ranges_with_mean(preprocessed_ranges, window_size);
        // std::cout << "Preprocessed Ranges: " << preprocessed_ranges.size() << std::endl;
        // for (auto each_range: preprocessed_ranges) {
        //     std::cout << each_range << " ";
        // }
        // std::cout << std::endl;
        return;
    }

    auto find_max_gap(const std::vector<float>& ranges) -> std::pair<int, int> {
        // Return the start index & end index of the max gap in free_space_ranges
        int max_start = 0, max_end = 0, current_start = 0;
        bool in_gap = false;
        for (size_t i = 0; i < ranges.size(); i++) {
            if (ranges[i] > 0.3 && !in_gap) { // not 0.0, for moving forward
                current_start = i;
                in_gap = true;
            } else if (ranges[i] <= 0.3 && in_gap) {
                if ((int)i - current_start > max_end - max_start) {
                    max_start = current_start;
                    max_end = i;
                }
                in_gap = false;
            }
        }
        if (in_gap && (int)ranges.size() - current_start > max_end - max_start) {
            max_start = current_start;
            max_end = ranges.size();
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
        return best_point;
    }

    auto GetSpeed(double steering_angle) -> double {
        double abs_angle = std::abs(steering_angle);
        if (abs_angle >= 0.0 * (M_PI / 180.0) && abs_angle <= 10.0 * (M_PI / 180.0)) {
            return 1.5;
        }
        if (abs_angle > 10.0 * (M_PI / 180.0) && abs_angle <= 20.0 * (M_PI / 180.0)) {
            return 1.0;
        }
        return 0.5;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        preprocess_lidar(scan_msg->ranges);

        /// TODO:
        // Find closest point to LiDAR
        auto min_it = std::min_element(preprocessed_ranges.begin(), preprocessed_ranges.end());
        int closest_point_idx = std::distance(preprocessed_ranges.begin(), min_it);
        
        // Eliminate all points inside 'bubble' (set them to zero)
        const int bubble_radius = 5;
        int left_idx = std::max(0, closest_point_idx - bubble_radius);
        int right_idx = std::min((int)preprocessed_ranges.size(), closest_point_idx + bubble_radius);
        for (int i = left_idx; i < right_idx; i++) {
            preprocessed_ranges[i] = 0.0;
        }

        // Find max length gap
        auto [start_i, end_i] = find_max_gap(preprocessed_ranges);

        // Find the best point in the gap
        int best_point = find_best_point(scan_msg->ranges, start_i, end_i);
        if (timer_node_->IfLogging()) {
            timer_node_->GetCurrentTime();
            RCLCPP_INFO(this->get_logger(), "Range Size: %ld", scan_msg->ranges.size());
            RCLCPP_INFO(this->get_logger(), "Gap: [%d, %d]", start_i, end_i);
            RCLCPP_INFO(this->get_logger(), "Best point: %d", best_point);
            RCLCPP_INFO(this->get_logger(), "angle: %f", scan_msg->angle_min + best_point * scan_msg->angle_increment);
        }
        
        // Publish Drive message
        double angle = scan_msg->angle_min + best_point * scan_msg->angle_increment;
        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.speed = GetSpeed(angle);
        ackermann_drive_pub->publish(drive_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}