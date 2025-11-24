#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class CorridorDetector : public rclcpp::Node
{
public:
    CorridorDetector() : Node("corridor_detector")
    {
        
        corridor_start_pub_ = this->create_publisher<std_msgs::msg::Bool>("/corridor_start", 10);

        // Subscriber to the LiDAR scan
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CorridorDetector::scan_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Corridor detector initialized. Seeking a corridor enter. Listening to /scan...");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr corridor_start_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // Parameters for corridor detection (can be set via parameter server)
    const double WALL_DISTANCE_MAX = 2.2; // Max distance for a side wall to be considered (meters)
    const double CORRIDOR_OPENING_MIN = 5.0; // Min distance needed in front to assume open corridor (meters)
    const int SIDE_SCAN_POINTS = 20; // Number of points to check on each side (e.g., 50 degrees/points)

    /**
     * @brief Simple callback to process LaserScan data and detect a corridor.
     * * The logic is: 
     * 1. Check if there's a wall on the LEFT (ranges in the -SIDE_SCAN_POINTS to 0 index range).
     * 2. Check if there's a wall on the RIGHT (ranges in the 0 to +SIDE_SCAN_POINTS index range).
     * 3. Check if the front is relatively clear.
     */
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        bool corridor_enter = false;

        if (msg->ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty LaserScan data.");
            return;
        }

        int total_points = msg->ranges.size();
        
        // Assuming the center point (front) is at index total_points / 2
        int center_index = total_points / 2;

        // Determine the indices for the left and right arcs
        int right_start = center_index - SIDE_SCAN_POINTS;
        int right_end = center_index - 1;

        int left_start = center_index + 1;
        int left_end = center_index + SIDE_SCAN_POINTS;
        
        // Ensure indices are within bounds
        if (right_start < 0 || left_end >= total_points) {
             RCLCPP_WARN_ONCE(this->get_logger(), "Scan point indices out of range. Adjust SIDE_SCAN_POINTS.");
             return;
        }

        // Check the right side (Wall presence required)
        bool wall_on_right = false;
        for (int i = right_start; i <= right_end; ++i) {
            // Check if distance is valid (not infinity, not zero) and within max wall distance
            if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > msg->range_min && msg->ranges[i] < WALL_DISTANCE_MAX) {
                wall_on_right = true;
                break;
            }
        }

        // Check the left side (Wall presence required)
        bool wall_on_left = false;
        for (int i = left_start; i <= left_end; ++i) {
            if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > msg->range_min && msg->ranges[i] < WALL_DISTANCE_MAX) {
                wall_on_left = true;
                break;
            }
        }
        
        // Check the front (min value in a small central arc)
        double front_distance = msg->ranges[center_index];

        // Simple check: Is the front distance large enough?
        bool front_is_open = std::isfinite(front_distance) && front_distance > CORRIDOR_OPENING_MIN;

        // A corridor is detected if there are walls on both sides AND the path ahead is clear.
        if (wall_on_left && wall_on_right && front_is_open) {
            corridor_enter = true;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Corridor DETECTED! Walls R/L: Yes, Front: %.2fm", front_distance);
        } else {
            corridor_enter = false;
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No Corridor. R:%s, L:%s, F:%.2fm (Need >%.2f)", 
                wall_on_right ? "Y" : "N", wall_on_left ? "Y" : "N", front_distance, CORRIDOR_OPENING_MIN);
        }

        // Publish the status
        auto bool_msg = std_msgs::msg::Bool();
        bool_msg.data = corridor_enter;
        corridor_start_pub_->publish(bool_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CorridorDetector>());
    rclcpp::shutdown();
    return 0;
}