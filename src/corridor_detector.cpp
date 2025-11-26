#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "group14_interfaces/msg/cluster_array.hpp" 
#include <chrono>
#include <memory>
#include <cmath>
#include <functional>
#include <algorithm>
#include <numeric>
#include <limits> // For infinity

using namespace std::chrono_literals;

// Constants for Corridor Detection
const double MIN_WALL_CLUSTER_SIZE = 45.0;     // Min points for a cluster to be considered a wall segment
const double MIN_CORRIDOR_WIDTH = 1.0;        // Minimum distance between the two walls 
const double MAX_CORRIDOR_WIDTH = 3.30;       // Max width 
const double Y_SIDE_THRESHOLD = 0.5;          // Minimum absolute Y-distance to be considered a side wall

const double MIN_WALL_SEGMENT_LENGTH = 2.15;   // Minimum length of the cluster segment to be a wall
const double MAX_NON_LINEARITY_ERROR = 0.17;   // Maximum allowed error from a straight line (Tolerance for non-straight walls)

class CorridorDetector : public rclcpp::Node
{
public:
    CorridorDetector() : Node("corridor_detector")
    {
        RCLCPP_INFO(this->get_logger(), "Corridor Detector node started.");

        // Publisher for the corridor trigger
        corridor_trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>("/corridor_trigger", 10);
        
        // Subscription to the clustered scan data
        RCLCPP_INFO(this->get_logger(), "Subscribing to /laser_scan_clustering for wall detection.");
        clustered_scan_subscription_ =
            this->create_subscription<group14_interfaces::msg::ClusterArray>(
                "/laser_scan_clustering",
                rclcpp::QoS(10),
                std::bind(&CorridorDetector::process_clustered_scan_, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Corridor Detector initialized (Cluster-Based Logic).");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr corridor_trigger_pub_;
    rclcpp::Subscription<group14_interfaces::msg::ClusterArray>::SharedPtr clustered_scan_subscription_;
    
    // State variable
    bool in_corridor_ = false;

  
    // Sends the specified state (true/false) to the /corridor_trigger topic.
    void publish_trigger(bool state)
    {
        // Avoid sending the same state repeatedly
        if (state == in_corridor_) return;
        
        auto bool_msg = std_msgs::msg::Bool();
        bool_msg.data = state;
        corridor_trigger_pub_->publish(bool_msg);

        in_corridor_ = state; // Update internal state

        if (state) {
             RCLCPP_INFO(this->get_logger(), "Corridor trigger on");
        } else {
             RCLCPP_INFO(this->get_logger(), "Corridor trigger off, corridor ended");
        }
    }

    // Main logic to process clustered laser scan data and detect corridor walls.
    void process_clustered_scan_(const group14_interfaces::msg::ClusterArray::SharedPtr clusters)
    {
        // Check if the conditions for a corridor are met
        bool corridor_detected = check_for_corridor_walls(clusters);

        if (corridor_detected && !in_corridor_) {
            // Corridor detected and we were previously outside
            publish_trigger(true);
        } else if (!corridor_detected && in_corridor_) {
            // Corridor ended and we were previously inside
            publish_trigger(false);
        }
    }
    
    /**
     * @brief Performs simple geometric validation on a cluster to check if it represents a straight wall segment.
     * Checks for minimum length and maximum non-linearity error.
     * @param cluster The cluster of points.
     * @return true if the cluster is linear and long enough, false otherwise.
     */
    bool validate_wall_segment(const group14_interfaces::msg::RangePointArray &cluster)
    {
        if (cluster.points.empty()) return false;

        // Get the start (p1) and end (p2) points of the cluster
        const auto& p1 = cluster.points.front().point.point;
        const auto& p2 = cluster.points.back().point.point;

        // Check minimum Length (Wall segment must be long enough)
        double segment_length = std::hypot(p2.x - p1.x, p2.y - p1.y);
        if (segment_length < MIN_WALL_SEGMENT_LENGTH) {
            RCLCPP_DEBUG(this->get_logger(), "Cluster rejected: Too short (Length: %.2fm).", segment_length);
            return false;
        }

        // Check Linearity (Tolerance for non-straight points)
        // Find the maximum perpendicular distance of any point to the line defined by p1 and p2.
        
        double A = p2.y - p1.y;
        double B = p1.x - p2.x;
        double C = -A * p1.x - B * p1.y;
        double sqrt_AB = std::hypot(A, B); // Square root of A^2 + B^2

        if (sqrt_AB < 1e-6) { // Points are too close to define a line
            return true; 
        }

        double max_perp_dist = 0.0;
        for (const auto &rp : cluster.points) {
            const auto& p = rp.point.point;
            // Perpendicular distance formula: |Ax + By + C| / sqrt(A^2 + B^2)
            double dist = std::abs(A * p.x + B * p.y + C) / sqrt_AB;
            if (dist > max_perp_dist) {
                max_perp_dist = dist;
            }
        }
        
        if (max_perp_dist > MAX_NON_LINEARITY_ERROR) {
            RCLCPP_DEBUG(this->get_logger(), "Cluster rejected: Too curved (Max error: %.2fm).", max_perp_dist);
            return false;
        }

        return true;
    }

    /*
     * Simplified logic to determine if two parallel walls (corridor) are present.
     * It requires two large, linear clusters, one on the left and one on the right,
     * and checks if the distance between them is within the defined corridor width limits.
     */
    bool check_for_corridor_walls(const group14_interfaces::msg::ClusterArray::SharedPtr &clusters)
    {
        if (clusters->clusters.size() < 2) {
            return false;
        }

        // Initialize with extreme values to find the closest points to the robot's center line (X axis)
        double max_left_y = -std::numeric_limits<double>::infinity(); 
        double min_right_y = std::numeric_limits<double>::infinity();  
        
        bool left_wall_found = false;
        bool right_wall_found = false;

        for (const auto &cluster : clusters->clusters) {
            
            // Basic size check
            if (cluster.points.size() < MIN_WALL_CLUSTER_SIZE) continue;
            
            // Geometric Validation
            if (!validate_wall_segment(cluster)) continue;

            double sum_y = 0.0;
            for (const auto &rp : cluster.points) {
                sum_y += rp.point.point.y;
            }
            double avg_y = sum_y / cluster.points.size();

            // 3. Side check
            if (avg_y < -Y_SIDE_THRESHOLD) { // Potential LEFT Wall (Negative Y)
                left_wall_found = true;
                // Find the point closest to the center line (max Y value)
                for (const auto &rp : cluster.points) {
                    if (rp.point.point.y > max_left_y) {
                        max_left_y = rp.point.point.y;
                    }
                }
            } else if (avg_y > Y_SIDE_THRESHOLD) { // Potential RIGHT Wall (Positive Y)
                right_wall_found = true;
                // Find the point closest to the center line (min Y value)
                for (const auto &rp : cluster.points) {
                    if (rp.point.point.y < min_right_y) {
                        min_right_y = rp.point.point.y;
                    }
                }
            }
        }
        
        // Final Condition Check (Requires both walls to be found)
        if (!left_wall_found || !right_wall_found) {
            return false;
        }

        // Check the distance (width) between the closest points of the two walls
        double measured_width = min_right_y - max_left_y;
        
        RCLCPP_DEBUG(this->get_logger(), "Corridor check: Measured Width: %.2fm (Limits: %.1f-%.1f)", 
            measured_width, MIN_CORRIDOR_WIDTH, MAX_CORRIDOR_WIDTH);

        if (measured_width >= MIN_CORRIDOR_WIDTH && measured_width <= MAX_CORRIDOR_WIDTH) {
            return true;
        }

        return false;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CorridorDetector>());
    rclcpp::shutdown();
    return 0;
}