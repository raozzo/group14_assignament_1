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

// Constants for corridor detection
const double MIN_WALL_CLUSTER_SIZE = 30.0;     // Min points for a cluster to be considered a wall segment
const double MIN_CORRIDOR_WIDTH = 0.25;        // Minimum distance between the two walls 
const double MAX_CORRIDOR_WIDTH = 1.25;       // Max width 
const double Y_SIDE_THRESHOLD = 0.005;          // Minimum absolute Y-distance to be considered a side wall

const double MIN_WALL_SEGMENT_LENGTH = 0.25;   // Minimum length of the cluster segment to be a wall

const double MAX_Y_VARIATION = 0.65;          // Maximum allowed variation in Y coordinates across the cluster (m)

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
     * Checks for minimum length and maximum variation in Y (since the corridor is horizontal).
     * @param cluster The cluster of points.
     * @param cluster_id The index of the cluster being processed.
     * @return true if the cluster is linear and long enough, false otherwise.
     */
    bool validate_wall_segment(const group14_interfaces::msg::RangePointArray &cluster, int cluster_id)
    {
        if (cluster.points.empty()) return false;

        // Get the start (p1) and end (p2) points of the cluster
        const auto& p1 = cluster.points.front().point.point;
        const auto& p2 = cluster.points.back().point.point;

        // Check minimum Length (Wall segment must be long enough)
        double segment_length = std::hypot(p2.x - p1.x, p2.y - p1.y);
        if (segment_length < MIN_WALL_SEGMENT_LENGTH) {
            RCLCPP_DEBUG(this->get_logger(), "Cluster #%d rejected: Too short (Length: %.2fm).", cluster_id, segment_length);
            return false;
        }

        // Check Linearity based on Y-variation (NEW LOGIC)
        // Find the maximum and minimum Y values in the cluster.
        double max_y_cluster = -std::numeric_limits<double>::infinity(); 
        double min_y_cluster = std::numeric_limits<double>::infinity();  

        for (const auto &rp : cluster.points) {
            const double current_y = rp.point.point.y;
            if (current_y > max_y_cluster) {
                max_y_cluster = current_y;
            }
            if (current_y < min_y_cluster) {
                min_y_cluster = current_y;
            }
        }
        
        double y_variation = max_y_cluster - min_y_cluster;
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Cluster #%d Y-Validation: Max Y=%.2fm, Min Y=%.2fm, Variation=%.2fm (Limit: %.2fm)", 
            cluster_id, max_y_cluster, min_y_cluster, y_variation, MAX_Y_VARIATION);


        if (y_variation > MAX_Y_VARIATION) {
            RCLCPP_DEBUG(this->get_logger(), "Cluster #%d rejected: Too vertically curved/wide (Y Variation: %.2fm).", cluster_id, y_variation);
            return false;
        }

        return true;
    }

    /*
     * Simplified logic to determine if two parallel walls (corridor) are present.
     */
    bool check_for_corridor_walls(const group14_interfaces::msg::ClusterArray::SharedPtr &clusters)
    {
        if (clusters->clusters.size() < 2) {
            return false;
        }

        double max_left_y = -std::numeric_limits<double>::infinity(); 
        double min_right_y = std::numeric_limits<double>::infinity();  
        
        bool left_wall_found = false;
        bool right_wall_found = false;

        int cluster_id = 0; // Cluster counter for logging

        for (const auto &cluster : clusters->clusters) {

            if (!cluster.points.empty()) {
                const auto& p_start = cluster.points.front().point.point;
                const auto& p_end = cluster.points.back().point.point;
                RCLCPP_DEBUG(this->get_logger(), 
                    "--- START Processing Cluster #%d --- Size=%zu, Start=(%.2f, %.2f), End=(%.2f, %.2f)",
                    cluster_id, cluster.points.size(), p_start.x, p_start.y, p_end.x, p_end.y);
            } else {
                RCLCPP_DEBUG(this->get_logger(), "--- START Processing Cluster #%d --- Empty.", cluster_id);
            }


            // Basic size check
            if (cluster.points.size() < MIN_WALL_CLUSTER_SIZE) {
                RCLCPP_DEBUG(this->get_logger(), "Cluster #%d rejected: Too small (Size: %zu).", cluster_id, cluster.points.size());
                cluster_id++;
                continue;
            }
            
            // Geometric Validation
            if (!validate_wall_segment(cluster, cluster_id)) {
                cluster_id++;
                continue;
            }

            double sum_y = 0.0;
            for (const auto &rp : cluster.points) {
                sum_y += rp.point.point.y;
            }
            double avg_y = sum_y / cluster.points.size();

            // Side check
            if (avg_y < -Y_SIDE_THRESHOLD) { // Potential LEFT Wall (Negative Y)
                if (left_wall_found){
                    RCLCPP_DEBUG(this->get_logger(), "Cluster #%d: potential left wall unuseful, a left candidate was already accepted (Avg Y: %.2f).", cluster_id, avg_y);
                    cluster_id++;
                    continue;
                }
                else {   
                    left_wall_found = true;
                    RCLCPP_DEBUG(this->get_logger(), "Cluster #%d accepted as LEFT wall candidate (Avg Y: %.2f).", cluster_id, avg_y);
                    // Find the point closest to the center line (max Y value)
                    for (const auto &rp : cluster.points) {
                        if (rp.point.point.y > max_left_y) {
                            max_left_y = rp.point.point.y;
                        }
                    }
                }
            } else if (avg_y > Y_SIDE_THRESHOLD) { // Potential RIGHT Wall (Positive Y)
                if (right_wall_found){
                    RCLCPP_DEBUG(this->get_logger(), "Cluster #%d: potential right wall unuseful, a right candidate was already accepted (Avg Y: %.2f).", cluster_id, avg_y);
                    cluster_id++;
                    continue;
                }
                else {
                    right_wall_found = true;
                    RCLCPP_DEBUG(this->get_logger(), "Cluster #%d accepted as RIGHT wall candidate (Avg Y: %.2f).", cluster_id, avg_y);
                    // Find the point closest to the center line (min Y value)
                    for (const auto &rp : cluster.points) {
                        if (rp.point.point.y < min_right_y) {
                            min_right_y = rp.point.point.y;
                        }
                    }
                }
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Cluster #%d rejected: Too central (Avg Y: %.2f).", cluster_id, avg_y);
            }
            
            cluster_id++;
        }
        
        // Final Condition Check (Requires both walls to be found)
        if (!left_wall_found || !right_wall_found) {
            RCLCPP_DEBUG(this->get_logger(), "Corridor failed: Left (%s) or Right (%s) wall not found.", 
                left_wall_found ? "found" : "missing", right_wall_found ? "found" : "missing");
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