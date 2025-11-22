#ifndef GROUP14_CYLINDERS_FINDER_HPP
#define GROUP14_CYLINDERS_FINDER_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "group14_assignment_1/utils.hpp"
#include "group14_interfaces/msg/tables_array.hpp"

class CylindersFinder : public rclcpp::Node
{
public:
    explicit CylindersFinder(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    /**
     * @brief Main callback for Lidar data processing and table tracking update.
     * Performs segmentation of laser points into clusters, identifies potential circular
     * shapes, and updates the state of detected tables.
     * @param scan Input LaserScan message.
     */
    void process_scan_(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    /**
     * @brief Segments the raw LaserScan data into distinct clusters of points.
     * Iterates through the laser ranges and groups consecutive points into clusters
     * based on the Adaptive Breakpoint Detector algorithm (G. A. Borges and M. J. Aldon,
     * "Line Extraction in 2D Range Images for Mobile Robotics", 2004.)
     * @param scan Reference to the input LaserScan message.
     * @param clusters Reference to the output vector of clusters (groups of RangePoints).
     * The vector is cleared at the beginning of the function.
     */
    void cluster_ranges_(
        const sensor_msgs::msg::LaserScan::SharedPtr &scan,
        std::vector<std::vector<group14::RangePoint>> &clusters);

    /**
     * @brief Merges the last cluster into the first to handle LIDAR wrap-around.
     * It takes the points from the last cluster and inserts them at the beginning of the first
     * cluster to preserve angular continuity. Then it removes the last cluster from the vector.
     *
     * @param[in,out] clusters Reference to the vector of clusters.
     */
    static void merge_cyclic_clusters_(std::vector<std::vector<group14::RangePoint>> &clusters);

    /**
     * @brief Converts a single polar measurement (range, angle) into a Cartesian PointStamped.
     * @param range The measured radial distance in meters.
     * @param angle The angular position in radians.
     * @param header ROS header (timestamp and frame_id) to attach to the point.
     * @return A PointStamped message containing the computed (x, y, 0) coordinates.
     */
    static geometry_msgs::msg::PointStamped rangeToPointStamped_(
        float range, float angle, std_msgs::msg::Header header);

    /**
     * @brief Computes the adaptive distance threshold as defined in the Adaptive Breakpoint Detector algorithm
     * (G. A. Borges and M. J. Aldon, "Line Extraction in 2D Range Images for Mobile Robotics", 2004).
     * Calculates the maximum expected distance between consecutive points on a continuous surface,
     * taking into account beam divergence (i.e. the max distance increases with the range value).
     *
     * @param prev_valid_range Range of the previous valid point (r).
     * @param angle_increment Angular distance (rad) between current and previous point (delta phi).
     * @param INCIDENCE_ANGLE_THRESHOLD Minimum incidence angle to assume continuity (lambda).
     * @param NOISE_FLOOR Sensor noise margin (sigma).
     * @return The dynamic threshold D_max.
     */
    static double compute_dynamic_D_max_(float prev_valid_range, float angle_increment,
                                         float INCIDENCE_ANGLE_THRESHOLD, float NOISE_FLOOR);

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    std::shared_ptr<rclcpp::Publisher<group14_interfaces::msg::TablesArray, std::allocator<void>>> tables_publisher_;

    const float INCIDENCE_ANGLE_THRESHOLD = 0.1745; // rad, i.e. 10deg
    const float SCAN_NOISE_FLOOR = 0.02;            // i.e. 2cm
};

#endif