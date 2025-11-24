#ifndef GROUP14_LASER_SCAN_CLUSTERING_HPP
#define GROUP14_LASER_SCAN_CLUSTERING_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "group14_interfaces/msg/cluster_array.hpp"

class LaserScanClustering : public rclcpp::Node
{
public:
    explicit LaserScanClustering(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    /**
     * @brief Activates the node's main logic upon receiving an initial pose.
     * Initializes the LIDAR subscription if it is not already active.
     * @param msg The initial pose message (unused).
     */
    void initial_pose_callback_(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    /**
     * @brief Main callback for processing incoming Lidar data.
     * Manages the segmentation of the scan into clusters and publishes the result
     * to the `/laser_scan_clustering` topic.
     * @param scan Input LaserScan message.
     */
    void process_scan_(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    /**
     * @brief Segments the raw LaserScan data into distinct clusters of points.
     * Iterates through the laser ranges and groups consecutive points into clusters
     * based on the Adaptive Breakpoint Detector algorithm (G. A. Borges and M. J. Aldon,
     * "Line Extraction in 2D Range Images for Mobile Robotics", 2004).
     * @param scan Reference to the input LaserScan message.
     * @param clusters Reference to the output ClusterArray message. The vector of clusters
     * is cleared at the beginning of the function.
     */
    void cluster_ranges_(
        const sensor_msgs::msg::LaserScan::SharedPtr &scan,
        group14_interfaces::msg::ClusterArray &clusters);

    /**
     * @brief Merges the last cluster into the first to handle LIDAR wrap-around.
     * Since the laser scans 360 degrees, an object might be split between the end
     * and the beginning of the scan array. This function checks continuity and merges them.
     *
     * @param clusters Reference to the ClusterArray message containing the detected clusters.
     */
    void merge_cyclic_clusters_(
        group14_interfaces::msg::ClusterArray &clusters);

    /**
     * @brief Computes the adaptive distance threshold as defined in the Adaptive Breakpoint Detector algorithm
     * (G. A. Borges and M. J. Aldon, "Line Extraction in 2D Range Images for Mobile Robotics", 2004).
     * Calculates the maximum expected distance between consecutive points on a continuous surface,
     * taking into account beam divergence (distance increases as range increases).
     *
     * @param prev_valid_range Range of the previous valid point (r).
     * @param angle_increment Angular distance (rad) between current and previous point (delta phi).
     * @param INCIDENCE_ANGLE_THRESHOLD Minimum incidence angle to assume continuity (lambda).
     * @param NOISE_FLOOR Sensor noise margin (sigma).
     * @return The dynamic threshold D_max.
     */
    double compute_dynamic_D_max_(
        float prev_valid_range, float angle_increment,
        float INCIDENCE_ANGLE_THRESHOLD, float NOISE_FLOOR);

    /**
     * @brief Converts a single polar measurement (range, angle) into a Cartesian PointStamped.
     * @param range The measured radial distance in meters.
     * @param angle The angular position in radians.
     * @param header ROS header (timestamp and frame_id) to attach to the point.
     * @return A PointStamped message containing the computed (x, y, 0) coordinates.
     */
    geometry_msgs::msg::PointStamped rangeToPointStamped_(
        float range, float angle, std_msgs::msg::Header header);

    /**
     * @brief Publishes visualization markers to RVIZ for debugging purposes.
     * Displays each cluster with a different color.
     * @param clusters The array of clusters to visualize.
     * @param header The header to attach to the marker message (for frame and timestamp).
     */
    void DEBUG_publish_clusters_(
        const group14_interfaces::msg::ClusterArray &clusters,
        const std_msgs::msg::Header &header);

    /**
     * @brief Helper function to cycle through colors for visualization.
     * @param index The index of the cluster.
     * @return A ColorRGBA message corresponding to the index.
     */
    std_msgs::msg::ColorRGBA DEBUG_get_color_(int index);

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;

    rclcpp::Publisher<group14_interfaces::msg::ClusterArray>::SharedPtr clusters_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clusters_marker_publisher_;

    const int MIN_CLUSTER_POINTS = 3;
    const float INCIDENCE_ANGLE_THRESHOLD = 0.1745; // rad, i.e. 10deg
    const float SCAN_NOISE_FLOOR = 0.01;            // i.e. 1cm
    const double PI = 3.14159265358979323846;
};

#endif