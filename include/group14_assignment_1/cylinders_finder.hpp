#ifndef GROUP14_CYLINDERS_FINDER_HPP
#define GROUP14_CYLINDERS_FINDER_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "group14_assignment_1/utils.hpp"
#include "group14_interfaces/msg/tables_array.hpp"
#include "group14_interfaces/srv/look_for_tables.hpp"

class Table
{
public:
    Table(group14::Circle circle_, double initial_weight = 0.0, int num_detections_ = 1);

    /**
     * @brief Updates the table estimation using a recursive weighted average.
     * @param new_circle The new circle measurement to merge into the current estimate.
     * @param distance The distance between the robot and the detected circle center.
     * @param cluster_size The number of LIDAR points used to fit the new circle.
     */
    void update(const group14::Circle &new_circle, double distance, int cluster_size);

    /**
     * @brief Checks if a candidate circle belongs to an already tracked table.
     * @param t The existing tracked table.
     * @param c The new candidate circle.
     * @return true if the candidate matches the tracked table.
     * @return false otherwise.
     */
    static bool is_same_table(const Table &t, const group14::Circle &c);

    group14::Circle circle;
    double cumulative_weight;
    int num_detections;

    static constexpr float DISTANCE_THRESHOLD = 0.25; // Max distance between tables' centers to be considered as the same table
};

class CylindersFinder : public rclcpp::Node
{
public:
    explicit CylindersFinder(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief Activates the node's main logic upon receiving an initial pose. Initializes the
     * LIDAR subscription and the service server if they are not already active.
     * @param msg The initial pose message (unused).
     */
    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    /**
     * @brief Main callback for Lidar data processing and table tracking update.
     * Performs segmentation of laser points into clusters, identifies potential circular
     * shapes, and updates the state of detected tables.
     * @param scan Input LaserScan message.
     */
    void process_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    /**
     * @brief Service callback to retrieve valid detected tables.
     * Filters stored table candidates based on the number of detections threshold
     * @param request  The service request (unused).
     * @param response The service response to be populated with the list of found tables.
     */
    void look_for_tables_callback(const std::shared_ptr<group14_interfaces::srv::LookForTables::Request> request,
                                  std::shared_ptr<group14_interfaces::srv::LookForTables::Response> response);

private:
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
     * @brief Processes a cluster to detect, validate, and track cylindrical tables.
     * Executes the detection pipeline: fitting (Kåsa), validation (radius/MSE),
     * transformation to map frame (using the provided TF), and tables tracking.
     * @param cluster The vector of points representing the segmented scan data.
     * @param tf The pre-calculated transform from the laser frame to the map frame.
     */
    void look_for_tables_(const std::vector<group14::RangePoint> &cluster,
                          geometry_msgs::msg::TransformStamped &tf);

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

    /**
     * @brief Estimates circle parameters (center and radius) by fitting a cluster of points
     * using Kasa's algebraic method (least squares). See I. Kåsa, "A circle fitting procedure and its
     * error analysis," in IEEE Transactions on Instrumentation and Measurement, vol. IM-25, no. 1, pp. 8-14,
     * March 1976
     * @param cluster Reference to a vector of group14::RangePoint representing the points of the
     * cluster to fit.
     * @return a `group14::Circle` object containing the center coordinates (x, y) and the radius
     * if the calculation is successful, `std::nullopt` otherwise.
     */
    static std::optional<group14::Circle> fit_circle_Kasa_(const std::vector<group14::RangePoint> &cluster);

    /**
     * @brief Validates a candidate circle against the point cluster using the estimated radius
     * (rejects noise (too small radius) or walls (too large radius)) and MSE
     * @param cluster The cluster of points to validate against
     * @param circle The estimated circle parameters (center and radius)
     * @return true if the circle meets both radius and MSE criteria.
     * @return false Otherwise.
     */
    bool validate_circle_fit_(const std::vector<group14::RangePoint> &cluster,
                              const group14::Circle &circle);

    /**
     * @brief Queries the TF2 buffer to find the transform from the source frame in `header`
     * to the `dst_frame_id` at the timestamp `header.stamp`.
     * @param header The header containing the source frame ID and the timestamp.
     * @param dst_frame_id The target frame ID (e.g., "map").
     * @return std::optional containing the TransformStamped if successful, or std::nullopt if the lookup fails.
     */
    std::optional<geometry_msgs::msg::TransformStamped> transform_(const std_msgs::msg::Header header,
                                                                   const std::string dst_frame_id);

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscription_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    std::shared_ptr<rclcpp::Publisher<group14_interfaces::msg::TablesArray, std::allocator<void>>> tables_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clusters_publisher_;

    std::vector<Table> tables_; // Detected tables
    std::shared_ptr<rclcpp::Service<group14_interfaces::srv::LookForTables>> service_;

    const int MIN_CLUSTER_POINTS = 3;
    const float INCIDENCE_ANGLE_THRESHOLD = 0.1745; // rad, i.e. 10deg
    const float SCAN_NOISE_FLOOR = 0.01;            // i.e. 1cm
    const float MIN_RADIUS = 0.02;
    const float MAX_RADIUS = 0.50;
    const double MSE_THRESHOLD = 0.005;
    const int NUM_DETECTIONS_THRESHOLD = 30;
};

#endif