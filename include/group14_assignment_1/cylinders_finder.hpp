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
#include "group14_interfaces/msg/range_point.hpp"
#include "group14_interfaces/msg/range_point_array.hpp"
#include "group14_interfaces/msg/cluster_array.hpp"
#include "group14_interfaces/srv/look_for_tables.hpp"

struct Circle
{
    geometry_msgs::msg::PointStamped center;
    float r;

    Circle(float x_center, float y_center, float r_, std_msgs::msg::Header header)
        : r(r_)
    {
        center.header = header;
        center.point.x = x_center;
        center.point.y = y_center;
        center.point.z = 0;
    }

    Circle(geometry_msgs::msg::PointStamped center_, float r_)
        : center(center_), r(r_) {}
};

class Table
{
public:
    Table(Circle circle_, double initial_weight = 0.0, int num_detections_ = 1);

    /**
     * @brief Updates the table estimation using a recursive weighted average.
     * @param new_circle The new circle measurement to merge into the current estimate.
     * @param distance The distance between the robot and the detected circle center.
     * @param cluster_size The number of LIDAR points used to fit the new circle.
     */
    void update(const Circle &new_circle, double distance, int cluster_size);

    /**
     * @brief Checks if a candidate circle belongs to an already tracked table.
     * @param t The existing tracked table.
     * @param c The new candidate circle.
     * @return true if the candidate matches the tracked table.
     * @return false otherwise.
     */
    static bool is_same_table(const Table &t, const Circle &c);

    Circle circle;
    double cumulative_weight;
    int num_detections;

    static constexpr float DISTANCE_THRESHOLD = 0.25; // Max distance between tables' centers to be considered as the same table
};

class CylindersFinder : public rclcpp::Node
{
public:
    explicit CylindersFinder(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief Callback for processing pre-clustered laser scan data.
     * Receives the cluster array, transforms the data to the map frame,
     * and iterates through each cluster to attempt circle detection.
     * @param scan The message containing the array of clusters (ClusterArray).
     */
    void process_clustered_scan_(const group14_interfaces::msg::ClusterArray &scan);

    /**
     * @brief Service callback to retrieve valid detected tables.
     * Filters stored table candidates based on the number of detections threshold.
     * @param request  The service request (unused).
     * @param response The service response to be populated with the list of found tables.
     */
    void look_for_tables_callback(const std::shared_ptr<group14_interfaces::srv::LookForTables::Request> request,
                                  std::shared_ptr<group14_interfaces::srv::LookForTables::Response> response);

private:
    /**
     * @brief Processes a single cluster to detect, validate, and track cylindrical tables.
     * Executes the detection pipeline: fitting (Kasa), validation (radius/MSE),
     * transformation to map frame (using the provided TF), and tables tracking.
     * @param cluster The RangePointArray message representing one cluster.
     * @param tf The pre-calculated transform from the laser frame to the map frame.
     */
    void look_for_tables_(const group14_interfaces::msg::RangePointArray &cluster,
                          geometry_msgs::msg::TransformStamped &tf);

    /**
     * @brief Estimates circle parameters (center and radius) by fitting a cluster of points
     * using Kasa's algebraic method (least squares). See I. Kåsa, "A circle fitting procedure and its
     * error analysis," in IEEE Transactions on Instrumentation and Measurement, 1976.
     * @param cluster Reference to a RangePointArray message representing the points of the
     * cluster to fit.
     * @return a `Circle` object containing the center coordinates (x, y) and the radius
     * if the calculation is successful, `std::nullopt` otherwise.
     */
    static std::optional<Circle> fit_circle_Kasa_(const group14_interfaces::msg::RangePointArray &cluster);

    /**
     * @brief Validates a candidate circle against the point cluster using the estimated radius
     * (rejects noise (too small radius) or walls (too large radius)) and MSE (Mean Squared Error).
     * @param cluster The cluster of points (RangePointArray) to validate against.
     * @param circle The estimated circle parameters (center and radius).
     * @return true if the circle meets both radius and MSE criteria.
     * @return false otherwise.
     */
    bool validate_circle_fit_(const group14_interfaces::msg::RangePointArray &cluster,
                              const Circle &circle);

    /**
     * @brief Queries the TF2 buffer to find the transform from the source frame in `header`
     * to the `dst_frame_id` at the timestamp `header.stamp`.
     * @param header The header containing the source frame ID and the timestamp.
     * @param dst_frame_id The target frame ID (e.g., "map").
     * @return std::optional containing the TransformStamped if successful, or std::nullopt if the lookup fails.
     */
    std::optional<geometry_msgs::msg::TransformStamped> transform_(
        std_msgs::msg::Header header, const std::string dst_frame_id);

    /**
     * @brief Publishes visualization markers for detected tables to RViz.
     * @param tables_ The vector of currently tracked tables.
     * @param NUM_DETECTIONS_THRESHOLD The minimum number of detections required for a table to be visualized.
     */
    void DEBUG_publish_table_markers_(std::vector<Table> &tables_, int NUM_DETECTIONS_THRESHOLD);

    /**
     * @brief Publishes a marker for a single candidate circle.
     * @param circle The circle parameters (center and radius) to visualize.
     */
    void DEBUG_publish_single_circle_marker_(Circle &circle);

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<group14_interfaces::msg::ClusterArray>::SharedPtr clustered_scan_subscription_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr table_markers_publisher_;

    std::vector<Table> tables_; // Detected tables
    std::shared_ptr<rclcpp::Service<group14_interfaces::srv::LookForTables>> service_;

    const float MIN_RADIUS = 0.02;
    const float MAX_RADIUS = 0.50;
    const double MSE_THRESHOLD = 0.0025;
    const int NUM_DETECTIONS_THRESHOLD = 20;
};

#endif