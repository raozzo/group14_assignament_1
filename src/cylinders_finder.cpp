#include "group14_assignment_1/cylinders_finder.hpp"

#include <cmath>
#include <Eigen/Dense>

CylindersFinder::CylindersFinder(const rclcpp::NodeOptions &options)
    : Node("cylinders_finder_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Cylinders finder node has been started.");

    // TF2 listener initialization
    RCLCPP_INFO(this->get_logger(), "Initializing TF2 listener");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initial pose availability listener to decide when to start node logic
    RCLCPP_INFO(this->get_logger(), "Waiting for /laser_scan_clustering to look for circular tables");
    clustered_scan_subscription_ =
        this->create_subscription<group14_interfaces::msg::ClusterArray>(
            "/laser_scan_clustering",
            rclcpp::QoS(10),
            std::bind(&CylindersFinder::process_clustered_scan_, this, std::placeholders::_1));

    // Initialize a marker publisher to show the detected tables during the robot motion
    table_markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("table_markers_topic", 10);
}

void CylindersFinder::process_clustered_scan_(const group14_interfaces::msg::ClusterArray &scan)
{
    // try a conversion to map reference frame
    std_msgs::msg::Header header = scan.clusters.front().points.front().point.header;
    std::optional<geometry_msgs::msg::TransformStamped> tf_to_map = transform_(header, "map");
    if (tf_to_map.has_value())
    {
        /* try to fit a circle to every cluster */
        for (const group14_interfaces::msg::RangePointArray &cluster : scan.clusters)
            look_for_tables_(cluster, tf_to_map.value());

        DEBUG_publish_table_markers_(tables_, NUM_DETECTIONS_THRESHOLD);
    }
    else
    {
        // if the scan cannot be referenced to the map frame, ignore it
    }
}

void CylindersFinder::look_for_tables_(const group14_interfaces::msg::RangePointArray &cluster,
                                       geometry_msgs::msg::TransformStamped &tf)
{
    // Try to fit a circle to the cluster points with Kasa mathod
    std::optional<Circle> circle = fit_circle_Kasa_(cluster);

    // If the cluster well represents a circular arc
    if (circle.has_value() && validate_circle_fit_(cluster, circle.value()))
    {
        // Build a new circle object (i.e. candidate table) with respect to the map frame
        geometry_msgs::msg::PointStamped center_wrt_map;
        tf2::doTransform(circle.value().center, center_wrt_map, tf);
        Circle candidate_table(center_wrt_map, circle.value().r);

        // Compare this candidate table with already seen tables
        bool match_found = false;
        for (Table &t : tables_)
        {
            if (Table::is_same_table(t, candidate_table))
            {
                // Compute the distance between the robot [(0, 0) in the laserscan frame] and the circle center
                double distance = std::hypot(circle.value().center.point.x, circle.value().center.point.y);

                // Update the table estimate
                t.update(candidate_table, distance, static_cast<int>(cluster.points.size()));

                match_found = true;
                break;
            }
        }

        if (!match_found)
        {
            // New table discovered
            tables_.emplace_back(candidate_table);
        }
    }
}

std::optional<Circle> CylindersFinder::fit_circle_Kasa_(const group14_interfaces::msg::RangePointArray &cluster)
{
    int n_points = static_cast<int>(cluster.points.size());

    // for each point (x, y):
    // (x - x_c)^2 + (y - y_c)^2 = R^2
    // (x^2 - 2xx_c + x_c^2) + (y^2 - 2yy_c + y_c^2) = R^2
    // x*(2x_c) + y*(2y_c) + 1*(R^2 - x_c^2 - y_c^2) = x^2 + y^2
    // A*X = b

    Eigen::MatrixXd A(n_points, 3);
    Eigen::VectorXd b(n_points);
    for (int i = 0; i < n_points; ++i)
    {
        double x = cluster.points[i].point.point.x;
        double y = cluster.points[i].point.point.y;
        A(i, 0) = x;
        A(i, 1) = y;
        A(i, 2) = 1.0;
        b(i) = x * x + y * y;
    }

    // Least squares solution of the A*X = b system
    Eigen::Vector3d X = A.colPivHouseholderQr().solve(b);

    // Circle params decoding
    float x_center = X(0) / 2.0;
    float y_center = X(1) / 2.0;
    float radius = std::sqrt(X(2) + x_center * x_center + y_center * y_center); // X(2) = R^2 - x_c^2 - y_c^2

    // Radius validation
    if (std::isnan(radius))
        return std::nullopt;

    return Circle(x_center, y_center, radius, cluster.points.front().point.header);
}

bool CylindersFinder::validate_circle_fit_(const group14_interfaces::msg::RangePointArray &cluster,
                                           const Circle &circle)
{
    // Radius check
    if (circle.r < MIN_RADIUS || circle.r > MAX_RADIUS)
        return false;

    // MSE check
    int n_points = static_cast<int>(cluster.points.size());
    double MSE = 0;
    for (const group14_interfaces::msg::RangePoint &p : cluster.points)
        MSE += std::pow(std::hypot(p.point.point.x - circle.center.point.x, p.point.point.y - circle.center.point.y) - circle.r, 2);
    MSE /= n_points;

    if (MSE > MSE_THRESHOLD)
        return false;

    return true;
}

std::optional<geometry_msgs::msg::TransformStamped> CylindersFinder::transform_(
    std_msgs::msg::Header header,
    const std::string dst_frame_id)
{
    try
    {
        geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
            dst_frame_id,
            header.frame_id,
            header.stamp,
            rclcpp::Duration::from_seconds(0.1));
        return tf;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Transform not available: %s", ex.what());
        return std::nullopt;
    }
}

void CylindersFinder::DEBUG_publish_table_markers_(std::vector<Table> &tables_, int NUM_DETECTIONS_THRESHOLD)
{
    visualization_msgs::msg::MarkerArray markers_msg;

    visualization_msgs::msg::Marker delete_all_marker;
    delete_all_marker.action = 3; // 3 = DELETEALL
    markers_msg.markers.push_back(delete_all_marker);

    int id = 0;
    for (const auto &table : tables_)
    {
        if (table.num_detections < NUM_DETECTIONS_THRESHOLD)
            continue;

        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = table.circle.center.header.frame_id;
        marker.header.stamp = this->now();
        marker.ns = "tables";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = table.circle.center.point.x;
        marker.pose.position.y = table.circle.center.point.y;
        marker.pose.position.z = 0.025;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = table.circle.r * 2.0;
        marker.scale.y = table.circle.r * 2.0;
        marker.scale.z = 0.05;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.8f;

        marker.lifetime = rclcpp::Duration::from_seconds(0);

        markers_msg.markers.push_back(marker);
    }

    table_markers_publisher_->publish(markers_msg);
}

void CylindersFinder::DEBUG_publish_single_circle_marker_(Circle &circle)
{
    (void)NUM_DETECTIONS_THRESHOLD;
    std::vector<Table> vector = {Table(circle)};
    DEBUG_publish_table_markers_(vector, 0);
}

/* TABLE CLASS */

Table::Table(Circle circle_, double initial_weight, int num_detections_)
    : circle(circle_), cumulative_weight(initial_weight), num_detections(num_detections_) {}

void Table::update(const Circle &new_circle, double distance, int cluster_size)
{
    // Compute a weight for the current measure (proportional to the number of points in the cluster,
    // inversely proportional to the distance at which the scan has been taken)
    double weight = cluster_size / (distance + 0.01);

    // Update the cumulative weight
    cumulative_weight += weight;

    // Update circle parameters estimate
    float x = circle.center.point.x; // old estimate of x
    float y = circle.center.point.y; // old estimate of y
    float r = circle.r;              // old estimate of r
    circle.center.point.x = x + (weight / cumulative_weight) * (new_circle.center.point.x - x);
    circle.center.point.y = y + (weight / cumulative_weight) * (new_circle.center.point.y - y);
    circle.r = r + (weight / cumulative_weight) * (new_circle.r - r);

    num_detections++;
}

bool Table::is_same_table(const Table &t, const Circle &c)
{
    return std::hypot(t.circle.center.point.x - c.center.point.x, t.circle.center.point.y - c.center.point.y) < DISTANCE_THRESHOLD;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CylindersFinder>());
    rclcpp::shutdown();
    return 0;
}