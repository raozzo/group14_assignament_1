#include "group14_assignment_1/laser_scan_clustering.hpp"

#include "group14_interfaces/msg/range_point.hpp"
#include "group14_interfaces/msg/range_point_array.hpp"

LaserScanClustering::LaserScanClustering(const rclcpp::NodeOptions &options)
    : Node("laser_scan_clustering_node", options)
{
    RCLCPP_INFO(this->get_logger(), "LaserScan clustering node has been started.");

    // TF2 listener initialization
    RCLCPP_INFO(this->get_logger(), "Initializing TF2 listener");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initial pose availability listener to decide when to start node logic
    RCLCPP_INFO(this->get_logger(), "Waiting for /initialpose to start logic...");
    initial_pose_subscription_ =
        this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            rclcpp::QoS(10),
            std::bind(&LaserScanClustering::initial_pose_callback_, this, std::placeholders::_1));

    // Initialize a publisher for clustered laser scan measures
    clusters_publisher_ =
        this->create_publisher<group14_interfaces::msg::ClusterArray>("laser_scan_clustering", 10);

    // Initialize a publisher to display the clustered laser scan measures
    clusters_marker_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("cluster_marker_topic", 10);
}

void LaserScanClustering::initial_pose_callback_(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    (void)msg;

    // avoid re-initializations is case of further received initial poses
    if (lidar_subscription_ != nullptr)
        return;

    // Starting logic
    RCLCPP_INFO(this->get_logger(), "Initial pose received. Starting logic...");

    // Subscription to LIDAR data
    RCLCPP_INFO(this->get_logger(), "Subscribing to /scan topic");
    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::QoS(10),
        std::bind(&LaserScanClustering::process_scan_, this, std::placeholders::_1));
}

void LaserScanClustering::process_scan_(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    int scan_dim = static_cast<int>(scan->ranges.size());
    if (scan_dim == 0)
        return;

    group14_interfaces::msg::ClusterArray clusters;
    cluster_ranges_(scan, clusters);
    clusters_publisher_->publish(clusters); // publish to topic /laser_scan_clustering

    // DEBUG_publish_clusters_(clusters, scan->header);
}

void LaserScanClustering::cluster_ranges_(
    const sensor_msgs::msg::LaserScan::SharedPtr &scan,
    group14_interfaces::msg::ClusterArray &clusters)
{
    clusters.clusters = {};

    std_msgs::msg::Header header = scan->header;
    int scan_dim = static_cast<int>(scan->ranges.size());
    float range_min = scan->range_min;
    float range_max = scan->range_max;

    // For each range measurement
    for (int i = 0; i < scan_dim; ++i)
    {
        // Remove unreliable measures
        float range = scan->ranges[i];
        if (range < range_min || range > range_max || std::isinf(range) || std::isnan(range))
            continue;

        // Build a RangePoint representing the measure
        float angle = scan->angle_min + i * scan->angle_increment;
        group14_interfaces::msg::RangePoint rp;
        rp.range = range;
        rp.angle = angle;
        rp.index = i;
        rp.point = rangeToPointStamped_(range, angle, header);

        // Assign the current rp to the correct cluster
        if (clusters.clusters.empty())
        {
            group14_interfaces::msg::RangePointArray cluster;
            cluster.points = std::vector<group14_interfaces::msg::RangePoint>{rp};
            clusters.clusters.emplace_back(cluster);
        }
        else
        {
            group14_interfaces::msg::RangePoint prev_rp = clusters.clusters.back().points.back();
            double euclidean_dist = std::hypot(rp.point.point.x - prev_rp.point.point.x,
                                               rp.point.point.y - prev_rp.point.point.y);
            double D_max = compute_dynamic_D_max_(prev_rp.range, (angle - prev_rp.angle),
                                                  INCIDENCE_ANGLE_THRESHOLD, SCAN_NOISE_FLOOR);

            if (euclidean_dist > D_max)
            {
                group14_interfaces::msg::RangePointArray cluster;
                cluster.points = std::vector<group14_interfaces::msg::RangePoint>{rp};
                clusters.clusters.emplace_back(cluster);
            }
            else
                clusters.clusters.back().points.emplace_back(rp);
        }
    }

    // Merge the 1st and the last clusters if their extremities are closer than D_max
    group14_interfaces::msg::RangePoint rp_1 = clusters.clusters.back().points.back();
    group14_interfaces::msg::RangePoint rp_2 = clusters.clusters.front().points.front();
    double euclidean_dist = std::hypot(rp_2.point.point.x - rp_1.point.point.x,
                                       rp_2.point.point.y - rp_1.point.point.y);
    double D_max = compute_dynamic_D_max_(rp_1.range, (rp_2.angle + 2 * PI - rp_1.angle),
                                          INCIDENCE_ANGLE_THRESHOLD, SCAN_NOISE_FLOOR);
    if (euclidean_dist > D_max)
        merge_cyclic_clusters_(clusters);

    // Remove clusters having less than MIN_CLUSTER_POINTS points
    clusters.clusters.erase(
        std::remove_if(clusters.clusters.begin(), clusters.clusters.end(),
                       [this](group14_interfaces::msg::RangePointArray &x)
                       { return static_cast<int>(x.points.size()) < MIN_CLUSTER_POINTS; }),
        clusters.clusters.end());
}

void LaserScanClustering::merge_cyclic_clusters_(
    group14_interfaces::msg::ClusterArray &clusters)
{
    group14_interfaces::msg::RangePointArray &front = clusters.clusters.front();
    group14_interfaces::msg::RangePointArray &back = clusters.clusters.back();
    front.points.reserve(front.points.size() + back.points.size());
    front.points.insert(front.points.begin(), back.points.begin(), back.points.end());

    clusters.clusters.pop_back();
}

double LaserScanClustering::compute_dynamic_D_max_(
    float prev_valid_range, float angle_increment,
    float INCIDENCE_ANGLE_THRESHOLD, float NOISE_FLOOR)
{
    return (prev_valid_range * sin(angle_increment)) / sin(INCIDENCE_ANGLE_THRESHOLD - angle_increment) + NOISE_FLOOR;
}

geometry_msgs::msg::PointStamped LaserScanClustering::rangeToPointStamped_(
    float range, float angle, std_msgs::msg::Header header)
{
    geometry_msgs::msg::PointStamped p;
    p.header = header;
    p.point.x = range * std::cos(angle);
    p.point.y = range * std::sin(angle);

    return p;
}

void LaserScanClustering::DEBUG_publish_clusters_(
    const group14_interfaces::msg::ClusterArray &clusters,
    const std_msgs::msg::Header &header)
{
    visualization_msgs::msg::MarkerArray markers_msg;
    visualization_msgs::msg::Marker marker;

    marker.header = header;
    marker.ns = "clusters_debug";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;

    for (size_t i = 0; i < clusters.clusters.size(); ++i)
    {
        std_msgs::msg::ColorRGBA color = DEBUG_get_color_(static_cast<int>(i));

        for (const auto &rp : clusters.clusters[i].points)
        {
            marker.points.push_back(rp.point.point);
            marker.colors.push_back(color);
        }
    }

    marker.lifetime = rclcpp::Duration::from_seconds(0);

    markers_msg.markers.push_back(marker);
    clusters_marker_publisher_->publish(markers_msg);
}

std_msgs::msg::ColorRGBA LaserScanClustering::DEBUG_get_color_(int index)
{
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;

    switch (index % 6)
    {
    case 0:
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        break;
    case 1:
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        break;
    case 2:
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        break;
    case 3:
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.0;
        break;
    case 4:
        color.r = 0.0;
        color.g = 1.0;
        color.b = 1.0;
        break;
    case 5:
        color.r = 1.0;
        color.g = 0.0;
        color.b = 1.0;
        break;
    }
    return color;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanClustering>());
    rclcpp::shutdown();
    return 0;
}