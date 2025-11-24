#include "group14_assignment_1/cylinders_finder_debug.hpp"

void CylindersFinderDebug::publish_markers(
    std::vector<Table> &tables_,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &marker_publisher_,
    int NUM_DETECTIONS_THRESHOLD,
    rclcpp::Time now)
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
        marker.header.stamp = now;
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

    marker_publisher_->publish(markers_msg);
}

void CylindersFinderDebug::publish_single_circle(
    group14::Circle &circle,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &marker_publisher_,
    int NUM_DETECTIONS_THRESHOLD,
    rclcpp::Time now)
{
    std::vector<Table> vector = {Table(circle)};
    publish_markers(vector, marker_publisher_, 0, now);
}

void CylindersFinderDebug::publish_clusters(
    const std::vector<std::vector<group14::RangePoint>> &clusters,
    const std_msgs::msg::Header &header,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &marker_publisher_)
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

    for (size_t i = 0; i < clusters.size(); ++i)
    {
        std_msgs::msg::ColorRGBA color = get_color(static_cast<int>(i));

        for (const auto &rp : clusters[i])
        {
            marker.points.push_back(rp.point.point);
            marker.colors.push_back(color);
        }
    }

    marker.lifetime = rclcpp::Duration::from_seconds(0);

    markers_msg.markers.push_back(marker);

    marker_publisher_->publish(markers_msg);
}

std_msgs::msg::ColorRGBA CylindersFinderDebug::get_color(int index)
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