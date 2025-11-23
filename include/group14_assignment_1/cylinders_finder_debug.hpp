#ifndef GROUP14_CYLINDERS_FINDER_DEBUG_HPP
#define GROUP14_CYLINDERS_FINDER_DEBUG_HPP

#include "rclcpp/rclcpp.hpp"

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "group14_assignment_1/cylinders_finder.hpp"

class CylindersFinderDebug
{
public:
    static void publish_markers(
        std::vector<Table> &tables_,
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &marker_publisher_,
        int NUM_DETECTIONS_THRESHOLD,
        rclcpp::Time now);

    static void publish_clusters(
        const std::vector<std::vector<group14::RangePoint>> &clusters,
        const std_msgs::msg::Header &header,
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &marker_publisher_);

    static std_msgs::msg::ColorRGBA get_color(int index);
};

#endif