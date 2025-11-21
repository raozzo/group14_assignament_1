#ifndef GROUP14_CYLINDERS_FINDER_HPP
#define GROUP14_CYLINDERS_FINDER_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "group14_interfaces/msg/tables_array.hpp"

class CylindersFinder : public rclcpp::Node
{
public:
    explicit CylindersFinder(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    void process_scan_reading(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    std::shared_ptr<rclcpp::Publisher<group14_interfaces::msg::TablesArray, std::allocator<void>>> tables_publisher_;
};

#endif