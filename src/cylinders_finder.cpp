#include "group14_assignment_1/cylinders_finder.hpp"

CylindersFinder::CylindersFinder(const rclcpp::NodeOptions &options)
    : Node("cylinders_finder_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Cylinders finder node has been started.");

    // TF2 listener initialization
    RCLCPP_INFO(this->get_logger(), "Initializing TF2 listener");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscription to LIDAR data
    RCLCPP_INFO(this->get_logger(), "Subscribing to /scan topic");
    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::QoS(10),
        std::bind(&CylindersFinder::process_scan_reading, this, std::placeholders::_1));
}

void CylindersFinder::process_scan_reading(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    (void)scan;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CylindersFinder>());
    rclcpp::shutdown();
    return 0;
}