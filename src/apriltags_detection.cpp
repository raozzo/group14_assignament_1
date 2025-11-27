#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"

class ApriltagsDetection : public rclcpp::Node
{
public:
    explicit ApriltagsDetection(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("apriltags_detection", options)
    {
        RCLCPP_INFO(this->get_logger(), "Apriltags detection node has been started.");

        // Tf listeners initialization
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // when apriltags are both detected... calculate goal position and send it to the nav stack
        apriltags_subscription_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "/apriltag/detections",
            rclcpp::SensorDataQoS(),
            std::bind(&ApriltagsDetection::apriltag_callback, this, std::placeholders::_1));

        // Initializing goal_in_map_frame_topic
        goal_in_map_publisher_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_in_map_frame", 10);
    }

private:
    /**
     * @brief Looks for both `tag36h11:1` and `tag36h11:10` any time the topic `/apriltag/detections`
     * publishes a new message, then calls the `calculate_goal` function to compute the midlle point
     * between the two apriltags.
     * @param msg the received message
     */
    void apriltag_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        if (msg->detections.empty())
            return;

        bool tag1_detected = false;
        bool tag2_detected = false;

        for (const apriltag_msgs::msg::AprilTagDetection &tag : msg->detections)
        {
            if (tag.id == TAG1_ID)
                tag1_detected = true;
            if (tag.id == TAG2_ID)
                tag2_detected = true;
        }

        if (tag1_detected && tag2_detected)
        {
            calculate_goal();
        }
    }

    /**
     * @brief Transforms the frames `tag36h11:1` and `tag36h11:10` into `map` frame reference,
     * then computes the middle point and publishes it on the `goal_in_map_frame` topic as a
     * `geometry_msgs::msg::PoseStamped` message.
     */
    void calculate_goal()
    {
        // RCLCPP_INFO(this->get_logger(), "Starting calculating goal");
        bool tf_available = false;
        geometry_msgs::msg::TransformStamped tf_tag1, tf_tag2;

        try
        {
            tf_tag1 = tf_buffer_->lookupTransform(MAP_FRAME_ID, TAG1_FRAME_ID, tf2::TimePointZero);
            tf_tag2 = tf_buffer_->lookupTransform(MAP_FRAME_ID, TAG2_FRAME_ID, tf2::TimePointZero);
            tf_available = true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Could not find transforms from %s or %s to %s: %s",
                                 TAG1_FRAME_ID.c_str(),
                                 TAG2_FRAME_ID.c_str(),
                                 MAP_FRAME_ID.c_str(),
                                 ex.what());
        }

        if (tf_available)
        {
            // RCLCPP_INFO(this->get_logger(), "Calculating midpoint between apriltags");
            geometry_msgs::msg::PoseStamped mid_point_in_map;
            mid_point_in_map.header.stamp = this->get_clock()->now();
            mid_point_in_map.header.frame_id = MAP_FRAME_ID;
            mid_point_in_map.pose.position.x =
                (tf_tag1.transform.translation.x + tf_tag2.transform.translation.x) / 2.0;
            mid_point_in_map.pose.position.y =
                (tf_tag1.transform.translation.y + tf_tag2.transform.translation.y) / 2.0;
            mid_point_in_map.pose.position.z = 0.0;
            mid_point_in_map.pose.orientation.w = 1.0;

            /*
            RCLCPP_INFO(this->get_logger(), ">>> FINAL GOAL: [x: %.2f, y: %.2f] at time %d <<<",
                        mid_point_in_map.pose.position.x,
                        mid_point_in_map.pose.position.y,
                        mid_point_in_map.header.stamp.sec);
            */
            goal_in_map_publisher_->publish(mid_point_in_map);
        }
    }

    /* CONSTANTS */
    const std::string MAP_FRAME_ID = "map";
    const std::string TAG1_FRAME_ID = "tag36h11:1";
    const std::string TAG2_FRAME_ID = "tag36h11:10";
    const int TAG1_ID = 1;
    const int TAG2_ID = 10;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltags_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_in_map_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ApriltagsDetection>());
    rclcpp::shutdown();
    return 0;
}