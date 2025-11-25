#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <memory>
#include <functional>

using namespace std::chrono_literals;

class CorridorNavigator : public rclcpp::Node
{
public:
    CorridorNavigator() : Node("corridor_navigator")
    {
        // 1. Initialize Publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // 2. Initialize Subscriber for the corridor trigger signal
        corridor_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/corridor_trigger", 
            10, 
            std::bind(&CorridorNavigator::corridor_trigger_callback, this, std::placeholders::_1)
        );

        // 3. Setup Timer for continuous publishing
        // The timer publishes velocity commands only if in_corridor_ is true.
        publish_timer_ = this->create_wall_timer(
            100ms, 
            std::bind(&CorridorNavigator::publish_cmd_vel, this)
        );

        RCLCPP_INFO(this->get_logger(), "Corridor Navigator initialized. Awaiting /corridor_trigger.");
    }

private:
    const double CORRIDOR_LINEAR_VELOCITY = 0.3; // m/s
    
    // State to know if we are in the corridor and should be moving
    bool in_corridor_ = false;

    // ROS 2 Components
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr corridor_trigger_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    /**
     * @brief Callback called by /corridor_trigger to change the navigation state.
     * @param msg std_msgs::msg::Bool message containing the corridor state.
     */
    void corridor_trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data == true && !in_corridor_) {
            // TRUE Signal: CORRIDOR START
            in_corridor_ = true;
            RCLCPP_INFO(this->get_logger(), "Corridor START detected. Starting movement at %.1f m/s.", CORRIDOR_LINEAR_VELOCITY);
            // We don't publish here; the publish_timer handles it
            
        } else if (msg->data == false && in_corridor_) {
            // FALSE Signal: CORRIDOR END
            in_corridor_ = false;
            // Publish Twist with 0 velocity to ensure a full stop
            stop_movement(); 
            RCLCPP_INFO(this->get_logger(), "Corridor END detected. Stopping movement.");
        }
    }
    
    /**
     * @brief Publishes a Twist message with zero linear velocity.
     */
    void stop_movement()
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(twist_msg);
    }

    /**
     * @brief Continuously publishes velocity to /cmd_vel if in_corridor_ is true.
     */
    void publish_cmd_vel()
    {
        if (in_corridor_) {
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = CORRIDOR_LINEAR_VELOCITY;
            twist_msg.angular.z = 0.0; // No rotation (go straight)
            cmd_vel_pub_->publish(twist_msg);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CorridorNavigator>());
    rclcpp::shutdown();
    return 0;
}