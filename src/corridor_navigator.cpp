#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp" 
#include <chrono>
#include <memory>
#include <functional>
#include <cmath>
#include <algorithm> 
#include <tf2/LinearMath/Quaternion.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

using namespace std::chrono_literals;

// Target Y coordinate 
const double TARGET_Y_MAP = 0.0; 
// Target Yaw orientation (Aligned with X-axis in 'map' frame): 0.0
const double TARGET_YAW = 0.0;   

const double CORRIDOR_LINEAR_VEL = 0.4; // Set the desired velocity (m/s) in the corridor when Nav2 navigation goal is temporarily canceled
const double ALIGNMENT_THRESHOLD_Y = 0.1; // Tolerance for Y error (m) w.r.t. the target
const double ALIGNMENT_THRESHOLD_YAW = 0.0003; // Tolerance for Yaw error (rad) w.r.t. the target
const double K = 1.5;  // Multiplication constant of velocity to obtain a quicker alignment
const double MAX_LINEAR_Y_VEL = 0.42;  // Threshold to avoid linear velocity to reach too high values
const double MAX_ANGULAR_Z_VEL = 0.40; // Threshold to avoid angular velocity to reach too high values

class CorridorNavigator : public rclcpp::Node
{
public:
    CorridorNavigator() : Node("corridor_navigator")
    {
        // Publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriber for the corridor trigger signal
        corridor_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/corridor_trigger", 
            10, 
            std::bind(&CorridorNavigator::corridor_trigger_callback, this, std::placeholders::_1)
        );

        publish_timer_ = this->create_wall_timer(
            150ms, 
            std::bind(&CorridorNavigator::publish_cmd_vel, this)
        );
        
        // Subscriber for Odometry/Pose
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 
            10, 
            std::bind(&CorridorNavigator::odom_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Corridor Navigator initialized. Awaiting /corridor_trigger.");
    }

private:
    // State to know if we are in the corridor and should be moving
    bool in_corridor_ = false;
    
    // State variables for alignment and position
    bool aligned_ = false;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr corridor_trigger_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; 


    // Converts a Quaternion message to Euler angles and returns the yaw.
    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
    {
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }


    // Callback called by /odom to update robot position and orientation.
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Assuming /odom pose is in the global map frame
        current_y_ = msg->pose.pose.position.y;
        current_yaw_ = get_yaw_from_quaternion(msg->pose.pose.orientation);
    }

    // Callback called by /corridor_trigger to change the navigation state.
    void corridor_trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // TRUE Signal: CORRIDOR START
        if (msg->data == true && !in_corridor_) {
            
            in_corridor_ = true;

            // Force re-alignment when the corridor is entered
            aligned_ = false; 
            RCLCPP_INFO(this->get_logger(), 
                "Corridor START detected. Initiating alignment to Y=%.2f and Yaw=%.2f.", 
                TARGET_Y_MAP, TARGET_YAW);
            
        } 
        // FALSE Signal: CORRIDOR END
        else if (msg->data == false && in_corridor_) {
            
             // Cancel the timer to stop publishing on /cmd_vel
            publish_timer_->cancel();

            corridor_trigger_sub_.reset();

            in_corridor_ = false;
            aligned_ = false;
            RCLCPP_INFO(this->get_logger(), "Corridor END detected. Stopping movement.");

            // Shutdown the node after corridor exit
             RCLCPP_INFO(this->get_logger(), "---Shutting down the Corridor Navigator---");
             rclcpp::shutdown();

        }
    }
    

    // Continuously publish velocity to /cmd_vel. Manages the alignment phase and the straight movement phase.
    void publish_cmd_vel()
    {
        if (in_corridor_) {
            auto twist_msg = geometry_msgs::msg::Twist();
            
            // Re-aligment when nav2 goal is temporarily canceled in order to navigate the corridor straightforward
            if (!aligned_) {
                
                // Calculate lateral (Y) error
                double error_y = TARGET_Y_MAP - current_y_;
                // Calculate the velocity basing it on the calculated error,  be sure that it doesn' t exceed pre-defined limiting values
                double limited_lin_y = std::clamp(K * error_y, -MAX_LINEAR_Y_VEL, MAX_LINEAR_Y_VEL);

                // Calculate angular (Yaw) error
                double error_yaw = TARGET_YAW - current_yaw_; 
                // Normalize error_yaw to be within [-pi, pi]
                double error_norm = std::atan2(std::sin(error_yaw), std::cos(error_yaw));
                // Calculate the velocity basing it on the calculated error,  be sure that it doesn' t exceed pre-defined limiting values
                double limited_ang_z = std::clamp(K* error_norm, -MAX_ANGULAR_Z_VEL, MAX_ANGULAR_Z_VEL);

                RCLCPP_INFO(this->get_logger(), "Actual distance to y target : %.5f, Actual normalized yaw error to the angular target: %.6f", error_y, error_yaw);
                RCLCPP_INFO(this->get_logger(), "Y linear velocity signal: %.5f, Z angular velocity signal: %.6f", limited_lin_y, limited_ang_z);
                

                // Check if alignment is completed for both y target and yaw target
                if (std::abs(error_y) < ALIGNMENT_THRESHOLD_Y && std::abs(error_norm) < ALIGNMENT_THRESHOLD_YAW) {
                    aligned_ = true;
                    RCLCPP_INFO(this->get_logger(), "Alignment complete. Moving forward with linear X velocity: %.1f).", CORRIDOR_LINEAR_VEL);
                }

                // Publish alignment command (X=0 during alignment)
                if (!aligned_){
                    twist_msg.linear.x = 0.0; // no X movements
                    twist_msg.linear.y = limited_lin_y;
                    twist_msg.angular.z = limited_ang_z;
                        
                    cmd_vel_pub_->publish(twist_msg);

                    return; 
                }
            }
            
            // Once aligned, proceed straight with fixed velocity, no corrections more needed
            twist_msg.linear.x = CORRIDOR_LINEAR_VEL;
            twist_msg.linear.y = 0.0;
            twist_msg.angular.z = 0.0; 

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