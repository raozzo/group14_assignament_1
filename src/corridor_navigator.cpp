#include <memory>
#include <functional>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Assuming your custom Action interface is defined here
// REPLACE WITH YOUR ACTUAL PACKAGE NAME
#include "group14_interfaces/action/corridor_walk.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // Required for the Result structure

using namespace std::placeholders;
using namespace std::chrono_literals;
using CorridorNavigation = group14_interfaces::action::CorridorWalk;
using GoalHandleCorridorNavigation = rclcpp_action::ServerGoalHandle<CorridorNavigation>;

/**
 * @brief CorridorNavigator node acts as a pure Action Server.
 * It executes the 'corridor_walk' task upon receiving a Goal 
 * and provides feedback and a final result.
 */
class CorridorNavigator : public rclcpp::Node
{
public:
    CorridorNavigator() : Node("corridor_navigator")
    {
        this->action_server_ = rclcpp_action::create_server<CorridorNavigation>(
            this,
            "corridor_walk", 
            std::bind(&CorridorNavigator::handle_goal, this, _1, _2),
            std::bind(&CorridorNavigator::handle_cancel, this, _1),
            std::bind(&CorridorNavigator::handle_accepted, this, _1));

        // Publisher for robot velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_corridor", 10);

        RCLCPP_INFO(this->get_logger(), "Corridor Navigator action server initialized. Waiting for GOALS from Cervellone...");
    }

private:
    rclcpp_action::Server<CorridorNavigation>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const CorridorNavigation::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received navigation goal. Estimated length: %.2f m", goal->corridor_length_estimate);
        (void)uuid;
        // Accept and execute the goal immediately
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Goal acceptance handler. Start execution in a new thread.
    void handle_accepted(const std::shared_ptr<GoalHandleCorridorNavigation> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted.");
        std::thread{std::bind(&CorridorNavigator::execute, this, _1), goal_handle}.detach();
    }


    // Goal cancellation request from a client
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleCorridorNavigation> goal_handle)
    {
        RCLCPP_WARN(this->get_logger(), "Received request to cancel goal.");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void execute(const std::shared_ptr<GoalHandleCorridorNavigation> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Corridor Navigation started.");
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<CorridorNavigation::Result>();
    auto feedback = std::make_shared<CorridorNavigation::Feedback>();

    const double TARGET_LENGTH = goal->corridor_length_estimate;
    double current_distance = 0.0;
    const double LINEAR_VELOCITY = 0.2; // m/s
    const double FREQUENCY_HZ = 15.0; // Set high frequency (50 Hz)
    const double CYCLE_TIME = 1.0 / FREQUENCY_HZ; // seconds for update cycle 

    // Start moving
    auto cmd_vel_msg = geometry_msgs::msg::Twist();
    cmd_vel_msg.linear.x = LINEAR_VELOCITY;
    cmd_vel_pub_->publish(cmd_vel_msg);

    // Main Navigation Loop (Simulation)
    rclcpp::Rate loop_rate(FREQUENCY_HZ); // Use the new high frequency
    
    while (rclcpp::ok() && current_distance < TARGET_LENGTH) {
        
        // Check for cancellation request
        if (goal_handle->is_canceling()) {
            
            // --- START MODIFICATION: Rimuovi comando di stop in caso di cancellazione ---
            // The Mux will switch back to Nav2, which must take over immediately.
            // Sending a stop command here risks a brief, undesirable stop.
            /* cmd_vel_msg.linear.x = 0.0; // Old: Stop robot
            cmd_vel_pub_->publish(cmd_vel_msg);
            */
            // --- END MODIFICATION: Rimuovi comando di stop in caso di cancellazione ---
            
            result->corridor_end = false;
            result->current_pose.header.frame_id = "map";
            result->current_pose.pose.position.x = current_distance; 
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "Navigation CANCELED at %.2fm.", current_distance);
            return;
        }

        // Update distance traveled and publish feedback
        current_distance += LINEAR_VELOCITY * CYCLE_TIME;
        if (current_distance > TARGET_LENGTH) {
            current_distance = TARGET_LENGTH; // Cap at the goal
        }
        
        feedback->distance_traveled = current_distance;
        goal_handle->publish_feedback(feedback);
        RCLCPP_DEBUG(this->get_logger(), "Feedback: Traveled %.2fm / %.2fm", current_distance, TARGET_LENGTH);

        loop_rate.sleep();
    }

    // Goal succeeded (Corridor end reached)
    
    // --- START MODIFICATION: Rimuovi comando di stop in caso di successo ---
    // The Mux will switch back to Nav2, which must take over immediately.
    // If Nav2 has already completed its goal, it will publish a velocity of zero itself.
    /*
    cmd_vel_msg.linear.x = 0.0; // Old: Stop robot
    cmd_vel_pub_->publish(cmd_vel_msg);
    */
    // --- END MODIFICATION: Rimuovi comando di stop in caso di successo ---

    result->corridor_end = true;
    result->current_pose.header.frame_id = "map";
    result->current_pose.pose.position.x = TARGET_LENGTH; // Final position
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Navigation SUCCEEDED! Reached end of corridor (%.2fm).", TARGET_LENGTH);
}
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CorridorNavigator>());
    rclcpp::shutdown();
    return 0;
}