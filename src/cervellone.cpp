//TODO: 
//1. al momento la posizone degli april tag è calcolata su "map" nella consegna deve essere riportata su odom
//
//

//CPP LIBRARIES 
#include <chrono>
#include <memory>
#include <string>
#include <cmath>

//ROS LIBRARIES 
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "group14_assignment_1/utils.hpp"

using namespace std::chrono_literals;

class Cervellone : public rclcpp::Node
{
  public:
  
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  // constructor
  Cervellone(): Node("cervellone")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());    
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    // Create the publisher
    init_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10);
    
    // Give time for subscribers to connect, then publish
    // (A 2-second timer that runs ONCE is a simple way to do this)
    init_timer_ = this->create_wall_timer(10s, [this]() {
      this->initialize_localization();
      // Cancel this timer so it only runs once
      this->init_timer_->cancel();
    });

    timer_ = this->create_wall_timer(1.0s, std::bind(&Cervellone::calculate_goal, this));
    
    // Initialize action for nv2pose
    this->nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    RCLCPP_INFO(this->get_logger(), "Cervellone pensa. aspettando le tags...");
  
  }
  
  private:
  //initialize robot position to do 2dPose
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  void initialize_localization()
  {
    auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";

    // Set the known spawn coordinates from the tutor's file
    msg.pose.pose.position.x = 0; // -8.29;
    msg.pose.pose.position.y =0; //1.87;
    msg.pose.pose.position.z =0; //elsee0.01;

    msg.pose.pose.orientation.w = 1.0; 
    msg.pose.pose.orientation.z = 0.0;

    // AMCL requires a covariance matrix to accept the update
    // This sets a small variance (high confidence) in X, Y, and Yaw
    msg.pose.covariance[0] = 0.25;  // X variance
    msg.pose.covariance[7] = 0.25;  // Y variance
    msg.pose.covariance[35] = 0.06; // Yaw variance

    RCLCPP_INFO(this->get_logger(), "Publishing Initial Pose to AMCL");
    init_pose_pub_->publish(msg);
  }

  //TODO: Now tha that we have the 2d pose we can find the tags
    //1. read apriltag 
    //2. calculate the median point between apriltag
    //3. send goal to nav

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // we don't need to redo the calculation if already done 
  bool goal_sent_ = false;
  std::string tag1_frame_ = "tag36h11:10"; 
  std::string tag2_frame_ = "tag36h11:1";

  //FIX: THIS NEED TO BE FIXED (SEE TODO header)
  std::string world_frame_ = "map";
  //std::string world_frame_ = "odom";

  // start navigation to goal 
  void send_goal_to_nav2(geometry_msgs::msg::PoseStamped goal_pose)
  {
    if (!this->nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "NAV ACTION NOT READY");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal_pose; 

    RCLCPP_INFO(this->get_logger(), "Sending goal to Nav2...");

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
    // Callback when goal is accepted/rejected
    send_goal_options.goal_response_callback =
    [this](const GoalHandleNav::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "GOAL REJECTKD");
      } else {
        goal_sent_ = true;
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    };

    //NOTE:
    // Callback when navigation is finished
    send_goal_options.result_callback =
    [this](const GoalHandleNav::WrappedResult & result) {
     switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Navigation SUCCEEDED!");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Navigation was ABORTED");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Navigation was CANCELED");
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          break;
      }
    };

    // Send the goal
    this->nav_client_->async_send_goal(goal_msg, send_goal_options);
  };

  void calculate_goal()
  {
    if (goal_sent_) { return; } 
    
    geometry_msgs::msg::TransformStamped t1, t2;
    bool t1_found = false;
    bool t2_found = false;

    
    // to DEBug i try to find each singular tag, then it can be slimmed 
    // Try to find Tag 1
      try {
        t1 = tf_buffer_->lookupTransform(world_frame_, tag1_frame_, tf2::TimePointZero);
        t1_found = true;
      }  catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                  "Could not find %s: %s", tag1_frame_.c_str(), ex.what());
      }

       // Try to find Tag 2
      try {
        t2 = tf_buffer_->lookupTransform(world_frame_, tag2_frame_, tf2::TimePointZero);
        t2_found = true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                 "Could not find %s: %s", tag2_frame_.c_str(), ex.what());
      }
    

    // If BOTH are found
    if (t1_found && t2_found) {
      RCLCPP_INFO(this->get_logger(), "BOTH TAGS FOUND! Calculating midpoint...");
      
      //calculate the midpoint
      double mid_x = (t1.transform.translation.x + t2.transform.translation.x) / 2.0;
      double mid_y = (t1.transform.translation.y + t2.transform.translation.y) / 2.0;
      
      //creation of the pose goal
      geometry_msgs::msg::PoseStamped goal_pose;
      goal_pose.header.stamp = this->get_clock()->now();
      goal_pose.header.frame_id = world_frame_;
            
      goal_pose.pose.position.x = mid_x;
      goal_pose.pose.position.y = mid_y;
      goal_pose.pose.position.z = 0.0;
      goal_pose.pose.orientation.w = 1.0;

      //now i have to publish the postion to nav 2
      if (this->nav_client_->action_server_is_ready()) {
        send_goal_to_nav2(goal_pose);
        //goal_sent_ = true; 
      } else {
        RCLCPP_WARN(this->get_logger(), "Nav2 not ready yet, retrying...");
      }
      
      //for now i only print in terminal do i can grep it and check      
      RCLCPP_INFO(this->get_logger(), ">>> FINAL GOAL: [x: %.2f, y: %.2f] <<<", mid_x, mid_y);
      
    }
  } 
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Cervellone>());
    rclcpp::shutdown();
    return 0;
}
