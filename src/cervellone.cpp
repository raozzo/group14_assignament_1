//TODO: 
//  2. al momento la posizone degli april tag è calcolata su "map" nella consegna deve essere riportata su odom
//  3. errore sincronizzazione camera
//  4. stampare posizini tavoli riferite a odom 
//  5. Dividere cervellone in diversi nodi

//DONE:
//  1. correggere logica di nav to goal
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


//utils services and messages
#include "group14_assignment_1/utils.hpp"
#include "group14_interfaces/srv/look_for_tables.hpp"
#include "group14_interfaces/msg/table.hpp"

#include "std_msgs/msg/bool.hpp" //This is temporary 

using namespace std::chrono_literals;

class Cervellone : public rclcpp::Node
{
  public:
  using LookForTables = group14_interfaces::srv::LookForTables;
  using TableMsg = group14_interfaces::msg::Table;
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
    
    // subsriction to /corrdor_trigger to manually test navigation stopping and resuming
    this->corridor_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/corridor_trigger", 10,
      std::bind(&Cervellone::corridor_callback, this, std::placeholders::_1));

    table_client_ = this->create_client<LookForTables>("look_for_tables");

    RCLCPP_INFO(this->get_logger(), "Cervellone pensa. aspettando le tags...");
  
  }
  
  private:
  //initialize robot position to do 2dPose
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
   
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr corridor_sub_;

  GoalHandleNav::SharedPtr current_goal_handle_;
  // Store the target pose so we can re-send it later
  geometry_msgs::msg::PoseStamped active_target_pose_;
  // Flag to know if we are currently paused
  bool is_navigation_paused_ = false;
  
  
  //function to transform from map to odom
  void log_pose_in_odom(geometry_msgs::msg::PoseStamped input_pose, std::string label)
  {
    geometry_msgs::msg::PoseStamped output_pose;

    try 
    {
      // Transform the pose to odom
      tf_buffer_->transform(input_pose, output_pose, "odom", tf2::durationFromSec(1.0));

      RCLCPP_INFO(this->get_logger(), "REPORT: %s relative to ODOM: [x: %.2f, y: %.2f, z: %.2f]", 
        label.c_str(), 
        output_pose.pose.position.x, 
        output_pose.pose.position.y,
        output_pose.pose.position.z);

    } catch (const tf2::TransformException & ex) 
    {
       RCLCPP_WARN(this->get_logger(), "Cannot transform %s to odom: %s", label.c_str(), ex.what());
    }
  }


  rclcpp::Client<LookForTables>::SharedPtr table_client_;
  
  // Request to table service 
  void request_table_detection()
  {
    if (!table_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Table Detector Service not available!");
        return;
    }

    auto request = std::make_shared<LookForTables::Request>();
        
    RCLCPP_INFO(this->get_logger(), "Requesting Table Detection");

    // Send request asynchronously
    auto future_result = table_client_->async_send_request(request,
        std::bind(&Cervellone::process_tables_response, this, std::placeholders::_1));
  }

    //Handle response 
  void process_tables_response(rclcpp::Client<LookForTables>::SharedFuture future)
  {
    auto result = future.get();

    if (result->tables.empty()) {
      RCLCPP_WARN(this->get_logger(), "Service returned NO tables.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Received %zu tables. Transforming to ODOM...", result->tables.size());

    // Iterate through detected tables
    for (size_t i = 0; i < result->tables.size(); ++i) 
    {
        const auto& table = result->tables[i];
        geometry_msgs::msg::PoseStamped table_pose;
    
  
        table_pose.header.frame_id = "map"; 
        table_pose.header.stamp = this->get_clock()->now();

        table_pose.pose.position = table.center.point; 
        table_pose.pose.orientation.w = 1.0;     // To reuse the same function used for tags i assign a default orientation 

  
        std::string label = "Table " + std::to_string(i + 1);
        log_pose_in_odom(table_pose, label);
    }
  }
   
  void stop_navigation()
  {
    if (!this->current_goal_handle_) {
        RCLCPP_WARN(this->get_logger(), "Cannot stop: No active goal!");
        return;
    }

    RCLCPP_WARN(this->get_logger(), "Stopping Robot...");

    // Cancel the goal
    this->nav_client_->async_cancel_goal(this->current_goal_handle_);
    
    // Mark as paused so we know we intend to resume later
    this->is_navigation_paused_ = true;
    
  }


  void resume_navigation()
  {
    if (!this->is_navigation_paused_) {
      //if not paudes do nothing 
      return;
    }

    RCLCPP_INFO(this->get_logger(), "End of corridor resuming navigation to original target...");

    // send again the saved pose
    send_goal_to_nav2(this->active_target_pose_);
    
    this->is_navigation_paused_ = false;
  }
  
  void corridor_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            // TRUE = Corridor Ended -> STOP
            RCLCPP_WARN(this->get_logger(), "Manual Trigger: STOPPING for Corridor!");
            this->stop_navigation();
        } else {
            // FALSE = Corridor ended -> RESUME
            RCLCPP_INFO(this->get_logger(), "Manual Trigger: RESUMING Navigation!");
            this->resume_navigation();
        }
    }

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

  //DONE: Now tha that we have the 2d pose we can find the tags
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
    
    //Save the goal in case of stop/resume 
    this->active_target_pose_ = goal_pose;

    // Callback when goal is accepted/rejected
    send_goal_options.goal_response_callback = [this](const GoalHandleNav::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "GOAL REJECTKD");
      } else {
        goal_sent_ = true;

        // save the goal handle and bool update 
        this->current_goal_handle_ = goal_handle; 
        this->is_navigation_paused_ = false; // Reset pause flag

        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    };

    //NOTE:
    // Callback when navigation is finished
    send_goal_options.result_callback = [this](const GoalHandleNav::WrappedResult & result)
    {
     switch (result.code) 
     {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Navigation SUCCEEDED!");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Navigation was ABORTED");
          //when the navigation is finsished i want to wait a little and the report the tables found and the tag postion in odom
          //table
          this->request_table_detection();
          //tags
          //log_pose_in_odom(t1, "TAG1");
          //log_pose_in_odom(t2, "TAG2");

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
    
    bool t1_found = false;
    bool t2_found = false;

    geometry_msgs::msg::TransformStamped t1, t2;
    
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

      geometry_msgs::msg::PoseStamped t1_pose;
      geometry_msgs::msg::PoseStamped t2_pose;
      t1_pose.header = t1.header;
      t1_pose.pose.position.x = t1.transform.translation.x;
      t1_pose.pose.position.y = t1.transform.translation.y;
      t1_pose.pose.position.z = t1.transform.translation.z;
      t1_pose.pose.orientation = t1.transform.rotation;
      log_pose_in_odom(t1_pose, "TAG1");
      t2_pose.header = t2.header;
      t2_pose.pose.position.x = t2.transform.translation.x;
      t2_pose.pose.position.y = t2.transform.translation.y;
      t2_pose.pose.position.z = t2.transform.translation.z;
      t2_pose.pose.orientation = t2.transform.rotation;
      log_pose_in_odom(t2_pose, "TAG2");
      
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
