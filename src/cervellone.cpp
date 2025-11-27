//CPP LIBRARIES 
#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <thread>

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
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp" 

//utils services and messages
#include "group14_interfaces/srv/look_for_tables.hpp"
#include "group14_interfaces/msg/table.hpp"

#include "std_msgs/msg/bool.hpp" //This is temporary 

using namespace std::chrono_literals;

class Cervellone : public rclcpp::Node
{
  public:
  using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;
  using LookForTables = group14_interfaces::srv::LookForTables;
  using TableMsg = group14_interfaces::msg::Table;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  
  //INFO:------------------- START CONSTRUCTOR-------------------------------
  Cervellone(): Node("cervellone")
  {
    //info that i'm starting the lifecicle manager client 
    RCLCPP_INFO(this->get_logger(), "Nav2 lifecylcle managar client");
   
    //Client for localization and navigation stack 
    client_localization_ = this->create_client<ManageLifecycleNodes>(
        "/lifecycle_manager_localization/manage_nodes"); 
    client_navigation_ = this->create_client<ManageLifecycleNodes>(
        "/lifecycle_manager_navigation/manage_nodes"); 
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    
    //TF listener initialization 
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());    
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //Publisher settings 
    init_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10);
    
    // start a thread to wait for apriltag //INFO: this is to eliminate the need of hard startup timer 
    startup_thread_ = std::thread(&Cervellone::wait_for_services_and_startup, this);

    //when corridor is detected
    RCLCPP_INFO(this->get_logger(), "Subscribing to /corridor_trigger topic.");
    this->corridor_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/corridor_trigger",
            2,
            std::bind(&Cervellone::corridor_callback, this, std::placeholders::_1));
     
    //Subscrbing to the apriltag publisher (in this topic we will receive the goal coordiantes)
    goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_in_map_frame",
            10,
            std::bind(&Cervellone::goal_callback_, this, std::placeholders::_1));
   

    // After 3s, start trying to send the goal to nav 2 
    timer_ = this->create_wall_timer(
            3.0s,
            [this](){
              if (navigation_ready_)
              {
                send_goal_to_nav2();
              }
            }
    );
    
     
    RCLCPP_INFO(this->get_logger(),"Client initialization for table detections ");
    table_client_ = this->create_client<LookForTables>("look_for_tables");

    RCLCPP_INFO(this->get_logger(), "Cervellone pensa. aspettando le tags..."); 
  }//INFO:END OF CONTRUCTOR 
  

  //INFO:------------------- START PRIVATE-------------------------------
  private:
  //initialize robot position to do 2dPose
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
   
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr corridor_sub_;

  GoalHandleNav::SharedPtr current_goal_handle_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  // Store the target pose so we can re-send it later
  geometry_msgs::msg::PoseStamped most_recent_goal_pose_;
  // Flag to know if we are currently paused
  bool is_navigation_paused_ = false;
  
  rclcpp::Client<ManageLifecycleNodes>::SharedPtr client_localization_;
  rclcpp::Client<ManageLifecycleNodes>::SharedPtr client_navigation_;
  
  rclcpp::Client<LookForTables>::SharedPtr table_client_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


  bool navigation_ready_ = false; 
  std::string TAG1_FRAME_ID = "tag36h11:1"; 
  std::string TAG2_FRAME_ID = "tag36h11:10";
  const int TAG1_ID = 1;
  const int TAG2_ID = 10;

  std::string MAP_FRAME_ID = "map";
  std::string ODOM_FRAME_ID = "odom";


  //INFO: --------STARTUP WAITING---------
  std::thread startup_thread_;

  void wait_for_services_and_startup()
  {
    RCLCPP_INFO(this->get_logger(), "STARTUP THREAD: Waiting for lifecycle managers to be available...");

    // Lifecycle manager localization
    while (!client_localization_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(),
          "Interrupted while waiting for localization service. Exiting.");
        return;
      }
     
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Waiting for localization manager...");
    }

    // Lifecycle manager localization
    while (!client_navigation_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(),
          "Interrupted while waiting for navigation service. Exiting.");
        return;
      }
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Waiting for Navigation Manager...");
    }

    RCLCPP_INFO(this->get_logger(), "Lifecycles managers are ready!");
    //since now are ready we start the full stack  thanks to the helper function 
    startup_full_stack();
  }

  
  void startup_full_stack()
  {
    auto request = std::make_shared<ManageLifecycleNodes::Request>();
    request->command = ManageLifecycleNodes::Request::STARTUP; 

    RCLCPP_INFO(this->get_logger(), "Requesting LOCALIZATION Startup...");

    client_localization_->async_send_request(request, [this](rclcpp::Client<ManageLifecycleNodes>::SharedFuture future_loc) {
      if (future_loc.get()->success) 
      {
        RCLCPP_INFO(this->get_logger(), "Localization Active");
        RCLCPP_INFO(this->get_logger(), "Publishing Initial pose");
        this-> set_initial_pose();

        RCLCPP_INFO(this->get_logger(), "starting navigation stack");
        this->startup_navigation();
      }
      else 
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to start Localization.");
      }
    });
  }

  // Function to set the inital pose
  void set_initial_pose()
  {
    auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = MAP_FRAME_ID;

    // Set the known spawn coordinates from the tutor's file
    msg.pose.pose.position.x = 0; // -8.29;
    msg.pose.pose.position.y = 0; // 1.87;
    msg.pose.pose.position.z = 0; // elsee0.01;

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

  void startup_navigation()
  {
    auto request = std::make_shared<ManageLifecycleNodes::Request>();
    request->command = ManageLifecycleNodes::Request::STARTUP; 

    RCLCPP_INFO(this->get_logger(), "Requesting NAVIGATION Startup...");

    client_navigation_->async_send_request(request, [this](rclcpp::Client<ManageLifecycleNodes>::SharedFuture future_nav) {
      if (future_nav.get()->success) 
      {
        RCLCPP_INFO(this->get_logger(), "Navigation Active.");
        navigation_ready_ = true;        
      } 
      else 
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to start Navigation.");
      }
    });
  }
  
  //function to transform from map to odom
  void log_pose_in_odom(geometry_msgs::msg::PoseStamped input_pose, std::string label)
  {
    geometry_msgs::msg::PoseStamped output_pose;

    try 
    {
      // Transform the pose to odom
      tf_buffer_->transform(input_pose, output_pose, ODOM_FRAME_ID, tf2::durationFromSec(1.0));

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
    
        table_pose.header.frame_id = MAP_FRAME_ID; 
        table_pose.header.stamp = builtin_interfaces::msg::Time();

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

    // Stop reading new goals from the goal_in_map_frame topic
    timer_->cancel();

    // Cancel the goal
    this->nav_client_->async_cancel_goal(this->current_goal_handle_, [this](const auto & cancel_response) { // Callback function
      if (cancel_response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) 
      {
        RCLCPP_INFO(this->get_logger(), "STOP CONFIMED: Goal successfully canceled");
        //it's actually cancelled so i can pause 
        this->is_navigation_paused_ = true;          
      } else {
        RCLCPP_ERROR(this->get_logger(), "STOP FAILED: Cancellation rejecte");
        //HACK:retry
        stop_navigation();
      }
    });
  }

  void resume_navigation()
  {
    if (!this->is_navigation_paused_) {
      //if not paudes do nothing 
      return;
    }

    RCLCPP_INFO(this->get_logger(), "End of corridor resuming navigation to original target...");

    // send again the most recent goal from the goal_in_map_frame topic
    send_goal_to_nav2();
    timer_->reset();  // reload the periodic goal update

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
 
  // start navigation to goal 
  void send_goal_to_nav2()
  {
    if (!this->nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "NAV ACTION NOT READY");
      return;
    }
    
    auto goal_msg = NavigateToPose::Goal();
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    goal_msg.pose = most_recent_goal_pose_; 

    // Callback when goal is accepted/rejected
    send_goal_options.goal_response_callback = [this, goal_msg](const GoalHandleNav::SharedPtr &goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "GOAL REJECTED");
      } else {
        // save the goal handle and bool update 
        this->current_goal_handle_ = goal_handle; 
        this->is_navigation_paused_ = false; // Reset pause flag
        RCLCPP_INFO(this->get_logger(), "Goal [odom: %lf, %lf; time %d] accepted by server, waiting for result",
            goal_msg.pose.pose.position.x,
            goal_msg.pose.pose.position.y,
            goal_msg.pose.header.stamp.sec);
      }
    };

    //NOTE:
    // Callback when navigation is finished
    send_goal_options.result_callback = [this, goal_msg](const GoalHandleNav::WrappedResult & result)
    {
      switch (result.code)
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Result of Goal [odom: %lf, %lf; time %d]: Navigation SUCCEEDED!",
              goal_msg.pose.pose.position.x,
              goal_msg.pose.pose.position.y,
              goal_msg.pose.header.stamp.sec);
          timer_->cancel(); // Stop setting new goals
          this->request_table_detection();
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_INFO(this->get_logger(), "Result of Goal [odom: %lf, %lf; time %d]: Navigation ABORTED!",
              goal_msg.pose.pose.position.x,
              goal_msg.pose.pose.position.y,
              goal_msg.pose.header.stamp.sec);
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_INFO(this->get_logger(), "Result of Goal [odom: %lf, %lf; time %d]: Navigation CANCELED!",
              goal_msg.pose.pose.position.x,
              goal_msg.pose.pose.position.y,
              goal_msg.pose.header.stamp.sec);
          break;
        default:
          RCLCPP_INFO(this->get_logger(), "Result of Goal [odom: %lf, %lf; time %d]: Unknown result!",
              goal_msg.pose.pose.position.x,
              goal_msg.pose.pose.position.y,
              goal_msg.pose.header.stamp.sec);
          break;
      }
    };

    // Send the goal
    this->nav_client_->async_send_goal(goal_msg, send_goal_options);
  };
  
  /* REACTION TO GOALS*/
  void goal_callback_(const geometry_msgs::msg::PoseStamped &goal)
  {
    // RCLCPP_INFO(this->get_logger(), "Goal received.");
    most_recent_goal_pose_ = goal;
  }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Cervellone>());
    rclcpp::shutdown();
    return 0;
}
