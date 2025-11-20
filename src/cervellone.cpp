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

using namespace std::chrono_literals;

class Cervellone : public rclcpp::transform_stamped
{
  public:
    // constructor

  private:

    //read apriltag 
    
    //calculate the median point between apriltag 

    // send goal to nav

} 


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AssignmentManager>());
    rclcpp::shutdown();
    return 0;
}
