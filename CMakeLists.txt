cmake_minimum_required(VERSION 3.8)
project(group14_assignment_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(group14_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)


## Include
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

# cervellone .cpp
add_executable(cervellone src/cervellone.cpp)
ament_target_dependencies(
  cervellone
  nav2_msgs
  rclcpp_action
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  group14_interfaces
)

add_executable(cylinders_finder src/cylinders_finder.cpp)
ament_target_dependencies(
  cylinders_finder
  rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs sensor_msgs visualization_msgs group14_interfaces
)
target_link_libraries(cylinders_finder Eigen3::Eigen)

add_executable(laser_scan_clustering src/laser_scan_clustering.cpp)
ament_target_dependencies(
  laser_scan_clustering
  rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs sensor_msgs visualization_msgs group14_interfaces
)

add_executable(corridor_detector src/corridor_detector.cpp)
ament_target_dependencies(
  corridor_detector
  rclcpp std_msgs tf2 tf2_ros tf2_geometry_msgs geometry_msgs sensor_msgs visualization_msgs group14_interfaces        
)

add_executable(corridor_navigator src/corridor_navigator.cpp)
ament_target_dependencies(
  corridor_navigator
  rclcpp
  std_msgs             
  geometry_msgs        
)

# Install launch files
install(DIRECTORY
  launch
  apriltag_ros
  DESTINATION share/${PROJECT_NAME}
)

#install include
install(DIRECTORY include/
  DESTINATION include
)

#install cervellone
install(TARGETS
  cervellone cylinders_finder laser_scan_clustering corridor_detector corridor_navigator
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
