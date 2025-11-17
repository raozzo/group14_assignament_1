import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Launch for tutor's launchfile 
    ir_launch_package_share = get_package_share_directory('ir_launch')

    assignment_1_launch_file = os.path.join(
        ir_launch_package_share,
        'launch',
        'assignment_1.launch.py'
    )

    include_assignment_1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(assignment_1_launch_file)
    )
    
    # Launch for apriltag 
    apriltag_package_share = get_package_share_directory('apriltag_ros')

    apriltag_launch_file = os.path.join(
        apriltag_package_share,
        'launch',
        'camera_36h11.launch.yml'
    )

    include_apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(apriltag_launch_file)
    )

    #this needs to be changed
    #Launch for the server
    turtlenode_server = Node(
        package='group14_ex4',
        executable='turtlenode_server',  
        output='screen'  # Shows print/log statements in the terminal
    )

    #Launch for the clinet
    burrow_client = Node(
        package='group14_ex4',
        executable='burrow_client',
        output='screen'
    )

    # --- 3. Return the LaunchDescription ---
    return LaunchDescription([
        include_assignment_1_launch,
        include_apriltag_launch,
        #turtlenode_server,
        #burrow_client
    ])


