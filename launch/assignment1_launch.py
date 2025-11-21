import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction, ExecuteProcess
from launch_ros.actions import Node, SetRemap, SetParameter

def generate_launch_description():
    
    # Launch for tutor's launchfile 
    ir_launch_package_share = get_package_share_directory('ir_launch')

    assignment_1_launch_file = os.path.join(
        ir_launch_package_share,
        'launch',
        'assignment_1.launch.py'
    )

    include_assignment_1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(assignment_1_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            #from the tutor launch we can see it exposes ab autostart flag for the nav stack
            'autostart': 'True'
        }.items()
    )
    
    # Launch for apriltag 
    apriltag_package_share = get_package_share_directory('group14_assignment_1')

    apriltag_launch_file = os.path.join(
        apriltag_package_share,
        'apriltag_ros',
        'launch',
        'camera_36h11.launch.yml'
    )


    apriltag_group = GroupAction(
        actions=[
            # Remap the node's internal name ('image_rect') to your sim topic
            # VERIFY: Use 'ros2 topic list' to check if your topic is 'image_raw' or 'image_rect'
            SetRemap(src='image_rect', dst='/rgb_camera/image'),
            SetRemap(src='camera_info', dst='/rgb_camera/camera_info'),
            
            #resize
            #SetParameter(name='size', value='0.05'),
            
            # Include the file required by the assignment
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(apriltag_launch_file),
                # Enable sim time so TF works correctly in Gazebo
                launch_arguments={
                    'use_sim_time': 'true'
                }.items() 
            )
        ]
    )



    #this needs to be changed
    #Launch for the server
    cervellone = Node(
        package='group14_assignment_1',
        executable='cervellone',  
        output='screen'  # Shows print/log statements in the terminal
    )

    cylinders_finder = Node(
        package='group14_assignment_1',
        executable='cylinders_finder',  
        output='screen'
    )

#
    #Launch for the clinet
   # burrow_client = Node(
    #    package='group14_ex4',
     #   executable='burrow_client',
     #   output='screen'
    #)

    # --- 3. Return the LaunchDescription ---
    return LaunchDescription([
        include_assignment_1_launch,
        apriltag_group,
        cervellone,
        cylinders_finder
        #burrow_client
    ])


