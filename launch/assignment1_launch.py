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
    #Aproltag launch arguments 
    apriltag_group = GroupAction(
        actions=[
            # Remapping of camera nodes  
            # Since there is no distrotion we can use directrly the image 
            SetRemap(src='image_rect', dst='/rgb_camera/image'),
            SetRemap(src='camera_info', dst='/rgb_camera/camera_info'),
            
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

    #----------- Nodes-----------------

    #Cervellone 
    task_manager = Node(
        package='group14_assignment_1',
        executable='task_manager',  
        output='screen',
        parameters = [{'use_sim_time': True}]
    )

    corridor_detector = Node(
        package='group14_assignment_1',
        executable='corridor_detector',  
        output='screen'
    )
    
    apriltags_detection = Node(
        package='group14_assignment_1',
        executable='apriltags_detection',  
        output='screen',
        parameters = [{'use_sim_time': True}]
    )

    corridor_navigator = Node(
        package='group14_assignment_1',
        executable='corridor_navigator',  
        output='screen'
    )

    cylinders_finder = Node(
        package='group14_assignment_1',
        executable='cylinders_finder',  
        output='screen'
    )

    laser_scan_clustering = Node( 
        package='group14_assignment_1',
        executable='laser_scan_clustering',  
        output='screen'
    )
    
    #Launch
    return LaunchDescription([
        include_assignment_1_launch,
        apriltag_group,
        task_manager,
        apriltags_detection,
        laser_scan_clustering,
        corridor_detector,
        corridor_navigator,
        cylinders_finder
    ])


