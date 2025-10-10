#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Path to your URDF file
    urdf_file_path = '/home/ajay/ros2_ws/src/my_tutorial_1/urdf/myrobo1.urdf'
    
    # Read the URDF file
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()
    
    # Path to SLAM params file
    slam_params_file = '/home/ajay/ros2_ws/src/my_tutorial_1/config/slam_params.yaml'
    
    # Path to RViz config file
    rviz_config_file = '/home/ajay/ros2_ws/src/my_tutorial_1/rviz/master.rviz'
    
    # EKF config (commented out for now)
    # ekf_config = os.path.join(
    #     get_package_share_directory('my_tutorial_1'),
    #     'config',
    #     'ekf.yaml'
    # )
    
    return LaunchDescription([
        
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        
        # Robot State Publisher - publishes robot transforms from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # Joint State Publisher - publishes joint states
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # Static Transform Publisher - base_link to laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_tf',
            arguments=['0.0', '-0.0455', '0.0475', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
        
        
        
    
        
       
        
         
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['/home/ajay/ros2_ws/src/my_tutorial_1/config/ekf.yaml']
            
        )
    ])
