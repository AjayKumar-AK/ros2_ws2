#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Path to your URDF file
    urdf_file_path = '/home/ajay/ros2_ws/src/my_tutorial_1/urdf/myrobo1.urdf'
    
    # Read the URDF file
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()
    
    # Path to RViz config file (create this path in your package)
    package_name = 'my_tutorial_1'
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'robot_view.rviz'
    )
    
    # If the above path doesn't exist, use a default config
    if not os.path.exists(rviz_config_file):
        # Create a temporary config file path - RViz will use default if not found
        rviz_config_file = ''
        
    ekf_config = os.path.join(
        get_package_share_directory('my_tutorial_1'),
        'config',
        'ekf.yaml'
    )
    return LaunchDescription([
        
        # Robot State Publisher - publishes robot transforms from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': False
            }]
        ),
        
        # Joint State Publisher - publishes joint states (optional, for wheel visualization)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
       
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['/home/ajay/ros2_ws/src/my_tutorial_1/config/ekf.yaml']
            
        )
    ])

'''
# RViz Node with custom config
        Node(
           package='rviz2',
           executable='rviz2',
           name='rviz2',
           output='screen',
           arguments=['-d', rviz_config_file] if rviz_config_file else [],
           parameters=[{
               'use_sim_time': False
           }],
           #Set initial RViz configuration via parameters
           remappings=[]
        ) ,
'''


