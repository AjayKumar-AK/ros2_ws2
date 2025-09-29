#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = '/home/ajay/ros2_ws/src/my_tutorial_1/urdf/gazebo.urdf'
    slam_params = '/home/ajay/ros2_ws/src/my_tutorial_1/config/slam_params.yaml'
    rviz_config = '/home/ajay/ros2_ws/src/my_tutorial_1/rviz/master.rviz'

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([

        # Start Gazebo (gz sim)
        ExecuteProcess(
            cmd=['gz', 'sim', '-v4', 'empty.sdf'],
            output='screen'
        ),

        # Spawn robot into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-string', robot_desc, '-name', 'mark3'],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc,
                         'use_sim_time': True}]
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            output='screen',
            parameters=[slam_params, {'use_sim_time': True}]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])

