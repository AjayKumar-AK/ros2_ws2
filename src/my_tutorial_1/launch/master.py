import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    

    # Robot model path
    urdf_file_path = '/home/ajay/ros2_ws/src/my_tutorial_1/urdf/myrobo1.urdf'
    
    # Read the URDF file
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()
        
    model_arg = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'config', 'mapper_params_online_async.yaml'])
    )

    # RViz config
    package_name = 'my_tutorial_1'
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'robot_view.rviz'
    )

    if not os.path.exists(rviz_config_file):
        rviz_config_file = ''  # fallback → RViz default

    return LaunchDescription([
    

        # Declare model argument
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([
            FindPackageShare('slam_toolbox'),
            'config',
            'mapper_params_online_async.yaml'
    ]),
    description='Full path to the ROS2 parameters file to use for SLAM Toolbox'
),
        DeclareLaunchArgument(
            'model',
            default_value=PathJoinSubstitution([FindPackageShare('my_tutorial_1'), 'urdf', 'myrobo1.urdf']),
            description='URDF/XACRO file path'
        ),

        # Use sim time argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # RPLIDAR driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py'])
            ])
        ),

        # EKF localization (fuses IMU + encoder)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['/home/ajay/ros2_ws/src/my_tutorial_1/config/ekf.yaml']
        ),

        # Robot State Publisher
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

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file] if rviz_config_file else [],
            parameters=[{'use_sim_time': False}]
        ),

        

        # SLAM Toolbox (online mapping mode)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file, {'use_sim_time': False}],
            remappings=[('/scan', '/scan')]  # already correct
        )
    ])

