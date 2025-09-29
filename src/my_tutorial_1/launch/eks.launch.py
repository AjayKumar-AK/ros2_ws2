from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_se_odom',
            output='screen',
            parameters=['/path/to/your/config/ekf.yaml']
        )
    ])

