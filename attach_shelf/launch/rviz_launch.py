from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Set the path to your RViz configuration file
    config_rviz = os.path.join(
        os.getenv('HOME'),
        'ros2_ws/src/attach_shelf/config/config.rviz'
    )

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', config_rviz]  # Pass the RViz config file
        )
    ])
