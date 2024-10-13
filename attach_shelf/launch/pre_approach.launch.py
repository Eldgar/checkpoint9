from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='attach_shelf',  # Replace with your package name
            executable='pre_approach',  # The name of your node executable
            name='pre_approach',
            parameters=[
                {'obstacle': 0.5},  # Set obstacle distance parameter
                {'degrees': 90},    # Set turning degrees parameter
            ],
            output='screen'
        )
    ])
