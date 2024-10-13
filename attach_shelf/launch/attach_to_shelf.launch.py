import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    obstacle_arg = DeclareLaunchArgument(
        'obstacle',
        default_value='0.0',
        description='Obstacle distance parameter'
    )

    degrees_arg = DeclareLaunchArgument(
        'degrees',
        default_value='0.0',
        description='Degrees to turn parameter'
    )

    # Get the launch configurations
    obstacle = LaunchConfiguration('obstacle')
    degrees = LaunchConfiguration('degrees')

    # Node to launch
    pre_approach_node = Node(
        package='attach_shelf',
        executable='pre_approach_v2',
        name='pre_approach_v2',
        output='screen',
        parameters=[{
            'obstacle': obstacle,
            'degrees': degrees
        }]
    )

    # Include rviz_launch.py
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('attach_shelf'),
                'launch',
                'rviz_launch.py'
            )
        )
    )

    # Return the LaunchDescription
    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        pre_approach_node,
        rviz_launch
    ])
