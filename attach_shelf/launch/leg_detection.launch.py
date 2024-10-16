import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the launch arguments
    obstacle_arg = DeclareLaunchArgument(
        'obstacle',
        default_value='0.5',
        description='Obstacle distance parameter'
    )
    
    degrees_arg = DeclareLaunchArgument(
        'degrees',
        default_value='90',
        description='Turning degrees parameter'
    )
    

    # Node for pre_approachv2
    pre_approach = Node(
        package='attach_shelf',  # Replace with your package name
        executable='leg_detection',  # The name of your node executable
        name='leg_detection',
        output='screen'
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
        pre_approach,
        rviz_launch
    ])
