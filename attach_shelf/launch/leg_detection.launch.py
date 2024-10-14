import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Node for detecting legs
    leg_detection_node = Node(
        package='attach_shelf',
        executable='leg_detection',
        name='leg_detection_node',
        output='screen',
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
        leg_detection_node,
        rviz_launch
    ])
