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
    
    final_approach_arg = DeclareLaunchArgument(
        'final_approach',
        default_value='false',
        description='Flag to enable/disable final approach behavior'
    )

    # Node for pre_approachv2
    pre_approach = Node(
        package='attach_shelf',
        executable='pre_approach_v2',
        name='pre_approach_v2',
        parameters=[{
            'obstacle': LaunchConfiguration('obstacle'),
            'degrees': LaunchConfiguration('degrees'),
            'final_approach': LaunchConfiguration('final_approach'),
        }],
        output='screen'
    )
    
    # Node for approach_service_server
    approach_service_server = Node(
        package='attach_shelf',
        executable='approach_service_server',
        name='approach_service_server',
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

    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        final_approach_arg,
        pre_approach,
        approach_service_server,
        rviz_launch
    ])

