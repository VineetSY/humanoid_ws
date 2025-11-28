import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('humanoid_control')
    xacro_file = os.path.join(pkg_path, 'urdf', 'humanoid_32dof.urdf.xacro')
    
    robot_desc = Command(['xacro ', xacro_file])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='humanoid_control',
            executable='system_supervisor',
            name='system_supervisor',
            output='screen'
        ),
        Node(
            package='humanoid_control',
            executable='motion_generator',
            name='motion_generator',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
