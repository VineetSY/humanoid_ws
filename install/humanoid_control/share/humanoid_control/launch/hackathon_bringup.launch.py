import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
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
            name='main_processing_unit',
            output='screen'
        ),
        Node(
            package='humanoid_control',
            executable='motion_generator',
            name='motor_control_unit',
            output='screen'
        ),
        Node(
            package='humanoid_control',
            executable='perception_sim',
            name='perception_unit',
            output='screen'
        ),
        Node(
            package='humanoid_control',
            executable='object_spawner',
            name='object_simulation',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
