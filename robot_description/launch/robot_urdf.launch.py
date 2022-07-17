import os
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node 
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    package_name = 'robot_urdf'
    path_to_urdf = get_package_share_path('robot_urdf') / 'robot.xacro'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': ParameterValue(Command(['xacro ',str(path_to_urdf)]),value_type=str)}],
            ),
        Node(
            package= package_name,
            executable='state_publisher',
            name='state_publisher',
            output='screen'),
    ])

