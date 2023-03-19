import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,  IncludeLaunchDescription
from launch.actions import RegisterEventHandler 
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable 
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node



def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_urdf"),
                    "quadruped_main.urdf.xacro",
                ]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-string', robot_description_content,
                   '-name', 'Quadruped',
                   '-allow_renaming', 'true'],
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],

    )

    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('ign_args', [' -r -v 4 empty.sdf'])]),
        ignition_spawn_entity,
        node_robot_state_publisher,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[bridge],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=bridge,
                on_start=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        ),
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])
