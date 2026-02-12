from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config = PathJoinSubstitution([
        FindPackageShare('bb_description'),
        'rviz',
        'display.rviz'
    ])

    return LaunchDescription([

        # ------------------------------------------------
        # Use simulation time (matches Gazebo)
        # ------------------------------------------------
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock from Gazebo'
        ),

        # ------------------------------------------------
        # RViz2 - VISUALIZATION ONLY
        # ------------------------------------------------
        # NOTE: robot_state_publisher is already running in gazebo.launch.py
        # NOTE: All bridges (/clock, /joint_states, /scan, etc.) are in gazebo.launch.py
        # This file ONLY launches RViz for visualization
        # ------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),
    ])