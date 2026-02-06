from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('bb_description'),
            'urdf',
            'bb.urdf.xacro'
        ])
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare('bb_description'),
        'rviz',
        'display.rviz'
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }],
            output='screen'
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
