from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='bb_controller',
            executable='wheel_odometry',
            name='wheel_odometry',
            output='screen',
            parameters=[{
                'publish_tf': False,
                'use_sim_time': True,
                'odom_frame': 'odom',
                'base_frame': 'base_footprint'
            }]
        ),
    ])
