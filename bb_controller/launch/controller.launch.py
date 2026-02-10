import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["simple_velocity_controller",
                "--controller-manager",
                "/controller_manager"
        ]
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        remappings=[
            ('/imu_sensor_broadcaster/imu', '/imu/data'),
        ],
    )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            simple_controller,
            imu_sensor_broadcaster_spawner,
        ]
    )