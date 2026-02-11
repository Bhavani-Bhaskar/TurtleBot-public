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

    # IMU Sensor Broadcaster with corrected remapping
    # The broadcaster publishes to /<controller_name>/imu by default
    # We remap /imu_sensor_broadcaster/imu -> /imu/data
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        remappings=[
            # Remap from default topic to desired topic
            # Format: ('from_topic', 'to_topic')
            ('imu_sensor_broadcaster/imu', 'imu/data'),
        ],
    )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            simple_controller,
            imu_sensor_broadcaster_spawner,
        ]
    )