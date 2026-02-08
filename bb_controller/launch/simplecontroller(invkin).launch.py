import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Spawn simple_velocity_controller
    simple_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    # Run the simple_controller node
    simple_controller_node = Node(
        package="bb_controller",
        executable="simple_controller",
        name="simple_controller",
        output="screen",
        parameters=[
            {"wheel_radius": 0.033},
            {"wheel_separation": 0.14022}
        ]
    )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            simple_velocity_controller_spawner,
            simple_controller_node,
        ]
    )