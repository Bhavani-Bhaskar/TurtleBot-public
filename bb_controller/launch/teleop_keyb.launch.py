import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():

    # Run the teleop_keyb node
    # This node will use the controllers already spawned by controller.launch.py
    teleop_keyb_node = Node(
        package="bb_controller",
        executable="teleop_keyb",
        name="teleop_keyb_controller",
        output="screen",
        parameters=[
            {"wheel_radius": 0.033},
            {"wheel_separation": 0.14022}
        ]
    )

    # Add a small delay to ensure controllers are fully initialized
    delayed_teleop_keyb = TimerAction(
        period=2.0,
        actions=[teleop_keyb_node]
    )

    return LaunchDescription(
        [
            delayed_teleop_keyb,
        ]
    )