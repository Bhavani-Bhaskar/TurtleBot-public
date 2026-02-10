import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():

    # Run the simple_controller node
    # This node will use the controllers already spawned by controller.launch.py
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

    # Add a small delay to ensure controllers are fully initialized
    delayed_simple_controller = TimerAction(
        period=2.0,
        actions=[simple_controller_node]
    )

    return LaunchDescription(
        [
            delayed_simple_controller,
        ]
    )