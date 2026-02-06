import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_bb = get_package_share_directory("bb_description")

    # Paths
    gazebo_launch = os.path.join(pkg_bb, "launch", "gazebo.launch.py")
    rviz_config = os.path.join(pkg_bb, "rviz", "display.rviz")

    # 1️⃣ Headless Gazebo (NO GUI)
    gazebo_headless = ExecuteProcess(
        cmd=[
            "gz", "sim",
            "-r",
            "empty.sdf",
            "--headless-rendering"
        ],
        output="screen"
    )

    # 2️⃣ Robot + spawn
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch)
    )

    # 3️⃣ RViz with config
    rviz = ExecuteProcess(
        cmd=[
            "rviz2",
            "-d",
            rviz_config
        ],
        output="screen"
    )

    joint_state_publisher = Node(
    package="joint_state_publisher",
    executable="joint_state_publisher",
    name="joint_state_publisher"
    )

    robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher"
    )

    return LaunchDescription([
        gazebo_headless,
        robot_launch,
        joint_state_publisher,
        rviz
    ])
