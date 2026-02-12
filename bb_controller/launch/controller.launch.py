import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():

    # ------------------------------------------------
    # IMPORTANT: Launch this AFTER gazebo.launch.py
    # ------------------------------------------------
    # The controller_manager is already running (started by gz_ros2_control plugin)
    # This file only SPAWNS the controllers
    # ------------------------------------------------

    # Joint State Broadcaster
    # Reads joint states from hardware interface and publishes to /joint_states
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Simple Velocity Controller
    # Provides velocity control interface for both wheels
    # Subscribes to: /simple_velocity_controller/commands (Float64MultiArray)
    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    # IMU Sensor Broadcaster
    # Reads IMU data from ros2_control hardware interface
    # Publishes to: /imu/data (sensor_msgs/Imu)
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        remappings=[
            # Remap from default topic to standard topic name
            ('imu_sensor_broadcaster/imu', 'imu/data'),
        ],
    )

    # ------------------------------------------------
    # Add delays to ensure proper startup sequence
    # ------------------------------------------------
    # Small delays help ensure controller_manager is fully ready
    delayed_joint_state_broadcaster = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delayed_simple_controller = TimerAction(
        period=3.0,
        actions=[simple_controller]
    )

    delayed_imu_broadcaster = TimerAction(
        period=4.0,
        actions=[imu_sensor_broadcaster_spawner]
    )

    return LaunchDescription([
        delayed_joint_state_broadcaster,
        delayed_simple_controller,
        delayed_imu_broadcaster,
    ])

    # ------------------------------------------------
    # FLOW SUMMARY:
    # ------------------------------------------------
    # Terminal 1: ros2 launch bb_description gazebo.launch.py
    #            (Starts Gazebo, controller_manager, bridges)
    # Terminal 2: ros2 launch bb_controller controller.launch.py
    #            (Spawns controllers - THIS FILE)
    # Terminal 3: ros2 launch bb_controller simplecontroller(invkin).launch.py
    #            OR ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # Terminal 4 (optional): ros2 launch bb_description rviz.launch.py
    #            (Visualization only)
    # ------------------------------------------------