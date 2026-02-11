from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    bb_pkg = FindPackageShare('bb_description')
    gz_pkg = FindPackageShare('ros_gz_sim')

    # Robot description (XACRO → STRING)
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                bb_pkg,
                'urdf',
                'bb.urdf.xacro'
            ])
        ]),
        value_type=str
    )

    return LaunchDescription([

        # -----------------------------
        # Launch arguments
        # -----------------------------
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        # -----------------------------
        # Launch Gazebo Harmonic
        # -----------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    gz_pkg,
                    'launch',
                    'gz_sim.launch.py'
                ])
            ),
            launch_arguments={
                'gz_args': '-r -v 4 empty.sdf'
            }.items()
        ),

        # -----------------------------
        # Robot State Publisher
        # -----------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }]
        ),

        # -----------------------------
        # Spawn robot in Gazebo
        # -----------------------------
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name', 'bumperbot',
                '-topic', '/robot_description'
            ]
        ),

        # -----------------------------
        # Clock bridge (REQUIRED)
        # -----------------------------
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            ]
        ),

        # -----------------------------
        # Joint states bridge
        # -----------------------------
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            arguments=[
                '/world/empty/model/bumperbot/joint_state'
                '@sensor_msgs/msg/JointState[gz.msgs.Model'
            ],
            remappings=[
                ('/world/empty/model/bumperbot/joint_state', '/joint_states')
            ]
        ),

        # -----------------------------
        # Odometry bridge
        # -----------------------------
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            arguments=[
                '/model/bumperbot/odometry'
                '@nav_msgs/msg/Odometry[gz.msgs.Odometry'
            ],
            remappings=[
                ('/model/bumperbot/odometry', '/odom')
            ]
        ),

        # -----------------------------
        # Laser scan bridge
        # -----------------------------
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
            ]
        ),

        # -----------------------------
        # Static TF (ONLY if needed)
        # -----------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                'base_link',
                'lidar_link'
            ]
        ),
    ])
