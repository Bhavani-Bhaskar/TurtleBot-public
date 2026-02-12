from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

# FIXES in this file:
# 1. Odometry+TF bridge: /model/bumperbot/tf → /tf  (makes odom frame visible)
# 2. Extra static TF: lidar_link → bumperbot/base_footprint/laser
#    Gazebo stamps LaserScan with a namespaced frame_id; this bridges it
#    into the ROS TF tree so RViz can place laser rays correctly.


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    bb_pkg = FindPackageShare('bb_description')
    gz_pkg = FindPackageShare('ros_gz_sim')

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([bb_pkg, 'urdf', 'bb.urdf.xacro'])
        ]),
        value_type=str
    )

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([gz_pkg, 'launch', 'gz_sim.launch.py'])
            ),
            launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
        ),

        # Wheel Odometry Node (publish_tf=False — TF owned by Gazebo bridge)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('bb_controller'), 'launch', 'odometry.launch.py'
                ])
            )
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }]
        ),

        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-name', 'bumperbot', '-topic', '/robot_description']
        ),

        # Clock bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
        ),

        # Joint states bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/world/empty/model/bumperbot/joint_state'
                '@sensor_msgs/msg/JointState[gz.msgs.Model'
            ],
            remappings=[('/world/empty/model/bumperbot/joint_state', '/joint_states')]
        ),

        # Odometry + TF bridge
        # /model/bumperbot/tf carries odom→base_footprint from Gazebo OdometryPublisher
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/model/bumperbot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/bumperbot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            remappings=[
                ('/model/bumperbot/odometry', '/odom'),
                ('/model/bumperbot/tf', '/tf'),
            ]
        ),

        # Laser scan bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan']
        ),

        # Static TF: base_link → lidar_link  (URDF chain)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_link']
        ),

        # FIX: Gazebo stamps /scan with frame_id 'bumperbot/base_footprint/laser'
        # Connect that namespaced frame into the ROS TF tree.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_gz_frame_fix',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0',
                'lidar_link',
                'bumperbot/base_footprint/laser'
            ]
        ),

    ])