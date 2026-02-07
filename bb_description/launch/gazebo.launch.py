from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    bb_pkg = FindPackageShare('bb_description')
    gz_pkg = FindPackageShare('ros_gz_sim')

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            bb_pkg,
            'urdf',
            'bb.urdf.xacro'
        ])
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        # Launch Gazebo Harmonic
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

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {
                    'robot_description': robot_description,
                    'use_sim_time': use_sim_time
                }
            ],
            output='screen'
        ),

        # Spawn Robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'bumperbot',
                '-topic', '/robot_description'
            ],
            output='screen'
        ),

        # Bridge Gazebo clock to ROS 2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),

        # Bridge joint states from Gazebo to ROS 2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/world/empty/model/bumperbot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'],
            remappings=[('/world/empty/model/bumperbot/joint_state', '/joint_states')],
            output='screen'
        ),

        # Bridge robot pose from Gazebo to ROS 2 (for RViz to see robot movement)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/bumperbot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
            remappings=[('/model/bumperbot/pose', '/tf')],
            output='screen'
        ),

        # Bridge odometry from Gazebo
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/bumperbot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
            remappings=[('/model/bumperbot/odometry', '/odom')],
            output='screen'
        ),

        # NOTE: Controllers are NOT loaded here
        # Use controller.launch.py to load them manually after this launch file
        # Bridge laser scan from Gazebo to ROS
        # Bridge laser scan with frame_id remapping
        # Bridge laser scan
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
            ],
            output='screen'
        ),
        
        # Static transform to match Gazebo's frame name
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'bumperbot/base_footprint/laser'],
            output='screen'
        ),
    ])