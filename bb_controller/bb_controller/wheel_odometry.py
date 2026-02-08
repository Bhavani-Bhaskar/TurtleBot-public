#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import math
import numpy as np


class WheelOdometry(Node):
    """
    Wheel Odometry Node
    - Subscribes to /joint_states (wheel encoders)
    - Calculates robot pose using differential drive kinematics
    - Publishes /odom topic with covariance
    - Broadcasts odom → base_link TF transform
    - Includes realistic noise modeling
    """
    
    def __init__(self):
        super().__init__('wheel_odometry')
        
        # Parameters
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.14022)
        self.declare_parameter('publish_tf', True)
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # Noise parameters (process noise - motion uncertainty)
        self.alpha = [0.001, 0.001, 0.001, 0.001]  # [α1, α2, α3, α4]
        
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}')
        self.get_logger().info(f'Wheel separation: {self.wheel_separation}')
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriber
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.last_left_pos = None
        self.last_right_pos = None
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('Wheel Odometry Node Started')
    
    def joint_callback(self, msg):
        current_time = self.get_clock().now()
        
        # Get wheel positions (find indices by name)
        try:
            left_idx = msg.name.index('wheel_left_joint')
            right_idx = msg.name.index('wheel_right_joint')
        except ValueError:
            self.get_logger().warn('Could not find wheel joints in joint_states')
            return
        
        left_pos = msg.position[left_idx]
        right_pos = msg.position[right_idx]
        
        # Initialize on first callback
        if self.last_left_pos is None:
            self.last_left_pos = left_pos
            self.last_right_pos = right_pos
            self.last_time = current_time
            return
        
        # Calculate change in wheel positions
        delta_left = left_pos - self.last_left_pos
        delta_right = right_pos - self.last_right_pos
        
        # Convert to distances
        left_distance = delta_left * self.wheel_radius
        right_distance = delta_right * self.wheel_radius
        
        # Calculate robot motion (differential drive kinematics)
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_separation
        
        # Update pose (using improved integration)
        self.x += distance * math.cos(self.theta + delta_theta / 2.0)
        self.y += distance * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Calculate velocities
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt > 0:
            linear_vel = distance / dt
            angular_vel = delta_theta / dt
        else:
            linear_vel = 0.0
            angular_vel = 0.0
        
        # Calculate covariance (uncertainty grows with motion)
        covariance = self.calculate_covariance(linear_vel, angular_vel, dt)
        
        # Publish odometry
        self.publish_odometry(current_time, linear_vel, angular_vel, covariance)
        
        # Update last values
        self.last_left_pos = left_pos
        self.last_right_pos = right_pos
        self.last_time = current_time
    
    def calculate_covariance(self, v, omega, dt):
        """
        Calculate odometry covariance based on motion
        Uses motion-based noise model from Probabilistic Robotics
        Uncertainty grows with distance traveled and rotation
        """
        # Position uncertainty (grows with distance and rotation)
        sigma_x_sq = (self.alpha[0] * abs(v) + self.alpha[1] * abs(omega))**2 * dt**2
        sigma_y_sq = sigma_x_sq  # Assume symmetric
        
        # Orientation uncertainty (grows with rotation and distance)
        sigma_theta_sq = (self.alpha[2] * abs(v) + self.alpha[3] * abs(omega))**2 * dt**2
        
        # Velocity uncertainty
        sigma_v_sq = 0.01  # m/s
        sigma_omega_sq = 0.01  # rad/s
        
        # Build 6x6 covariance matrix (x, y, z, roll, pitch, yaw)
        # We only populate x, y, yaw since it's a 2D robot
        pose_covariance = [
            sigma_x_sq,     0,              0, 0, 0, 0,              # x
            0,              sigma_y_sq,     0, 0, 0, 0,              # y
            0,              0,              0, 0, 0, 0,              # z (unused)
            0,              0,              0, 0, 0, 0,              # roll (unused)
            0,              0,              0, 0, 0, 0,              # pitch (unused)
            0,              0,              0, 0, 0, sigma_theta_sq  # yaw
        ]
        
        # Build 6x6 twist covariance matrix (vx, vy, vz, wx, wy, wz)
        twist_covariance = [
            sigma_v_sq,     0,              0, 0, 0, 0,                 # vx
            0,              sigma_v_sq,     0, 0, 0, 0,                 # vy
            0,              0,              0, 0, 0, 0,                 # vz (unused)
            0,              0,              0, 0, 0, 0,                 # wx (unused)
            0,              0,              0, 0, 0, 0,                 # wy (unused)
            0,              0,              0, 0, 0, sigma_omega_sq     # wz
        ]
        
        return pose_covariance, twist_covariance
    
    def publish_odometry(self, current_time, linear_vel, angular_vel, covariance):
        """Publish odometry message with covariance"""
        pose_cov, twist_cov = covariance
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (convert theta to quaternion)
        quat = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        # Covariance
        odom.pose.covariance = pose_cov
        
        # Velocity
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel
        
        # Velocity covariance
        odom.twist.covariance = twist_cov
        
        # Publish
        self.odom_pub.publish(odom)
        
        # Also publish TF transform
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            
            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = WheelOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()