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

    def __init__(self):
        super().__init__('wheel_odometry')

        # ----------------------------
        # Parameters
        # ----------------------------
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.14022)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        
        # Motion noise parameters (standard deviations)
        # These represent real-world uncertainties in odometry
        self.declare_parameter('enable_motion_noise', True)
        self.declare_parameter('alpha1', 0.05)  # rot->rot noise (rad^2/rad)
        self.declare_parameter('alpha2', 0.005)  # trans->rot noise (rad^2/m)
        self.declare_parameter('alpha3', 0.05)  # trans->trans noise (m^2/m)
        self.declare_parameter('alpha4', 0.005)  # rot->trans noise (m^2/rad)
        
        # Covariance parameters
        self.declare_parameter('enable_covariance', True)
        self.declare_parameter('base_position_covariance', 0.001)  # m^2
        self.declare_parameter('base_orientation_covariance', 0.001)  # rad^2
        self.declare_parameter('base_velocity_covariance', 0.01)  # (m/s)^2 or (rad/s)^2

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # Get noise parameters
        self.enable_motion_noise = self.get_parameter('enable_motion_noise').value
        self.alpha1 = self.get_parameter('alpha1').value
        self.alpha2 = self.get_parameter('alpha2').value
        self.alpha3 = self.get_parameter('alpha3').value
        self.alpha4 = self.get_parameter('alpha4').value
        
        # Get covariance parameters
        self.enable_covariance = self.get_parameter('enable_covariance').value
        self.base_pos_cov = self.get_parameter('base_position_covariance').value
        self.base_ori_cov = self.get_parameter('base_orientation_covariance').value
        self.base_vel_cov = self.get_parameter('base_velocity_covariance').value

        # ----------------------------
        # Publishers / Subscribers
        # ----------------------------
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # ----------------------------
        # State
        # ----------------------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_left_pos = None
        self.last_right_pos = None
        self.last_time = self.get_clock().now()
        
        # Accumulated covariance (uncertainty grows with movement)
        self.pose_covariance = np.zeros((3, 3))  # x, y, theta
        self.pose_covariance[0, 0] = self.base_pos_cov  # x variance
        self.pose_covariance[1, 1] = self.base_pos_cov  # y variance
        self.pose_covariance[2, 2] = self.base_ori_cov  # theta variance

        self.get_logger().info('Wheel Odometry started')
        self.get_logger().info(f'Publishing TF: {self.odom_frame} → {self.base_frame}')
        self.get_logger().info(f'Motion noise enabled: {self.enable_motion_noise}')
        self.get_logger().info(f'Covariance enabled: {self.enable_covariance}')

    def joint_callback(self, msg):
        now = self.get_clock().now()

        try:
            left_idx = msg.name.index('wheel_left_joint')
            right_idx = msg.name.index('wheel_right_joint')
        except ValueError:
            return

        left_pos = msg.position[left_idx]
        right_pos = msg.position[right_idx]

        if self.last_left_pos is None:
            self.last_left_pos = left_pos
            self.last_right_pos = right_pos
            self.last_time = now
            return

        dl = (left_pos - self.last_left_pos) * self.wheel_radius
        dr = (right_pos - self.last_right_pos) * self.wheel_radius

        distance = (dl + dr) / 2.0
        dtheta = (dr - dl) / self.wheel_separation
        
        # Apply motion noise if enabled (models wheel slip, encoder errors, etc.)
        if self.enable_motion_noise:
            # Sample noise based on motion model (Probabilistic Robotics, Thrun et al.)
            # Noise depends on both the distance traveled and rotation
            sigma_trans = math.sqrt(
                self.alpha3 * abs(distance) + 
                self.alpha4 * abs(dtheta)
            )
            sigma_rot = math.sqrt(
                self.alpha1 * abs(dtheta) + 
                self.alpha2 * abs(distance)
            )
            
            # Add Gaussian noise to motion
            distance_noisy = distance + np.random.normal(0, sigma_trans)
            dtheta_noisy = dtheta + np.random.normal(0, sigma_rot)
        else:
            distance_noisy = distance
            dtheta_noisy = dtheta

        # Update pose with noisy motion
        self.x += distance_noisy * math.cos(self.theta + dtheta_noisy / 2.0)
        self.y += distance_noisy * math.sin(self.theta + dtheta_noisy / 2.0)
        self.theta += dtheta_noisy
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Update covariance (uncertainty accumulates with movement)
        if self.enable_covariance:
            # Jacobian of motion model with respect to pose
            cos_theta = math.cos(self.theta)
            sin_theta = math.sin(self.theta)
            
            # Simplified covariance propagation
            # In practice, this grows with distance traveled
            distance_abs = abs(distance)
            dtheta_abs = abs(dtheta)
            
            self.pose_covariance[0, 0] += (
                self.alpha3 * distance_abs + 
                self.alpha4 * dtheta_abs
            ) * cos_theta**2
            
            self.pose_covariance[1, 1] += (
                self.alpha3 * distance_abs + 
                self.alpha4 * dtheta_abs
            ) * sin_theta**2
            
            self.pose_covariance[2, 2] += (
                self.alpha1 * dtheta_abs + 
                self.alpha2 * distance_abs
            )

        dt = (now - self.last_time).nanoseconds / 1e9
        vx = distance / dt if dt > 0 else 0.0
        wz = dtheta / dt if dt > 0 else 0.0

        self.publish_odometry(now, vx, wz)

        self.last_left_pos = left_pos
        self.last_right_pos = right_pos
        self.last_time = now

    def publish_odometry(self, time, vx, wz):
        odom = Odometry()
        odom.header.stamp = time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        
        # Populate pose covariance (6x6 matrix: x, y, z, roll, pitch, yaw)
        # ROS uses row-major order, we only fill x, y, and yaw (theta)
        if self.enable_covariance:
            odom.pose.covariance = [0.0] * 36  # Initialize 6x6 matrix
            odom.pose.covariance[0] = self.pose_covariance[0, 0]   # x variance
            odom.pose.covariance[7] = self.pose_covariance[1, 1]   # y variance
            odom.pose.covariance[35] = self.pose_covariance[2, 2]  # yaw variance
            
            # Twist covariance (velocity uncertainty)
            odom.twist.covariance = [0.0] * 36
            odom.twist.covariance[0] = self.base_vel_cov   # linear x velocity variance
            odom.twist.covariance[35] = self.base_vel_cov  # angular z velocity variance

        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = WheelOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()