#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import math


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

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

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

        self.get_logger().info('Wheel Odometry started')
        self.get_logger().info(f'Publishing TF: {self.odom_frame} → {self.base_frame}')

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

        self.x += distance * math.cos(self.theta + dtheta / 2.0)
        self.y += distance * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

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
