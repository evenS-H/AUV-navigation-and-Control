#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3, PointStamped
from .LOS_Main import LOSMain, LOSParams

# EKF
from nav_msgs.msg import Odometry


class LOS(Node):

    def __init__(self):
        super().__init__('los')
        self.get_logger().info('LOS started.')

        params = LOSParams()
        self.los = LOSMain(params)

        # Storage for latest navigation data
        self.pose = None
        self.ref = None
        self.target = None

        # Subscribe to Navigation data (position, reference, target)
        self.sub_ekf = self.create_subscription(
            Odometry, "/odometry/filtered", self.on_ekf_odom, 10
        )

        self.sub_ref = self.create_subscription(
            PointStamped, "/los_ref", self.on_ref, 10
        )

        self.sub_target = self.create_subscription(
            PointStamped, "/los_target", self.on_target, 10
        )

        # Publish desired course angles
        self.cmd_pub = self.create_publisher(Vector3, "/los/cmd_course", 10)

        # Timer
        self.timer = self.create_timer(float(params.dt), self.on_timer)

    def on_ekf_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pose = Point(x=float(p.x), y=float(p.y), z=float(p.z))

        self.get_logger().debug(
            f"EKF position: ({self.pose.x:.2f}, {self.pose.y:.2f}, {self.pose.z:.2f})"
        )

    def on_ref(self, msg: PointStamped):
        self.ref = msg.point

        self.get_logger().debug(
            f"reference: ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f})"
        )

    def on_target(self, msg: PointStamped):
        self.target = msg.point

        self.get_logger().debug(
            f"target: ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f})"
        )

    def on_timer(self):
        if self.pose is None or self.ref is None or self.target is None:
            return  # Wait until all data is available

        x, y, z = self.pose.x, self.pose.y, self.pose.z
        x_ref, y_ref, z_ref = self.ref.x, self.ref.y, self.ref.z
        x_t, y_t, z_t = self.target.x, self.target.y, self.target.z

        try:
            out = self.los.update(x, y, z, x_ref, y_ref, z_ref, x_t, y_t, z_t)
        except Exception as e:
            self.get_logger().error(f"LOS update failed: {e}")
            return

        chi_d_xy = out["chi_d_xy"]
        chi_d_xz = out["chi_d_xz"]

        msg = Vector3()
        msg.x = float(chi_d_xy)
        msg.y = float(chi_d_xz)
        msg.z = 0.0
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LOS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
