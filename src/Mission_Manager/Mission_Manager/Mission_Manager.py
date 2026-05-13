import math
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32, Bool


@dataclass
class NedPoint:
    x: float  # North [m]
    y: float  # East  [m]
    z: float  # Down  [m]


class Mission_Manager(Node):
    def __init__(self):
        super().__init__("mission_manager")

        self.declare_parameter("acceptance_radius", 2.0)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("loop", False)
        self.declare_parameter("start_index", 0)
        self.declare_parameter("route_topic", "/auv_route_ned")
        self.declare_parameter("ekf_topic",   "/odometry/filtered")

        route_topic = str(self.get_parameter("route_topic").value)
        ekf_topic   = str(self.get_parameter("ekf_topic").value)

        self._wps: List[NedPoint] = []
        self._seg_i: int = 0
        self._active: bool = False
        self._pos: Optional[NedPoint] = None
        self._frame_id = "odom"

        self.sub_route = self.create_subscription(Path,     route_topic, self._on_route, 10)
        self.sub_ekf   = self.create_subscription(Odometry, ekf_topic,   self._on_ekf,   50)

        self.pub_ref    = self.create_publisher(PointStamped, "/los_ref",              10)
        self.pub_target = self.create_publisher(PointStamped, "/los_target",           10)
        self.pub_index  = self.create_publisher(Int32,        "/mission_segment_index", 10)
        self.pub_active = self.create_publisher(Bool,         "/mission_active",        10)

        self.timer_pub = self.create_timer(0.5, self._publish_segment)

        self.get_logger().info(f"Mission_Manager listening on route: {route_topic}")
        self.get_logger().info(f"Mission_Manager listening on EKF:   {ekf_topic}")
        self.get_logger().info("Publishes: /los_ref, /los_target (PointStamped NED)")

        self._publish_active()

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_route(self, msg: Path):
        if len(msg.poses) < 2:
            self.get_logger().warning("Route has fewer than 2 waypoints — ignored.")
            return

        self._wps = [
            NedPoint(
                x=float(ps.pose.position.x),
                y=float(ps.pose.position.y),
                z=float(ps.pose.position.z),
            )
            for ps in msg.poses
        ]

        start_index = int(self.get_parameter("start_index").value)
        start_index = max(0, min(start_index, len(self._wps) - 2))

        self._seg_i  = start_index
        self._active = True
        self._publish_active()

        self.get_logger().info(
            f"New route: {len(self._wps)} waypoints, starting at segment {self._seg_i}."
        )

        if bool(self.get_parameter("auto_start").value):
            self._publish_segment()

    def _on_ekf(self, msg: Odometry):
        p = msg.pose.pose.position
        self._pos = NedPoint(x=float(p.x), y=float(p.y), z=float(p.z))
        self._frame_id = msg.header.frame_id

        if not self._active or len(self._wps) < 2 or self._seg_i >= len(self._wps) - 1:
            return

        target = self._wps[self._seg_i + 1]
        dist   = self._distance_3d(self._pos, target)
        acc    = float(self.get_parameter("acceptance_radius").value)

        if dist <= acc:
            self.get_logger().info(
                f"Reached segment {self._seg_i} target (dist={dist:.2f} m <= {acc:.2f} m). Advancing."
            )
            self._advance_and_publish()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _publish_active(self):
        msg = Bool()
        msg.data = self._active
        self.pub_active.publish(msg)

    @staticmethod
    def _distance_3d(a: NedPoint, b: NedPoint) -> float:
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

    def _publish_segment(self):
        if not self._active or len(self._wps) < 2:
            return
        if self._seg_i >= len(self._wps) - 1:
            return

        ref    = self._wps[self._seg_i]
        target = self._wps[self._seg_i + 1]
        now    = self.get_clock().now().to_msg()

        ref_msg = PointStamped()
        ref_msg.header.frame_id = self._frame_id
        ref_msg.header.stamp    = now
        ref_msg.point.x = ref.x
        ref_msg.point.y = ref.y
        ref_msg.point.z = ref.z
        self.pub_ref.publish(ref_msg)

        tgt_msg = PointStamped()
        tgt_msg.header.frame_id = self._frame_id
        tgt_msg.header.stamp    = now
        tgt_msg.point.x = target.x
        tgt_msg.point.y = target.y
        tgt_msg.point.z = target.z
        self.pub_target.publish(tgt_msg)

        idx = Int32()
        idx.data = self._seg_i
        self.pub_index.publish(idx)

        self.get_logger().debug(
            f"Segment {self._seg_i}: "
            f"ref(N={ref.x:.2f}, E={ref.y:.2f}, D={ref.z:.2f}) -> "
            f"tgt(N={target.x:.2f}, E={target.y:.2f}, D={target.z:.2f})"
        )

    def _advance_and_publish(self):
        self._seg_i += 1

        if self._seg_i >= len(self._wps) - 1:
            if bool(self.get_parameter("loop").value):
                self._seg_i = 0
                self.get_logger().info("End of route — looping to segment 0.")
            else:
                self.get_logger().info("End of route — mission complete.")
                self._active = False
                self._publish_active()
                return

        self._publish_segment()


def main():
    rclpy.init()
    node = Mission_Manager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
