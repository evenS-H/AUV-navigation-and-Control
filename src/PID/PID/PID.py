import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

from . import PID_params as P
from . import PID_Main as runner


class PID(Node):

    def __init__(self):
        super().__init__("pid")
        self.get_logger().info("PID started.")

        self.cmd_course = None
        self.chi_yaw_meas: float = 0.0
        self.chi_pitch_meas: float = 0.0
        self._got_chi_yaw = False
        self._got_chi_pitch = False

        # Subscribe to LOS course command
        self.sub_los = self.create_subscription(
            Vector3, "/los/cmd_course", self.on_cmd_course, 10
        )

        # Subscribe to measured course angles from Navigation
        self.sub_chi_yaw = self.create_subscription(
            Float64, "/odometry/chi_yaw", self.on_chi_yaw, 10
        )
        self.sub_chi_pitch = self.create_subscription(
            Float64, "/odometry/chi_pitch", self.on_chi_pitch, 10
        )

        # Publish combined yaw/pitch command
        self.cmd_yaw_pitch_pub = self.create_publisher(
            Vector3, "/pid/cmd_yaw_pitch", 10
        )

        # Publish individual axis commands
        self.yaw_pub = self.create_publisher(
            Float64, "/pid/raw_gimbal_yaw_cmd", 10
        )
        self.pitch_pub = self.create_publisher(
            Float64, "/pid/raw_gimbal_pitch_cmd", 10
        )

        self.timer = self.create_timer(float(P.YAW.DT), self.on_timer)

    def on_cmd_course(self, msg: Vector3):
        self.cmd_course = msg

    def on_chi_yaw(self, msg: Float64):
        self.chi_yaw_meas = float(msg.data)
        self._got_chi_yaw = True

    def on_chi_pitch(self, msg: Float64):
        self.chi_pitch_meas = float(msg.data)
        self._got_chi_pitch = True

    def on_timer(self):
        if self.cmd_course is None:
            return
        if not (self._got_chi_yaw and self._got_chi_pitch):
            return

        yaw_ref   = float(self.cmd_course.x)
        pitch_ref = float(self.cmd_course.y)

        try:
            u_yaw, u_pitch = runner.step(
                yaw_ref, self.chi_yaw_meas,
                pitch_ref, self.chi_pitch_meas,
            )
        except Exception as e:
            self.get_logger().error(f"PID step failed: {e}")
            return

        out = Vector3()
        out.x = float(u_yaw)
        out.y = float(u_pitch)
        out.z = 0.0
        self.cmd_yaw_pitch_pub.publish(out)

        #Stonefish
        self.yaw_pub.publish(Float64(data=float(-u_yaw)))
        self.pitch_pub.publish(Float64(data=float(-u_pitch)))


def main(args=None):
    rclpy.init(args=args)
    node = PID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
