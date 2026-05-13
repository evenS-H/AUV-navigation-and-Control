import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from mavros_msgs.msg import OverrideRCIn

from . import Thruster_Vectoring_Params as P


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


# Piecewise linear interpolation over a sorted lookup table
def table_lookup(x: float, xs: list, ys: list) -> float:
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for i in range(len(xs) - 1):
        x0, x1 = xs[i], xs[i + 1]
        if x0 <= x <= x1:
            y0, y1 = ys[i], ys[i + 1]
            t = (x - x0) / (x1 - x0) if (x1 - x0) else 0.0
            return y0 + t * (y1 - y0)
    return ys[-1]


def angle_to_pwm(angle_rad: float, sign: float, max_rad: float, lut_deg: list, lut_pwm: list) -> int:
    ang = clamp(angle_rad * sign, -max_rad, max_rad)
    deg = math.degrees(ang)
    pwm = table_lookup(deg, lut_deg, [float(v) for v in lut_pwm])
    return int(round(clamp(pwm, P.PWM_MIN, P.PWM_MAX)))


class Thruster_Vectoring(Node):
    def __init__(self):
        super().__init__("thruster_vectoring")

        self.declare_parameter("manual_stop",  False)
        self.declare_parameter("thruster_pwm", P.THRUSTER_PWM)

        self.manual_stop  = bool(self.get_parameter("manual_stop").value)
        self.thruster_pwm = int(self.get_parameter("thruster_pwm").value)

        self.mission_active = False
        self.prev_thruster_on = None

        self.u_yaw   = 0.0
        self.u_pitch = 0.0
        self.last_cmd = time.monotonic()

        self.cur_pitch_pwm = angle_to_pwm(0.0, 1.0, P.PITCH_MAX_RAD, P.PITCH_LUT_DEG, P.PITCH_LUT_PWM)
        self.cur_yaw_pwm   = angle_to_pwm(0.0, 1.0, P.YAW_MAX_RAD,   P.YAW_LUT_DEG,   P.YAW_LUT_PWM)
        self.cur_thr_pwm   = P.THRUSTER_NEUTRAL

        self.rc_pub = self.create_publisher(OverrideRCIn, P.RC_OVERRIDE_TOPIC, 10)

        self.create_subscription(Vector3, P.YAW_PITCH_TOPIC, self.cb_yaw_pitch, 10)
        self.create_subscription(Bool, "/mission_active", self.cb_mission_active, 10)

        self.dt = 1.0 / P.UPDATE_HZ
        self.timer = self.create_timer(self.dt, self.on_timer)

        # Send neutral on startup
        self.publish_rc_override(
            self.cur_pitch_pwm,
            self.cur_yaw_pwm,
            self.cur_thr_pwm if P.CONTROL_THRUSTER else None,
        )

        self.get_logger().info(
            f"Thruster_Vectoring ready. pitch_rc={P.PITCH_RC_CH} yaw_rc={P.YAW_RC_CH} "
            f"thruster_rc={P.THRUSTER_RC_CH} update_hz={P.UPDATE_HZ}"
        )

    def cb_yaw_pitch(self, msg: Vector3):
        self.u_yaw   = float(msg.x)
        self.u_pitch = float(msg.y)
        self.last_cmd = time.monotonic()

    def cb_mission_active(self, msg: Bool):
        self.mission_active = bool(msg.data)

    def publish_rc_override(self, pitch_pwm: int, yaw_pwm: int, thruster_pwm: Optional[int] = None):
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18

        # RC channels are 1-based; array is 0-based
        msg.channels[P.PITCH_RC_CH - 1] = int(clamp(pitch_pwm, P.PWM_MIN, P.PWM_MAX))
        msg.channels[P.YAW_RC_CH - 1]   = int(clamp(yaw_pwm,   P.PWM_MIN, P.PWM_MAX))

        if P.CONTROL_THRUSTER and thruster_pwm is not None:
            msg.channels[P.THRUSTER_RC_CH - 1] = int(clamp(thruster_pwm, P.PWM_MIN, P.PWM_MAX))

        self.rc_pub.publish(msg)

    def slew(self, cur: int, target: int, rate_pwm_s: float) -> int:
        max_step = rate_pwm_s * self.dt
        delta = clamp(target - cur, -max_step, max_step)
        return int(round(cur + delta))

    def update_params(self):
        self.manual_stop  = bool(self.get_parameter("manual_stop").value)
        self.thruster_pwm = int(clamp(int(self.get_parameter("thruster_pwm").value), P.PWM_MIN, P.PWM_MAX))

    def on_timer(self):
        self.update_params()

        if (time.monotonic() - self.last_cmd) > P.CMD_TIMEOUT_S:
            yaw_cmd   = 0.0
            pitch_cmd = 0.0
        else:
            yaw_cmd   = self.u_yaw
            pitch_cmd = self.u_pitch

        tgt_pitch = angle_to_pwm(pitch_cmd, P.PITCH_SIGN, P.PITCH_MAX_RAD, P.PITCH_LUT_DEG, P.PITCH_LUT_PWM)
        tgt_yaw   = angle_to_pwm(yaw_cmd,   P.YAW_SIGN,   P.YAW_MAX_RAD,   P.YAW_LUT_DEG,   P.YAW_LUT_PWM)

        self.cur_pitch_pwm = self.slew(self.cur_pitch_pwm, tgt_pitch, P.SLEW_PWM_PER_SEC)
        self.cur_yaw_pwm   = self.slew(self.cur_yaw_pwm,   tgt_yaw,   P.SLEW_PWM_PER_SEC)

        if P.CONTROL_THRUSTER:
            thruster_on = self.mission_active and not self.manual_stop
            desired_thr = self.thruster_pwm if thruster_on else P.THRUSTER_NEUTRAL
            self.cur_thr_pwm = int(clamp(desired_thr, P.PWM_MIN, P.PWM_MAX))

            if thruster_on != self.prev_thruster_on:
                if thruster_on:
                    self.get_logger().info(f"Thruster ON - mission active, pwm={self.thruster_pwm}")
                elif self.manual_stop:
                    self.get_logger().info("Thruster OFF - manual_stop active")
                else:
                    self.get_logger().info("Thruster OFF - mission inactive")
                self.prev_thruster_on = thruster_on

            self.publish_rc_override(self.cur_pitch_pwm, self.cur_yaw_pwm, self.cur_thr_pwm)
        else:
            self.publish_rc_override(self.cur_pitch_pwm, self.cur_yaw_pwm, None)


def main():
    rclpy.init()
    node = Thruster_Vectoring()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            neutral_pitch = angle_to_pwm(0.0, 1.0, P.PITCH_MAX_RAD, P.PITCH_LUT_DEG, P.PITCH_LUT_PWM)
            neutral_yaw   = angle_to_pwm(0.0, 1.0, P.YAW_MAX_RAD,   P.YAW_LUT_DEG,   P.YAW_LUT_PWM)
            if P.CONTROL_THRUSTER:
                node.publish_rc_override(neutral_pitch, neutral_yaw, P.THRUSTER_NEUTRAL)
            else:
                node.publish_rc_override(neutral_pitch, neutral_yaw, None)
            time.sleep(0.1)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

