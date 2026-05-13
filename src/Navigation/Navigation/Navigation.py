#!/usr/bin/env python3

import math
from collections import deque
from typing import Deque, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Quaternion, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


# Physical constants
RHO = 1025.0   # seawater density [kg/m^3]
G   = 9.80665  # gravitational acceleration [m/s^2]

# Timing and filter constants
HZ = 25.0
VZ_ALPHA = 0.25
CAL_SAMPLES = 20

# Median filter window for depth (removes single-sample dropouts)
PRESSURE_MEDIAN_WINDOW = 3

# Position history window for NED velocity estimation
POS_WINDOW_SEC = 0.4
MIN_GROUND_SPEED = 0.03  # [m/s] — hold last valid course below this speed

# Freshness timeouts
DVL_TIMEOUT_SEC      = 0.5
PRESSURE_TIMEOUT_SEC = 0.5
HEADING_TIMEOUT_SEC  = 0.5

# Calibration timing
HEADING_BUFFER_SEC       = 2.0
RESET_DELAY_SEC          = 0.05
HEADING_WINDOW_SEC       = 0.15
CALIBRATION_FINALIZE_SEC = 0.15

# Fixed compass mounting offset — set to 0.0 if not needed
HEADING_OFFSET_DEG = 0.0
HEADING_OFFSET_RAD = math.radians(HEADING_OFFSET_DEG)

PointT   = Tuple[float, float, float, float]  # (t_sec, north, east, down)
HeadingT = Tuple[float, float]                # (t_sec, heading_deg)


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll  * 0.5)
    sr = math.sin(roll  * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw   * 0.5)
    sy = math.sin(yaw   * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def rotate_xy(yaw: float, x: float, y: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


def circular_mean_rad(angles_rad) -> float:
    if not angles_rad:
        return 0.0
    s = sum(math.sin(a) for a in angles_rad)
    c = sum(math.cos(a) for a in angles_rad)
    if abs(s) < 1e-9 and abs(c) < 1e-9:
        return 0.0
    return math.atan2(s, c)


class Navigation(Node):
    """
    Fused odometry node.

    Fuses DVL dead-reckoning position, Bar30 pressure depth, and compass
    heading into a NED odometry estimate published on /odometry/filtered.

    Call /calibrate_depth_zero to trigger:
      1. DVL dead-reckoning reset
      2. Compass heading capture (sets the yaw offset between DVL frame and NED)
      3. Pressure zero reference reset
    """

    def __init__(self) -> None:
        super().__init__("navigation")

        self.declare_parameter("heading_topic",    "/mavros/global_position/compass_hdg")
        self.declare_parameter("dvl_reset_service", "/dvl/reset_dead_reckoning")

        self.heading_topic    = str(self.get_parameter("heading_topic").value)
        self.dvl_reset_service = str(self.get_parameter("dvl_reset_service").value)

        # DVL data in DVL local/start frame
        self.pos: Optional[Tuple[float, float, float]] = None
        self.vel_body: Optional[Tuple[float, float, float]] = None
        self.rpy_deg: Optional[Tuple[float, float, float]] = None

        self.last_pos_stamp: Optional[Time] = None
        self.last_vel_stamp: Optional[Time] = None
        self.last_rpy_stamp: Optional[Time] = None

        # Compass heading (corrected with mounting offset)
        self.heading_deg: Optional[float] = None
        self.last_heading_stamp: Optional[Time] = None
        self.heading_history: Deque[HeadingT] = deque()

        # Yaw offset: DVL reset frame -> NED world frame
        self.yaw_offset_rad: float = 0.0
        self.yaw_offset_valid: bool = False

        # Pressure / depth state
        self.pressures: Deque[float] = deque(maxlen=CAL_SAMPLES)
        self.p_ref: Optional[float] = None
        self.z = 0.0
        self.vz_down = 0.0
        self.last_z: Optional[float] = None
        self.last_pressure_t: Optional[Time] = None
        self.last_valid_pressure_t: Optional[Time] = None
        self.pressure_z_window: Deque[float] = deque(maxlen=PRESSURE_MEDIAN_WINDOW)

        # NED velocity and course angles derived from DVL position history
        self.pos_history: Deque[PointT] = deque()
        self.v_north = 0.0
        self.v_east  = 0.0
        self.v_down  = 0.0
        self.chi_yaw   = 0.0
        self.chi_pitch = 0.0
        self.have_valid_course = False

        # Async calibration sequence state
        self.calibration_pending = False
        self.calibration_started_at: Optional[Time] = None
        self.calibration_target_t_sec: Optional[float] = None
        self.reset_future = None
        self.reset_response_ok = False
        self.reset_response_message = ""

        # Subscriptions
        self.create_subscription(Vector3Stamped, "/dvl/raw/pos", self.on_pos, 10)
        self.create_subscription(Vector3Stamped, "/dvl/raw/vel", self.on_vel, 10)
        self.create_subscription(Vector3Stamped, "/dvl/raw/rpy", self.on_rpy, 10)

        be_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Float64,       self.heading_topic,            self.on_heading,  be_qos)
        self.create_subscription(FluidPressure, "/bar30/pressure", self.on_pressure, be_qos)

        # Services
        self.create_service(Trigger, "/calibrate_depth_zero", self.on_calibrate)
        self.reset_client = self.create_client(Trigger, self.dvl_reset_service)

        # Publishers
        self.odom_pub      = self.create_publisher(Odometry,       "/odometry/filtered",      10)
        self.vel_ned_pub   = self.create_publisher(Vector3Stamped, "/odometry/velocity_ned",  10)
        self.chi_yaw_pub   = self.create_publisher(Float64,        "/odometry/chi_yaw",        10)
        self.chi_pitch_pub = self.create_publisher(Float64,        "/odometry/chi_pitch",      10)
        self.yaw_offset_pub = self.create_publisher(Float64,       "/odometry/yaw_offset",    10)

        self.create_timer(1.0 / HZ, self.tick)

        self.get_logger().info(
            f"Navigation started. Heading topic: {self.heading_topic}. "
            f"DVL reset service: {self.dvl_reset_service}. "
            f"Heading offset: {HEADING_OFFSET_DEG:.2f} deg. "
            "Call /calibrate_depth_zero to initialise."
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def on_pos(self, msg: Vector3Stamped) -> None:
        self.pos = (msg.vector.x, msg.vector.y, msg.vector.z)
        self.last_pos_stamp = self.stamp_or_now(msg.header.stamp)
        self.update_course_from_position(self.last_pos_stamp)

    def on_vel(self, msg: Vector3Stamped) -> None:
        self.vel_body = (msg.vector.x, msg.vector.y, msg.vector.z)
        self.last_vel_stamp = self.stamp_or_now(msg.header.stamp)

    def on_rpy(self, msg: Vector3Stamped) -> None:
        self.rpy_deg = (msg.vector.x, msg.vector.y, msg.vector.z)
        self.last_rpy_stamp = self.stamp_or_now(msg.header.stamp)

    def on_heading(self, msg: Float64) -> None:
        raw = float(msg.data)
        self.heading_deg = (raw + HEADING_OFFSET_DEG) % 360.0
        stamp = self.get_clock().now()
        self.last_heading_stamp = stamp
        t_sec = stamp.nanoseconds * 1e-9
        self.heading_history.append((t_sec, self.heading_deg))
        while len(self.heading_history) >= 2 and (t_sec - self.heading_history[0][0]) > HEADING_BUFFER_SEC:
            self.heading_history.popleft()

    def on_pressure(self, msg: FluidPressure) -> None:
        p = float(msg.fluid_pressure)
        now = self.get_clock().now()

        self.pressures.append(p)
        self.last_pressure_t = now

        if self.p_ref is None:
            return

        raw_z = (p - self.p_ref) / (RHO * G)

        self.pressure_z_window.append(raw_z)
        z_candidate = sorted(self.pressure_z_window)[len(self.pressure_z_window) // 2]

        if self.last_z is not None and self.last_valid_pressure_t is not None:
            dt = (now - self.last_valid_pressure_t).nanoseconds * 1e-9
            if dt > 1e-4:
                raw_vz = (z_candidate - self.last_z) / dt
                self.vz_down = VZ_ALPHA * raw_vz + (1.0 - VZ_ALPHA) * self.vz_down
                self.v_down = self.vz_down

        self.z = z_candidate
        self.last_z = z_candidate
        self.last_valid_pressure_t = now

    def on_calibrate(self, request, response):
        del request

        if self.calibration_pending:
            response.success = False
            response.message = "Calibration already in progress."
            return response

        if not self.pressures:
            response.success = False
            response.message = "No pressure samples received yet."
            return response

        if self.heading_deg is None or self.is_stale(self.last_heading_stamp, self.get_clock().now(), HEADING_TIMEOUT_SEC):
            response.success = False
            response.message = "No fresh heading available."
            return response

        if not self.reset_client.wait_for_service(timeout_sec=0.5):
            response.success = False
            response.message = (
                f"DVL reset service '{self.dvl_reset_service}' not found."
            )
            return response

        self.calibration_pending = True
        self.calibration_started_at = self.get_clock().now()
        self.calibration_target_t_sec = self.calibration_started_at.nanoseconds * 1e-9 + RESET_DELAY_SEC
        self.reset_response_ok = False
        self.reset_response_message = ""

        self.reset_future = self.reset_client.call_async(Trigger.Request())
        self.reset_future.add_done_callback(self._on_reset_done)

        response.success = True
        response.message = (
            "Calibration started: sending DVL reset, capturing heading, resetting depth zero."
        )
        self.get_logger().info(response.message)
        return response

    def _on_reset_done(self, future) -> None:
        try:
            result = future.result()
            self.reset_response_ok = bool(result.success)
            self.reset_response_message = str(result.message)
        except Exception as exc:
            self.reset_response_ok = False
            self.reset_response_message = f"Reset call failed: {exc}"

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def tick(self) -> None:
        now = self.get_clock().now()

        if self.calibration_pending:
            self.maybe_finalize_calibration(now)

        if self.pos is None or self.rpy_deg is None or self.p_ref is None or not self.yaw_offset_valid:
            return

        if self.is_stale(self.last_pos_stamp,  now, DVL_TIMEOUT_SEC):
            return
        if self.is_stale(self.last_rpy_stamp,  now, DVL_TIMEOUT_SEC):
            return
        if self.is_stale(self.last_pressure_t, now, PRESSURE_TIMEOUT_SEC):
            return

        north, east = self.get_world_xy()
        down = self.z

        roll_deg, pitch_deg, yaw_deg = self.rpy_deg
        roll      = math.radians(roll_deg)
        pitch     = math.radians(pitch_deg)
        yaw_world = wrap_pi(self.yaw_offset_rad + math.radians(yaw_deg))

        vx_body = vy_body = vz_body = 0.0
        if self.vel_body is not None and not self.is_stale(self.last_vel_stamp, now, DVL_TIMEOUT_SEC):
            vx_body, vy_body, vz_body = self.vel_body

        stamp_msg = self.last_pos_stamp.to_msg() if self.last_pos_stamp is not None else now.to_msg()

        odom = Odometry()
        odom.header.stamp      = stamp_msg
        odom.header.frame_id   = "odom"
        odom.child_frame_id    = "base_link"
        odom.pose.pose.position.x   = float(north)
        odom.pose.pose.position.y   = float(east)
        odom.pose.pose.position.z   = float(down)
        odom.pose.pose.orientation  = quat_from_rpy(roll, pitch, yaw_world)
        odom.twist.twist.linear.x   = float(vx_body)
        odom.twist.twist.linear.y   = float(vy_body)
        odom.twist.twist.linear.z   = float(vz_body)
        self.odom_pub.publish(odom)

        vel_ned = Vector3Stamped()
        vel_ned.header.stamp    = stamp_msg
        vel_ned.header.frame_id = "odom"
        vel_ned.vector.x = float(self.v_north)
        vel_ned.vector.y = float(self.v_east)
        vel_ned.vector.z = float(self.v_down)
        self.vel_ned_pub.publish(vel_ned)

        self.chi_yaw_pub.publish(Float64(data=float(self.chi_yaw)))
        self.chi_pitch_pub.publish(Float64(data=float(self.chi_pitch)))
        self.yaw_offset_pub.publish(Float64(data=float(self.yaw_offset_rad)))

    def maybe_finalize_calibration(self, now: Time) -> None:
        if self.calibration_started_at is None:
            return

        if self.reset_future is not None and not self.reset_future.done():
            return

        elapsed = (now - self.calibration_started_at).nanoseconds * 1e-9
        if elapsed < CALIBRATION_FINALIZE_SEC:
            return

        if not self.reset_response_ok:
            self.get_logger().error(
                f"Calibration aborted — DVL reset failed: {self.reset_response_message or 'unknown error'}"
            )
            self.calibration_pending = False
            return

        yaw_offset = self.estimate_heading_offset()
        if yaw_offset is None:
            self.get_logger().error("Calibration aborted — no heading samples near reset window.")
            self.calibration_pending = False
            return

        self.yaw_offset_rad   = yaw_offset
        self.yaw_offset_valid = True

        self.p_ref = sum(self.pressures) / len(self.pressures)
        self.z = 0.0
        self.vz_down = 0.0
        self.v_down  = 0.0
        self.last_z  = 0.0
        self.last_pressure_t       = now
        self.last_valid_pressure_t = now
        self.pressure_z_window.clear()

        self.pos_history.clear()
        self.v_north = 0.0
        self.v_east  = 0.0
        self.chi_yaw   = 0.0
        self.chi_pitch = 0.0
        self.have_valid_course = False

        self.calibration_pending = False
        self.get_logger().info(
            f"Calibration complete. yaw_offset={math.degrees(self.yaw_offset_rad):.2f} deg, "
            "depth zeroed."
        )

    def estimate_heading_offset(self) -> Optional[float]:
        if self.calibration_target_t_sec is None or not self.heading_history:
            return None

        target = self.calibration_target_t_sec
        window_samples = [
            math.radians(hdg)
            for (t_sec, hdg) in self.heading_history
            if abs(t_sec - target) <= HEADING_WINDOW_SEC
        ]
        if window_samples:
            return wrap_pi(circular_mean_rad(window_samples))

        # Fallback: closest sample in buffer
        t_sec, hdg = min(self.heading_history, key=lambda item: abs(item[0] - target))
        del t_sec
        return wrap_pi(math.radians(hdg))

    def get_world_xy(self) -> Tuple[float, float]:
        if self.pos is None:
            return 0.0, 0.0
        x_local, y_local, _ = self.pos
        return rotate_xy(self.yaw_offset_rad, x_local, y_local)

    def update_course_from_position(self, stamp: Time) -> None:
        if self.pos is None or self.p_ref is None or not self.yaw_offset_valid:
            return

        t_sec = stamp.nanoseconds * 1e-9
        north, east = self.get_world_xy()
        down = self.z
        self.pos_history.append((t_sec, north, east, down))

        while len(self.pos_history) >= 2 and (t_sec - self.pos_history[0][0]) > (POS_WINDOW_SEC + 0.2):
            self.pos_history.popleft()

        if len(self.pos_history) < 2:
            return

        oldest = self.pos_history[0]
        newest = self.pos_history[-1]
        dt = newest[0] - oldest[0]
        if dt <= 1e-4:
            return

        self.v_north = (newest[1] - oldest[1]) / dt
        self.v_east  = (newest[2] - oldest[2]) / dt
        self.v_down  = self.vz_down

        delta_xy = math.hypot(newest[1] - oldest[1], newest[2] - oldest[2])
        delta_z  = newest[3] - oldest[3]

        if delta_xy >= 0.01:
            self.chi_yaw   = math.atan2(self.v_east, self.v_north)
            self.chi_pitch = math.atan2(delta_z, delta_xy)
            self.have_valid_course = True
        elif not self.have_valid_course:
            self.chi_yaw   = 0.0
            self.chi_pitch = 0.0

    def stamp_or_now(self, stamp_msg) -> Time:
        if stamp_msg.sec == 0 and stamp_msg.nanosec == 0:
            return self.get_clock().now()
        return Time.from_msg(stamp_msg)

    @staticmethod
    def is_stale(last_stamp: Optional[Time], now: Time, timeout_sec: float) -> bool:
        if last_stamp is None:
            return True
        return (now - last_stamp).nanoseconds * 1e-9 > timeout_sec


def main() -> None:
    rclpy.init()
    node = Navigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
