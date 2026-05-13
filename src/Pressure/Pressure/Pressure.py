#!/usr/bin/env python3
import site
site.addsitedir('/root/persistent_ws/ws/pydeps')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from pymavlink import mavutil


class Pressure(Node):
    def __init__(self):
        super().__init__("pressure")

        self.declare_parameter("connection_url", "tcp:127.0.0.1:5777")
        self.declare_parameter("pressure_topic", "/bar30/pressure")
        self.declare_parameter("frame_id", "bar30_link")

        self.connection_url = str(self.get_parameter("connection_url").value)
        self.pressure_topic = str(self.get_parameter("pressure_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        # Request SCALED_PRESSURE2 at 25 Hz
        self.message_rate_hz = 25.0
        self.interval_us = int(1e6 / self.message_rate_hz)

        self.pub_pressure = self.create_publisher(
            FluidPressure,
            self.pressure_topic,
            10,
        )

        self.get_logger().info(f"Connecting to MAVLink at {self.connection_url}")
        self.master = mavutil.mavlink_connection(self.connection_url)

        try:
            self.master.wait_heartbeat(timeout=10)
            self.get_logger().info("MAVLink heartbeat received.")
        except Exception as e:
            self.get_logger().error(f"No heartbeat received: {e}")
            raise

        # Request SCALED_PRESSURE2 (message ID 137) at 25 Hz
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            137,                  # SCALED_PRESSURE2
            self.interval_us,
            0, 0, 0, 0, 0,
        )

        self.get_logger().info(
            f"Requested SCALED_PRESSURE2 at {self.message_rate_hz:.1f} Hz. "
            f"Publishing on {self.pressure_topic}."
        )

        # Poll faster than the publish rate to catch every MAVLink frame
        self.create_timer(0.01, self.poll_mavlink)

    def poll_mavlink(self):
        msg = self.master.recv_match(type="SCALED_PRESSURE2", blocking=False)
        if msg is None:
            return

        out = FluidPressure()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id

        # MAVLink press_abs is in hPa; ROS FluidPressure is in Pa
        out.fluid_pressure = float(msg.press_abs) * 100.0
        out.variance = 0.0

        self.pub_pressure.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Pressure()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
