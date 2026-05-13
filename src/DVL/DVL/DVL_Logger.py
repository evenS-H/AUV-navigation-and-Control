import csv
import os
import threading
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Bool


class DVLLogger(Node):
    """
    Subscribes to DVL raw topics and logs them to a CSV file.
    One row is written each time a velocity message arrives.
    """

    def __init__(self):
        super().__init__("dvl_logger")

        self.declare_parameter("log_dir", "/tmp")
        self.declare_parameter("hz", 25.0)

        log_dir = str(self.get_parameter("log_dir").value)
        hz = float(self.get_parameter("hz").value)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._csv_path = os.path.join(log_dir, f"dvl_log_{timestamp}.csv")

        self._lock = threading.Lock()
        self._latest = {
            "vel": (0.0, 0.0, 0.0),
            "pos": (0.0, 0.0, 0.0),
            "rpy": (0.0, 0.0, 0.0),
            "valid": False,
        }

        self._file = open(self._csv_path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            "time_sec",
            "vx", "vy", "vz",
            "x", "y", "z",
            "roll", "pitch", "yaw",
            "valid",
        ])

        self.create_subscription(Vector3Stamped, "/dvl/raw/vel", self._on_vel, 10)
        self.create_subscription(Vector3Stamped, "/dvl/raw/pos", self._on_pos, 10)
        self.create_subscription(Vector3Stamped, "/dvl/raw/rpy", self._on_rpy, 10)
        self.create_subscription(Bool, "/dvl/raw/valid", self._on_valid, 10)

        self.create_timer(1.0 / hz, self._write_row)

        self.get_logger().info(f"DVL Logger writing to {self._csv_path}")

    def _on_vel(self, msg: Vector3Stamped):
        with self._lock:
            self._latest["vel"] = (msg.vector.x, msg.vector.y, msg.vector.z)

    def _on_pos(self, msg: Vector3Stamped):
        with self._lock:
            self._latest["pos"] = (msg.vector.x, msg.vector.y, msg.vector.z)

    def _on_rpy(self, msg: Vector3Stamped):
        with self._lock:
            self._latest["rpy"] = (msg.vector.x, msg.vector.y, msg.vector.z)

    def _on_valid(self, msg: Bool):
        with self._lock:
            self._latest["valid"] = bool(msg.data)

    def _write_row(self):
        with self._lock:
            vel = self._latest["vel"]
            pos = self._latest["pos"]
            rpy = self._latest["rpy"]
            valid = self._latest["valid"]

        t = time.time()
        self._writer.writerow([
            f"{t:.4f}",
            f"{vel[0]:.4f}", f"{vel[1]:.4f}", f"{vel[2]:.4f}",
            f"{pos[0]:.4f}", f"{pos[1]:.4f}", f"{pos[2]:.4f}",
            f"{rpy[0]:.4f}", f"{rpy[1]:.4f}", f"{rpy[2]:.4f}",
            int(valid),
        ])
        self._file.flush()

    def destroy_node(self):
        try:
            self._file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = DVLLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
