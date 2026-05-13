import json
import socket
import threading
import time
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class DVL(Node):
    def __init__(self):
        super().__init__("dvl")

        # Parameters
        self.declare_parameter("ip", "192.168.2.95")
        self.declare_parameter("port", 16171)
        self.declare_parameter("hz", 25.0)
        self.declare_parameter("command_timeout_sec", 2.0)

        self.ip = str(self.get_parameter("ip").value)
        self.port = int(self.get_parameter("port").value)
        self.hz = float(self.get_parameter("hz").value) or 10.0
        self.command_timeout_sec = float(self.get_parameter("command_timeout_sec").value) or 2.0
        if self.hz <= 0.0:
            self.hz = 10.0
        if self.command_timeout_sec <= 0.0:
            self.command_timeout_sec = 2.0

        # Publishers
        self.pub_vel = self.create_publisher(Vector3Stamped, "/dvl/raw/vel", 10)
        self.pub_pos = self.create_publisher(Vector3Stamped, "/dvl/raw/pos", 10)
        self.pub_rpy = self.create_publisher(Vector3Stamped, "/dvl/raw/rpy", 10)
        self.pub_valid = self.create_publisher(Bool, "/dvl/raw/valid", 10)

        # Services
        self.create_service(Trigger, "/dvl/reset_dead_reckoning", self.on_reset_dead_reckoning)
        self.create_service(Trigger, "/dvl/calibrate_gyro", self.on_calibrate_gyro)

        # Hold-last values
        self._lock = threading.Lock()
        self._last = {
            "vel": None,
            "pos": None,
            "rpy": None,
            "valid": False,
        }

        # TCP connection state
        self._sock_lock = threading.Lock()
        self._sock: Optional[socket.socket] = None

        # Pending command response tracking
        self._pending_lock = threading.Lock()
        self._pending: dict[str, dict[str, Any]] = {}

        # TCP worker thread
        self._stop = False
        threading.Thread(target=self._worker, daemon=True).start()

        # Publish timer
        self.create_timer(1.0 / self.hz, self._publish_fixed_rate)

        self.get_logger().info(
            f"DVL: TCP {self.ip}:{self.port} -> publish {self.hz:.1f} Hz, "
            f"command timeout {self.command_timeout_sec:.1f} s"
        )

    def destroy_node(self):
        self._stop = True
        self._close_socket()
        self._fail_pending_commands("node shutting down")
        super().destroy_node()

    # -------------------------
    # Service callbacks
    # -------------------------

    def on_reset_dead_reckoning(self, request, response):
        del request
        ok, message = self._send_command_and_wait("reset_dead_reckoning")
        response.success = ok
        response.message = message
        if ok:
            self._clear_last_samples_after_reset()
            self.get_logger().info("DVL dead reckoning reset accepted.")
        else:
            self.get_logger().warning(f"DVL dead reckoning reset failed: {message}")
        return response

    def on_calibrate_gyro(self, request, response):
        del request
        ok, message = self._send_command_and_wait("calibrate_gyro")
        response.success = ok
        response.message = message
        if ok:
            self.get_logger().info("DVL gyro calibration accepted.")
        else:
            self.get_logger().warning(f"DVL gyro calibration failed: {message}")
        return response

    # -------------------------
    # Update hold-last from JSON
    # -------------------------

    def _set_last_vec_if_present(self, d: dict, keys: tuple[str, str, str], name: str):
        if not all(k in d for k in keys):
            return
        try:
            a = float(d[keys[0]])
            b = float(d[keys[1]])
            c = float(d[keys[2]])
        except Exception:
            return
        with self._lock:
            self._last[name] = (a, b, c)

    def _set_last_valid_if_present(self, d: dict):
        v = d.get("velocity_valid", d.get("valid", None))
        if isinstance(v, bool):
            with self._lock:
                self._last["valid"] = v

    def _handle(self, d: dict):
        self._set_last_vec_if_present(d, ("vx", "vy", "vz"), "vel")
        self._set_last_vec_if_present(d, ("x", "y", "z"), "pos")
        self._set_last_vec_if_present(d, ("roll", "pitch", "yaw"), "rpy")
        self._set_last_valid_if_present(d)

    def _handle_response_if_pending(self, d: dict) -> bool:
        if d.get("type") != "response":
            return False
        cmd = d.get("response_to")
        if not isinstance(cmd, str):
            return False
        with self._pending_lock:
            entry = self._pending.get(cmd)
            if entry is None:
                return True
            entry["response"] = d
            entry["event"].set()
        return True

    # -------------------------
    # Publish hold-last at fixed rate
    # -------------------------

    def _publish_vec(self, tup, pub):
        if tup is None:
            return
        m = Vector3Stamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.vector.x, m.vector.y, m.vector.z = tup
        pub.publish(m)

    def _publish_fixed_rate(self):
        with self._lock:
            vel = self._last["vel"]
            pos = self._last["pos"]
            rpy = self._last["rpy"]
            valid = bool(self._last["valid"])

        self._publish_vec(vel, self.pub_vel)
        self._publish_vec(pos, self.pub_pos)
        self._publish_vec(rpy, self.pub_rpy)

        b = Bool()
        b.data = valid
        self.pub_valid.publish(b)

    def _clear_last_samples_after_reset(self):
        with self._lock:
            self._last["vel"] = None
            self._last["pos"] = None
            self._last["rpy"] = None
            self._last["valid"] = False

    # -------------------------
    # Commands over TCP
    # -------------------------

    def _send_command_and_wait(self, command: str) -> tuple[bool, str]:
        entry = {"event": threading.Event(), "response": None}
        with self._pending_lock:
            if command in self._pending:
                return False, f"command already pending: {command}"
            self._pending[command] = entry

        payload = json.dumps({"command": command}) + "\n"

        try:
            with self._sock_lock:
                if self._sock is None:
                    raise RuntimeError("DVL TCP socket is not connected")
                self._sock.sendall(payload.encode("utf-8"))
        except Exception as e:
            with self._pending_lock:
                self._pending.pop(command, None)
            return False, f"failed sending command: {e}"

        if not entry["event"].wait(timeout=self.command_timeout_sec):
            with self._pending_lock:
                self._pending.pop(command, None)
            return False, f"timeout waiting for response to {command}"

        with self._pending_lock:
            done = self._pending.pop(command, None)

        response = entry.get("response") if done is not None else entry.get("response")
        if not isinstance(response, dict):
            return False, f"no valid response received for {command}"

        success = bool(response.get("success", False))
        if success:
            return True, f"{command} acknowledged"

        error_message = response.get("error_message", "")
        if not isinstance(error_message, str) or not error_message:
            error_message = f"{command} rejected by DVL"
        return False, error_message

    def _fail_pending_commands(self, message: str):
        with self._pending_lock:
            items = list(self._pending.items())
            self._pending.clear()
        for _cmd, entry in items:
            entry["response"] = {"success": False, "error_message": message}
            entry["event"].set()

    def _close_socket(self):
        with self._sock_lock:
            sock = self._sock
            self._sock = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                sock.close()
            except Exception:
                pass

    # -------------------------
    # TCP worker: connect, read lines, parse JSON
    # -------------------------

    def _worker(self):
        buf = ""
        while not self._stop:
            try:
                sock = socket.create_connection((self.ip, self.port), timeout=5.0)
                sock.settimeout(1.0)
                with self._sock_lock:
                    self._sock = sock
                buf = ""
                self.get_logger().info("DVL connected.")

                while not self._stop:
                    try:
                        data = sock.recv(4096)
                        if not data:
                            raise ConnectionError("connection closed")
                        buf += data.decode("utf-8", errors="ignore")
                    except socket.timeout:
                        continue

                    lines = buf.split("\n")
                    buf = lines.pop()
                    for line in lines:
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            d = json.loads(line)
                        except json.JSONDecodeError:
                            continue
                        if self._handle_response_if_pending(d):
                            continue
                        self._handle(d)

            except Exception as e:
                self.get_logger().warning(f"DVL error: {e}. Reconnecting...")
                self._close_socket()
                self._fail_pending_commands(f"connection lost: {e}")
                time.sleep(1.0)


def main():
    rclpy.init()
    node = DVL()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
