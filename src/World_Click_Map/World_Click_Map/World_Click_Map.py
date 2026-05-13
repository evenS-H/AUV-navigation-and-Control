#!/usr/bin/env python3
import json
import os
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

try:
    import pymap3d as pm
except ModuleNotFoundError:
    extra = "/root/persistent_ws/ws/pydeps"
    if os.path.isdir(extra) and extra not in sys.path:
        sys.path.insert(0, extra)
    import pymap3d as pm


HTML_PAGE = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>ROS2 AUV Route Planner (NED + Depth)</title>

  <link
    rel="stylesheet"
    href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
    integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY="
    crossorigin=""
  />
  <style>
    html, body { height: 100%; margin: 0; }
    #map { height: 100%; width: 100%; }

    .panel {
      position: absolute; z-index: 999; top: 12px; left: 12px;
      background: rgba(255,255,255,0.95); border-radius: 12px;
      padding: 10px 12px;
      font-family: system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif;
      box-shadow: 0 6px 20px rgba(0,0,0,0.15);
      max-width: 420px;
    }

    .row { display: flex; gap: 8px; margin-top: 8px; flex-wrap: wrap; align-items: center; }

    button {
      padding: 8px 10px; border: 0; border-radius: 10px; cursor: pointer; font-size: 14px;
    }
    button.primary { background: #111; color: #fff; }
    button.secondary { background: #eee; }

    input {
      padding: 8px;
      border: 1px solid #ddd;
      border-radius: 10px;
      font-size: 14px;
    }

    .small { font-size: 12px; color: #555; margin-top: 8px; line-height: 1.35; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace; }

    .badge {
      display: inline-block; padding: 2px 8px; border-radius: 999px;
      background: #f0f0f0; font-size: 12px; margin-left: 6px;
    }
  </style>
</head>

<body>
  <div id="map"></div>

  <div class="panel">
    <div>
      AUV Route Planner (NED + Depth)
      <span class="badge" id="count">0 pts</span>
    </div>

    <div class="small">
      Click to add waypoints (uses current depth). Drag markers to adjust position.
      Use "Set depth for last" to modify the last waypoint's depth.
    </div>

    <div class="small">
      Published ROS convention is <b>NED</b>: x = North, y = East, z = Down.
    </div>

    <div class="row">
      <button class="primary" id="send">Send route to ROS 2</button>
      <button class="secondary" id="undo">Undo last</button>
      <button class="secondary" id="clear">Clear</button>
    </div>

    <div class="row">
      <input id="depth" type="number" step="0.1" min="0" value="0" style="width: 170px;"
             placeholder="Depth (m, +down)" />
      <button class="secondary" id="setDepth">Set depth for last</button>
    </div>

    <div class="small mono" id="last">Last: —</div>
  </div>

  <script
    src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
    integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo="
    crossorigin=""
  ></script>

  <script>
    const map = L.map("map", { worldCopyJump: true }).setView([20, 0], 2);

    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      maxZoom: 19,
      attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);

    const countEl = document.getElementById("count");
    const lastEl = document.getElementById("last");
    const sendBtn = document.getElementById("send");
    const undoBtn = document.getElementById("undo");
    const clearBtn = document.getElementById("clear");

    const depthEl = document.getElementById("depth");
    const setDepthBtn = document.getElementById("setDepth");

    const waypoints = [];
    const markers = [];
    let line = L.polyline([], {}).addTo(map);

    function fmt(n) { return Number(n).toFixed(6); }

    function updateUI() {
      countEl.textContent = `${waypoints.length} pts`;
      if (waypoints.length) {
        const p = waypoints[waypoints.length - 1];
        lastEl.textContent = `Last: lat ${fmt(p.lat)}, lon ${fmt(p.lon)}, depth ${Number(p.depth).toFixed(1)} m`;
      } else {
        lastEl.textContent = "Last: —";
      }
    }

    function redrawLine() {
      line.setLatLngs(waypoints.map(p => [p.lat, p.lon]));
    }

    function renumberPopups() {
      markers.forEach((m, i) => {
        const p = waypoints[i];
        m.setPopupContent(
          `<b>WP ${i+1}</b><br>
           lat ${fmt(p.lat)}<br>
           lon ${fmt(p.lon)}<br>
           depth ${Number(p.depth).toFixed(1)} m<br>
           <small>(drag to adjust position)</small>`
        );
      });
    }

    function addWaypoint(lat, lon, depth=0.0) {
      waypoints.push({ lat, lon, depth });

      const idx = waypoints.length - 1;
      const marker = L.marker([lat, lon], { draggable: true }).addTo(map);

      marker.on("drag", (e) => {
        const ll = e.target.getLatLng();
        waypoints[idx].lat = ll.lat;
        waypoints[idx].lon = ll.lng;
        redrawLine();
        updateUI();
      });

      marker.on("dragend", () => renumberPopups());

      markers.push(marker);
      redrawLine();
      renumberPopups();
      marker.openPopup();
      updateUI();
    }

    map.on("click", (e) => {
      const depth = Number(depthEl.value);
      addWaypoint(
        e.latlng.lat,
        e.latlng.lng,
        Number.isFinite(depth) && depth >= 0 ? depth : 0.0
      );
    });

    setDepthBtn.addEventListener("click", () => {
      if (!waypoints.length) return;
      const depth = Number(depthEl.value);
      if (!Number.isFinite(depth) || depth < 0) {
        alert("Depth must be a number >= 0");
        return;
      }
      waypoints[waypoints.length - 1].depth = depth;
      renumberPopups();
      updateUI();
    });

    undoBtn.addEventListener("click", () => {
      if (!waypoints.length) return;
      waypoints.pop();
      const m = markers.pop();
      map.removeLayer(m);
      redrawLine();
      renumberPopups();
      updateUI();
    });

    clearBtn.addEventListener("click", () => {
      waypoints.splice(0, waypoints.length);
      while (markers.length) map.removeLayer(markers.pop());
      redrawLine();
      updateUI();
    });

    async function sendRoute() {
      if (waypoints.length < 2) {
        alert("Add at least 2 waypoints.");
        return;
      }

      const payload = {
        frame_id: "wgs84",
        waypoints: waypoints.map(p => ({ lat: p.lat, lon: p.lon, depth: p.depth }))
      };

      const resp = await fetch("/route", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });

      if (!resp.ok) {
        const txt = await resp.text();
        alert("Server error: " + txt);
        return;
      }

      alert("Route sent to ROS 2!");
    }

    sendBtn.addEventListener("click", sendRoute);

    updateUI();
  </script>
</body>
</html>
"""


class RouteHTTPHandler(BaseHTTPRequestHandler):
    def _send(self, status: int, body: bytes, content_type: str = "text/plain; charset=utf-8"):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            self._send(200, HTML_PAGE.encode("utf-8"), "text/html; charset=utf-8")
            return
        self._send(404, b"Not found")

    def do_POST(self):
        path = urlparse(self.path).path
        if path != "/route":
            self._send(404, b"Not found")
            return

        length = int(self.headers.get("Content-Length", "0"))
        raw    = self.rfile.read(length)

        try:
            data   = json.loads(raw.decode("utf-8"))
            wps    = data["waypoints"]
            points = [(float(w["lat"]), float(w["lon"]), float(w.get("depth", 0.0))) for w in wps]
        except Exception as e:
            self._send(400, f"Bad JSON: {e}".encode("utf-8"))
            return

        if len(points) < 2:
            self._send(422, b"Need at least 2 waypoints")
            return

        for lat, lon, depth in points:
            if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                self._send(422, b"lat/lon out of range")
                return
            if depth < 0:
                self._send(422, b"depth must be >= 0 (meters, positive down)")
                return

        node: "World_Click_Map" = self.server.node  # type: ignore[attr-defined]
        node.publish_route_ned(points)
        self._send(200, b"ok")


class World_Click_Map(Node):
    def __init__(self):
        super().__init__("world_click_map")

        self.declare_parameter("host",               "0.0.0.0")
        self.declare_parameter("port",               8000)
        self.declare_parameter("ui_host",            "192.168.2.2")
        self.declare_parameter("origin_is_fixed",    False)
        self.declare_parameter("origin_lat",         0.0)
        self.declare_parameter("origin_lon",         0.0)
        self.declare_parameter("origin_alt",         0.0)
        self.declare_parameter("frame_id",           "odom")

        self._origin_set = False
        self._lat0 = 0.0
        self._lon0 = 0.0
        self._alt0 = 0.0

        self.pub_path_ned = self.create_publisher(Path,   "/auv_route_ned",      10)
        self.pub_info     = self.create_publisher(String, "/auv_route_ned_info", 10)

        host     = str(self.get_parameter("host").value)
        port     = int(self.get_parameter("port").value)
        frame_id = str(self.get_parameter("frame_id").value)

        self.get_logger().info(f"HTTP UI: http://{host}:{port}")
        self.get_logger().info(
            f"Publishes /auv_route_ned as nav_msgs/Path in frame '{frame_id}' "
            "(x=North, y=East, z=Down meters)"
        )

        self.httpd = ThreadingHTTPServer((host, port), RouteHTTPHandler)
        self.httpd.node = self
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()

        ui_host = str(self.get_parameter("ui_host").value)
        self.get_logger().info(f"Open map at: http://{ui_host}:{port}")

    def _ensure_origin(self, first_lat: float, first_lon: float):
        if self._origin_set:
            return

        if bool(self.get_parameter("origin_is_fixed").value):
            self._lat0 = float(self.get_parameter("origin_lat").value)
            self._lon0 = float(self.get_parameter("origin_lon").value)
            self._alt0 = float(self.get_parameter("origin_alt").value)
            self.get_logger().info(
                f"Fixed origin: lat={self._lat0:.8f}, lon={self._lon0:.8f}, alt={self._alt0:.2f} m"
            )
        else:
            self._lat0 = float(first_lat)
            self._lon0 = float(first_lon)
            self._alt0 = 0.0
            self.get_logger().info(
                f"First-waypoint origin: lat={self._lat0:.8f}, lon={self._lon0:.8f}"
            )

        self._origin_set = True

        frame_id = str(self.get_parameter("frame_id").value)
        info = {
            "origin": {"lat": self._lat0, "lon": self._lon0, "alt": self._alt0},
            "frame":  frame_id,
            "convention": "x=north(m), y=east(m), z=down(m)",
        }
        self.pub_info.publish(String(data=json.dumps(info)))

    def publish_route_ned(self, wgs_points):
        """
        wgs_points: list of (lat, lon, depth) where depth is meters, positive down.
        Publishes nav_msgs/Path in NED convention (x=North, y=East, z=Down).
        """
        first_lat, first_lon, _ = wgs_points[0]
        self._ensure_origin(first_lat, first_lon)

        now      = self.get_clock().now().to_msg()
        frame_id = str(self.get_parameter("frame_id").value)

        path = Path()
        path.header.stamp    = now
        path.header.frame_id = frame_id

        ned_list = []
        for lat, lon, depth in wgs_points:
            n, e, _ = pm.geodetic2ned(
                lat, lon, 0.0,
                self._lat0, self._lon0, 0.0,
            )
            d = float(depth)

            ps = PoseStamped()
            ps.header.stamp    = now
            ps.header.frame_id = frame_id
            ps.pose.position.x = float(n)
            ps.pose.position.y = float(e)
            ps.pose.position.z = d
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

            ned_list.append((float(n), float(e), d))

        self.pub_path_ned.publish(path)

        self.get_logger().info(
            f"Published NED route: {len(wgs_points)} waypoints | "
            f"origin(lat={self._lat0:.6f}, lon={self._lon0:.6f})"
        )

        dbg = {
            "origin": {"lat": self._lat0, "lon": self._lon0, "alt": self._alt0},
            "wgs84_waypoints": [
                {"lat": lat, "lon": lon, "depth": depth}
                for (lat, lon, depth) in wgs_points
            ],
            "ned_waypoints": [
                {"north": n, "east": e, "down": d}
                for (n, e, d) in ned_list
            ],
            "frame": frame_id,
            "convention": "x=north(m), y=east(m), z=down(m)",
        }
        self.pub_info.publish(String(data=json.dumps(dbg)))

    def destroy_node(self):
        try:
            self.httpd.shutdown()
            self.httpd.server_close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = World_Click_Map()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
