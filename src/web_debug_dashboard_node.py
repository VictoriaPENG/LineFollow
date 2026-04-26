#!/usr/bin/env python3
"""
多画面 Web 调试页。

功能：
1. 订阅多路 ROS 图像话题
2. 将每路图像编码为 MJPEG
3. 提供一个四宫格调试页面，默认根路径 `/`
4. 在有桌面环境时可尝试自动打开浏览器
"""

from __future__ import annotations

import os
import json
import threading
import time
import webbrowser
from http import server
from socketserver import ThreadingMixIn
from typing import Dict

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


class _StreamState:
    def __init__(self, label: str) -> None:
        self.label = label
        self.frame = None
        self.updated_at = 0.0
        self.cond = threading.Condition()


class _ThreadingHTTPServer(ThreadingMixIn, server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class _MotorSpeedState:
    def __init__(self) -> None:
        self.left_rpm = 0.0
        self.right_rpm = 0.0
        self.updated_at = 0.0
        self.lock = threading.Lock()


class WebDebugDashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("web_debug_dashboard_node")

        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("port", 8091)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("open_browser", True)
        self.declare_parameter("source_topic", "/image")
        self.declare_parameter("source_topic_type", "compressed")
        self.declare_parameter("detect_topic", "/line_follow/debug_detect")
        self.declare_parameter("binary_topic", "/line_follow/debug_binary")
        self.declare_parameter("curve_topic", "/line_follow/debug_angle_curve")
        self.declare_parameter("speed_status_topic", "/motor_speed_status")

        self.bind_host = str(self.get_parameter("bind_host").value)
        self.port = int(self.get_parameter("port").value)
        self.jpeg_quality = max(30, min(95, int(self.get_parameter("jpeg_quality").value)))
        self.open_browser = bool(self.get_parameter("open_browser").value)
        self.bridge = CvBridge()
        self.speed_status_topic = str(self.get_parameter("speed_status_topic").value)
        self.motor_speed = _MotorSpeedState()

        topic_map = {
            "source": (
                str(self.get_parameter("source_topic").value),
                "Camera",
                str(self.get_parameter("source_topic_type").value).strip().lower(),
            ),
            "detect": (str(self.get_parameter("detect_topic").value), "Detect", "raw"),
            "binary": (str(self.get_parameter("binary_topic").value), "Binary", "raw"),
            "curve": (str(self.get_parameter("curve_topic").value), "Angle Curve", "raw"),
        }
        self.streams: Dict[str, _StreamState] = {
            name: _StreamState(label) for name, (_, label, _) in topic_map.items()
        }
        self._subscriptions = []

        for name, (topic, _, topic_type) in topic_map.items():
            if topic_type == "compressed":
                self._subscriptions.append(
                    self.create_subscription(
                        CompressedImage,
                        topic,
                        self._make_compressed_image_callback(name),
                        qos_profile_sensor_data,
                    )
                )
            else:
                self._subscriptions.append(
                    self.create_subscription(
                        Image,
                        topic,
                        self._make_image_callback(name),
                        qos_profile_sensor_data,
                    )
                )
        self._subscriptions.append(
            self.create_subscription(
                Float32MultiArray,
                self.speed_status_topic,
                self._on_speed_status,
                10,
            )
        )

        self.httpd = _ThreadingHTTPServer((self.bind_host, self.port), self._make_handler())
        self.server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.server_thread.start()

        url = f"http://127.0.0.1:{self.port}/"
        self.get_logger().info(
            f"web debug dashboard ready on {self.bind_host}:{self.port}, open {url}"
        )
        if self.open_browser and (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            self.create_timer(1.0, self._open_browser_once)
            self._browser_opened = False
        else:
            self._browser_opened = True

    def _open_browser_once(self) -> None:
        if self._browser_opened:
            return
        self._browser_opened = True
        try:
            webbrowser.open(f"http://127.0.0.1:{self.port}/", new=1, autoraise=True)
        except Exception as exc:
            self.get_logger().warn(f"open browser failed: {exc}")

    def _make_image_callback(self, stream_name: str):
        def _callback(msg: Image) -> None:
            try:
                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                ok, encoded = cv2.imencode(
                    ".jpg",
                    image,
                    [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
                )
                if not ok:
                    return
                state = self.streams[stream_name]
                with state.cond:
                    state.frame = encoded.tobytes()
                    state.updated_at = time.time()
                    state.cond.notify_all()
            except Exception as exc:
                self.get_logger().warn(f"convert image failed for {stream_name}: {exc}")

        return _callback

    def _make_compressed_image_callback(self, stream_name: str):
        def _callback(msg: CompressedImage) -> None:
            try:
                state = self.streams[stream_name]
                with state.cond:
                    state.frame = bytes(msg.data)
                    state.updated_at = time.time()
                    state.cond.notify_all()
            except Exception as exc:
                self.get_logger().warn(f"store compressed image failed for {stream_name}: {exc}")

        return _callback

    def _on_speed_status(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            return
        with self.motor_speed.lock:
            self.motor_speed.left_rpm = float(msg.data[0])
            self.motor_speed.right_rpm = float(msg.data[1])
            self.motor_speed.updated_at = time.time()

    def _status_json(self) -> bytes:
        now = time.time()
        with self.motor_speed.lock:
            updated_at = self.motor_speed.updated_at
            body = {
                "left_rpm": self.motor_speed.left_rpm,
                "right_rpm": self.motor_speed.right_rpm,
                "updated_at": updated_at,
                "age_sec": None if updated_at <= 0.0 else max(0.0, now - updated_at),
                "stale": updated_at <= 0.0 or (now - updated_at) > 1.0,
                "speed_status_topic": self.speed_status_topic,
            }
        return json.dumps(body, ensure_ascii=False).encode("utf-8")

    def _html(self) -> bytes:
        cards = []
        for stream_name, state in self.streams.items():
            cards.append(
                f"""
                <section class="card">
                  <header>
                    <span>{state.label}</span>
                    <code>/stream/{stream_name}.mjpg</code>
                  </header>
                  <img src="/stream/{stream_name}.mjpg" alt="{state.label}" />
                </section>
                """
            )
        html = f"""
        <!doctype html>
        <html lang="zh-CN">
        <head>
          <meta charset="utf-8" />
          <meta name="viewport" content="width=device-width, initial-scale=1" />
          <title>Line Follow Web Debug</title>
          <style>
            :root {{
              --bg: #0f172a;
              --panel: #111827;
              --line: #334155;
              --text: #e5e7eb;
              --muted: #94a3b8;
              --accent: #22c55e;
            }}
            * {{ box-sizing: border-box; }}
            body {{
              margin: 0;
              font-family: "Noto Sans SC", "Microsoft YaHei", sans-serif;
              background:
                radial-gradient(circle at top left, rgba(34,197,94,0.16), transparent 28%),
                linear-gradient(135deg, #020617 0%, var(--bg) 42%, #111827 100%);
              color: var(--text);
            }}
            .wrap {{
              min-height: 100vh;
              padding: 16px;
            }}
            .title {{
              display: flex;
              justify-content: space-between;
              align-items: center;
              gap: 12px;
              margin-bottom: 16px;
              padding: 14px 18px;
              border: 1px solid rgba(148,163,184,0.2);
              border-radius: 18px;
              background: rgba(15,23,42,0.72);
              backdrop-filter: blur(10px);
            }}
            .title h1 {{
              margin: 0;
              font-size: 22px;
            }}
            .title span {{
              color: var(--muted);
              font-size: 13px;
            }}
            .status {{
              display: grid;
              grid-template-columns: repeat(3, minmax(0, 1fr));
              gap: 12px;
              margin-bottom: 16px;
            }}
            .metric {{
              border: 1px solid rgba(148,163,184,0.18);
              border-radius: 8px;
              padding: 12px 14px;
              background: rgba(17,24,39,0.86);
            }}
            .metric label {{
              display: block;
              color: var(--muted);
              font-size: 12px;
              margin-bottom: 6px;
            }}
            .metric strong {{
              display: block;
              font-size: 26px;
              line-height: 1.1;
              font-variant-numeric: tabular-nums;
            }}
            .metric small {{
              color: var(--muted);
              font-size: 12px;
            }}
            .metric.stale strong {{
              color: #f97316;
            }}
            .grid {{
              display: grid;
              grid-template-columns: repeat(2, minmax(0, 1fr));
              gap: 16px;
            }}
            .card {{
              border: 1px solid rgba(148,163,184,0.18);
              border-radius: 18px;
              overflow: hidden;
              background: rgba(17,24,39,0.85);
              box-shadow: 0 18px 60px rgba(2,6,23,0.28);
            }}
            .card header {{
              display: flex;
              justify-content: space-between;
              align-items: center;
              gap: 12px;
              padding: 10px 14px;
              background: rgba(15,23,42,0.88);
              border-bottom: 1px solid rgba(148,163,184,0.14);
            }}
            .card header span {{
              color: var(--text);
              font-size: 14px;
              font-weight: 700;
            }}
            .card header code {{
              color: var(--accent);
              font-size: 12px;
            }}
            .card img {{
              display: block;
              width: 100%;
              aspect-ratio: 16 / 9;
              object-fit: contain;
              background: #000;
            }}
            @media (max-width: 900px) {{
              .status {{
                grid-template-columns: 1fr;
              }}
              .grid {{
                grid-template-columns: 1fr;
              }}
            }}
          </style>
        </head>
        <body>
          <div class="wrap">
            <div class="title">
              <div>
                <h1>Line Follow Multi-View Debug</h1>
                <span>默认四宫格调试页</span>
              </div>
              <span>刷新页面即可重连流</span>
            </div>
            <section class="status">
              <div class="metric" id="left-speed">
                <label>Left Motor</label>
                <strong>--</strong>
                <small>RPM</small>
              </div>
              <div class="metric" id="right-speed">
                <label>Right Motor</label>
                <strong>--</strong>
                <small>RPM</small>
              </div>
              <div class="metric" id="speed-state">
                <label>Motor Status</label>
                <strong>Waiting</strong>
                <small>{self.speed_status_topic}</small>
              </div>
            </section>
            <div class="grid">
              {''.join(cards)}
            </div>
          </div>
          <script>
            const leftSpeed = document.querySelector("#left-speed");
            const rightSpeed = document.querySelector("#right-speed");
            const speedState = document.querySelector("#speed-state");

            function setMetric(card, value, stale) {{
              card.classList.toggle("stale", stale);
              card.querySelector("strong").textContent = value;
            }}

            async function refreshStatus() {{
              try {{
                const response = await fetch("/api/status", {{ cache: "no-store" }});
                const data = await response.json();
                const stale = Boolean(data.stale);
                setMetric(leftSpeed, stale ? "--" : Number(data.left_rpm).toFixed(0), stale);
                setMetric(rightSpeed, stale ? "--" : Number(data.right_rpm).toFixed(0), stale);
                speedState.classList.toggle("stale", stale);
                speedState.querySelector("strong").textContent = stale ? "No Data" : "Live";
              }} catch (error) {{
                setMetric(leftSpeed, "--", true);
                setMetric(rightSpeed, "--", true);
                speedState.classList.add("stale");
                speedState.querySelector("strong").textContent = "Offline";
              }}
            }}

            refreshStatus();
            setInterval(refreshStatus, 250);
          </script>
        </body>
        </html>
        """
        return html.encode("utf-8")

    def _make_handler(self):
        node = self

        class Handler(server.BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    body = node._html()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if self.path == "/api/status":
                    body = node._status_json()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if self.path.startswith("/stream/") and self.path.endswith(".mjpg"):
                    stream_name = self.path[len("/stream/") : -len(".mjpg")]
                    state = node.streams.get(stream_name)
                    if state is None:
                        self.send_error(404, "stream not found")
                        return

                    self.send_response(200)
                    self.send_header("Age", "0")
                    self.send_header("Cache-Control", "no-cache, private")
                    self.send_header("Pragma", "no-cache")
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()

                    last_seen = 0.0
                    while rclpy.ok():
                        with state.cond:
                            if state.frame is None or state.updated_at <= last_seen:
                                state.cond.wait(timeout=1.0)
                            frame = state.frame
                            updated_at = state.updated_at

                        if frame is None or updated_at <= last_seen:
                            continue

                        last_seen = updated_at
                        try:
                            self.wfile.write(b"--frame\r\n")
                            self.send_header("Content-Type", "image/jpeg")
                            self.send_header("Content-Length", str(len(frame)))
                            self.end_headers()
                            self.wfile.write(frame)
                            self.wfile.write(b"\r\n")
                        except BrokenPipeError:
                            break
                        except ConnectionResetError:
                            break
                    return

                self.send_error(404, "not found")

            def log_message(self, format, *args):
                return

        return Handler

    def destroy_node(self):
        try:
            self.httpd.shutdown()
            self.httpd.server_close()
        except Exception:
            pass
        return super().destroy_node()


def main() -> int:
    rclpy.init()
    node = WebDebugDashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
