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
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray


class _StreamState:
    """保存单路视频流的最新帧及等待条件。"""

    def __init__(self, label: str) -> None:
        self.label = label
        self.frame = None
        self.updated_at = 0.0
        self.cond = threading.Condition()


class _ThreadingHTTPServer(ThreadingMixIn, server.HTTPServer):
    """支持并发请求的 HTTP 服务，供多路 MJPEG 和状态接口复用。"""

    daemon_threads = True
    allow_reuse_address = True


class _MotorSpeedState:
    """缓存最近一次左右电机转速，供网页端读取。"""

    def __init__(self) -> None:
        self.left_rpm = 0.0
        self.right_rpm = 0.0
        self.updated_at = 0.0
        self.lock = threading.Lock()


class _ScalarState:
    """缓存最近一次标量状态，供网页端读取。"""

    def __init__(self, default=0.0) -> None:
        self.value = default
        self.updated_at = 0.0
        self.lock = threading.Lock()


class _CmdVelState:
    """缓存最近一次 cmd_vel，供网页端读取。"""

    def __init__(self) -> None:
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.updated_at = 0.0
        self.lock = threading.Lock()


class WebDebugDashboardNode(Node):
    """提供图像流、速度状态和四宫格调试页面的 Web 节点。"""

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
        self.declare_parameter("speed_cmd_topic", "/motor_speed_cmd")
        self.declare_parameter("speed_status_topic", "/motor_speed_status")
        self.declare_parameter("line_detected_topic", "/line_follow/line_detected")
        self.declare_parameter("heading_error_topic", "/line_follow/visual_heading_error_deg")
        self.declare_parameter("lateral_error_topic", "/line_follow/visual_lateral_error_norm")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.bind_host = str(self.get_parameter("bind_host").value)
        self.port = int(self.get_parameter("port").value)
        self.jpeg_quality = max(30, min(95, int(self.get_parameter("jpeg_quality").value)))
        self.open_browser = bool(self.get_parameter("open_browser").value)
        self.bridge = CvBridge()
        self.speed_cmd_topic = str(self.get_parameter("speed_cmd_topic").value)
        self.speed_status_topic = str(self.get_parameter("speed_status_topic").value)
        self.line_detected_topic = str(self.get_parameter("line_detected_topic").value)
        self.heading_error_topic = str(self.get_parameter("heading_error_topic").value)
        self.lateral_error_topic = str(self.get_parameter("lateral_error_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.cmd_speed = _MotorSpeedState()
        self.status_speed = _MotorSpeedState()
        self.line_detected = _ScalarState(False)
        self.heading_error = _ScalarState(0.0)
        self.lateral_error = _ScalarState(0.0)
        self.cmd_vel = _CmdVelState()

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
                self.speed_cmd_topic,
                self._on_speed_cmd,
                10,
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
        self._subscriptions.append(
            self.create_subscription(Bool, self.line_detected_topic, self._on_line_detected, 10)
        )
        self._subscriptions.append(
            self.create_subscription(Float32, self.heading_error_topic, self._on_heading_error, 10)
        )
        self._subscriptions.append(
            self.create_subscription(Float32, self.lateral_error_topic, self._on_lateral_error, 10)
        )
        self._subscriptions.append(
            self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)
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

    def _update_motor_speed(self, state: _MotorSpeedState, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            return
        with state.lock:
            state.left_rpm = float(msg.data[0])
            state.right_rpm = float(msg.data[1])
            state.updated_at = time.time()

    def _on_speed_cmd(self, msg: Float32MultiArray) -> None:
        self._update_motor_speed(self.cmd_speed, msg)

    def _on_speed_status(self, msg: Float32MultiArray) -> None:
        self._update_motor_speed(self.status_speed, msg)

    def _update_scalar(self, state: _ScalarState, value) -> None:
        with state.lock:
            state.value = value
            state.updated_at = time.time()

    def _on_line_detected(self, msg: Bool) -> None:
        self._update_scalar(self.line_detected, bool(msg.data))

    def _on_heading_error(self, msg: Float32) -> None:
        self._update_scalar(self.heading_error, float(msg.data))

    def _on_lateral_error(self, msg: Float32) -> None:
        self._update_scalar(self.lateral_error, float(msg.data))

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self.cmd_vel.lock:
            self.cmd_vel.linear_x = float(msg.linear.x)
            self.cmd_vel.angular_z = float(msg.angular.z)
            self.cmd_vel.updated_at = time.time()

    def _speed_snapshot(self, state: _MotorSpeedState, now: float) -> dict:
        with state.lock:
            updated_at = state.updated_at
            return {
                "left_rpm": state.left_rpm,
                "right_rpm": state.right_rpm,
                "updated_at": updated_at,
                "age_sec": None if updated_at <= 0.0 else max(0.0, now - updated_at),
                "stale": updated_at <= 0.0 or (now - updated_at) > 1.0,
            }

    def _scalar_snapshot(self, state: _ScalarState, now: float) -> dict:
        with state.lock:
            updated_at = state.updated_at
            return {
                "value": state.value,
                "updated_at": updated_at,
                "age_sec": None if updated_at <= 0.0 else max(0.0, now - updated_at),
                "stale": updated_at <= 0.0 or (now - updated_at) > 1.0,
            }

    def _cmd_vel_snapshot(self, state: _CmdVelState, now: float) -> dict:
        with state.lock:
            updated_at = state.updated_at
            return {
                "linear_x": state.linear_x,
                "angular_z": state.angular_z,
                "updated_at": updated_at,
                "age_sec": None if updated_at <= 0.0 else max(0.0, now - updated_at),
                "stale": updated_at <= 0.0 or (now - updated_at) > 1.0,
            }

    def _status_json(self) -> bytes:
        now = time.time()
        body = {
            "cmd": self._speed_snapshot(self.cmd_speed, now),
            "status": self._speed_snapshot(self.status_speed, now),
            "line_detected": self._scalar_snapshot(self.line_detected, now),
            "heading_error": self._scalar_snapshot(self.heading_error, now),
            "lateral_error": self._scalar_snapshot(self.lateral_error, now),
            "cmd_vel": self._cmd_vel_snapshot(self.cmd_vel, now),
            "speed_cmd_topic": self.speed_cmd_topic,
            "speed_status_topic": self.speed_status_topic,
            "line_detected_topic": self.line_detected_topic,
            "heading_error_topic": self.heading_error_topic,
            "lateral_error_topic": self.lateral_error_topic,
            "cmd_vel_topic": self.cmd_vel_topic,
        }
        body.update(body["status"])
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
              grid-template-columns: repeat(2, minmax(0, 1fr));
              gap: 12px;
              margin-bottom: 16px;
            }}
            .speed-panel {{
              border: 1px solid rgba(148,163,184,0.18);
              border-radius: 8px;
              padding: 12px 14px;
              background: rgba(17,24,39,0.86);
            }}
            .speed-panel header {{
              display: flex;
              justify-content: space-between;
              align-items: center;
              gap: 12px;
              margin-bottom: 10px;
            }}
            .speed-panel header strong {{
              font-size: 14px;
            }}
            .speed-panel header code {{
              color: var(--accent);
              font-size: 12px;
            }}
            .speed-values {{
              display: grid;
              grid-template-columns: repeat(2, minmax(0, 1fr));
              gap: 10px;
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
            .speed-panel.stale .metric strong,
            .speed-panel.stale header strong {{
              color: #f97316;
            }}
            .signal-panel {{
              border: 1px solid rgba(148,163,184,0.18);
              border-radius: 8px;
              padding: 12px 14px;
              background: rgba(17,24,39,0.86);
            }}
            .signal-panel header {{
              display: flex;
              justify-content: space-between;
              align-items: center;
              gap: 12px;
              margin-bottom: 8px;
            }}
            .signal-panel header strong {{
              font-size: 14px;
            }}
            .signal-panel header code {{
              color: var(--accent);
              font-size: 12px;
            }}
            .signal-value {{
              display: flex;
              align-items: baseline;
              gap: 8px;
              font-variant-numeric: tabular-nums;
            }}
            .signal-value strong {{
              font-size: 28px;
              line-height: 1.1;
            }}
            .signal-value small {{
              color: var(--muted);
            }}
            .bar {{
              height: 8px;
              margin-top: 10px;
              border-radius: 999px;
              overflow: hidden;
              background: rgba(148,163,184,0.2);
            }}
            .bar span {{
              display: block;
              height: 100%;
              width: 0%;
              background: var(--accent);
              transition: width 0.12s linear, background 0.12s linear;
            }}
            .signal-panel.good .signal-value strong {{
              color: #22c55e;
            }}
            .signal-panel.warn .signal-value strong {{
              color: #f97316;
            }}
            .signal-panel.warn .bar span {{
              background: #f97316;
            }}
            .signal-panel.bad .signal-value strong {{
              color: #ef4444;
            }}
            .signal-panel.bad .bar span {{
              background: #ef4444;
            }}
            .signal-panel.stale .signal-value strong,
            .signal-panel.stale header strong,
            .speed-panel.stale .speed-values strong {{
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
              <div class="speed-panel" id="cmd-speed">
                <header>
                  <strong>模型计算速度</strong>
                  <code>{self.speed_cmd_topic}</code>
                </header>
                <div class="speed-values">
                  <div class="metric">
                    <label>Left</label>
                    <strong data-side="left">--</strong>
                    <small>RPM</small>
                  </div>
                  <div class="metric">
                    <label>Right</label>
                    <strong data-side="right">--</strong>
                    <small>RPM</small>
                  </div>
                </div>
              </div>
              <div class="speed-panel" id="status-speed">
                <header>
                  <strong>驱动下发速度</strong>
                  <code>{self.speed_status_topic}</code>
                </header>
                <div class="speed-values">
                  <div class="metric">
                    <label>Left</label>
                    <strong data-side="left">--</strong>
                    <small>RPM</small>
                  </div>
                  <div class="metric">
                    <label>Right</label>
                    <strong data-side="right">--</strong>
                    <small>RPM</small>
                  </div>
                </div>
              </div>
              <div class="signal-panel" id="line-detected">
                <header>
                  <strong>识别状态</strong>
                  <code>{self.line_detected_topic}</code>
                </header>
                <div class="signal-value">
                  <strong data-value>--</strong>
                  <small>有线/丢线</small>
                </div>
                <div class="bar"><span></span></div>
              </div>
              <div class="signal-panel" id="heading-error">
                <header>
                  <strong>航向误差</strong>
                  <code>{self.heading_error_topic}</code>
                </header>
                <div class="signal-value">
                  <strong data-value>--</strong>
                  <small>deg</small>
                </div>
                <div class="bar"><span></span></div>
              </div>
              <div class="signal-panel" id="lateral-error">
                <header>
                  <strong>横向偏差</strong>
                  <code>{self.lateral_error_topic}</code>
                </header>
                <div class="signal-value">
                  <strong data-value>--</strong>
                  <small>norm</small>
                </div>
                <div class="bar"><span></span></div>
              </div>
              <div class="signal-panel" id="cmd-vel">
                <header>
                  <strong>仿真速度</strong>
                  <code>{self.cmd_vel_topic}</code>
                </header>
                <div class="speed-values">
                  <div class="metric">
                    <label>Linear</label>
                    <strong data-side="linear">--</strong>
                    <small>m/s</small>
                  </div>
                  <div class="metric">
                    <label>Angular</label>
                    <strong data-side="angular">--</strong>
                    <small>rad/s</small>
                  </div>
                </div>
              </div>
            </section>
            <div class="grid">
              {''.join(cards)}
            </div>
          </div>
          <script>
            const cmdSpeed = document.querySelector("#cmd-speed");
            const statusSpeed = document.querySelector("#status-speed");
            const lineDetected = document.querySelector("#line-detected");
            const headingError = document.querySelector("#heading-error");
            const lateralError = document.querySelector("#lateral-error");
            const cmdVel = document.querySelector("#cmd-vel");

            function setSpeedPanel(panel, data) {{
              const stale = !data || Boolean(data.stale);
              panel.classList.toggle("stale", stale);
              panel.querySelector('[data-side="left"]').textContent =
                stale ? "--" : Number(data.left_rpm).toFixed(0);
              panel.querySelector('[data-side="right"]').textContent =
                stale ? "--" : Number(data.right_rpm).toFixed(0);
            }}

            function setPanelState(panel, state) {{
              panel.classList.remove("good", "warn", "bad", "stale");
              panel.classList.add(state);
            }}

            function setLineDetected(panel, data) {{
              const valueNode = panel.querySelector("[data-value]");
              const bar = panel.querySelector(".bar span");
              if (!data || data.stale) {{
                valueNode.textContent = "--";
                bar.style.width = "0%";
                setPanelState(panel, "stale");
                return;
              }}
              const ok = Boolean(data.value);
              valueNode.textContent = ok ? "有线" : "丢线";
              bar.style.width = "100%";
              setPanelState(panel, ok ? "good" : "bad");
            }}

            function setScalarPanel(panel, data, options) {{
              const valueNode = panel.querySelector("[data-value]");
              const bar = panel.querySelector(".bar span");
              if (!data || data.stale) {{
                valueNode.textContent = "--";
                bar.style.width = "0%";
                setPanelState(panel, "stale");
                return;
              }}
              const value = Number(data.value);
              const absValue = Math.abs(value);
              const ratio = Math.min(1, absValue / options.limit);
              valueNode.textContent = options.format(value);
              bar.style.width = `${{Math.round(ratio * 100)}}%`;
              if (absValue <= options.good) setPanelState(panel, "good");
              else if (absValue <= options.warn) setPanelState(panel, "warn");
              else setPanelState(panel, "bad");
            }}

            function setCmdVelPanel(panel, data) {{
              const stale = !data || Boolean(data.stale);
              panel.classList.toggle("stale", stale);
              panel.querySelector('[data-side="linear"]').textContent =
                stale ? "--" : Number(data.linear_x).toFixed(2);
              panel.querySelector('[data-side="angular"]').textContent =
                stale ? "--" : Number(data.angular_z).toFixed(2);
            }}

            async function refreshStatus() {{
              try {{
                const response = await fetch("/api/status", {{ cache: "no-store" }});
                const data = await response.json();
                setSpeedPanel(cmdSpeed, data.cmd);
                setSpeedPanel(statusSpeed, data.status);
                setLineDetected(lineDetected, data.line_detected);
                setScalarPanel(headingError, data.heading_error, {{
                  good: 4,
                  warn: 15,
                  limit: 45,
                  format: (v) => v.toFixed(1),
                }});
                setScalarPanel(lateralError, data.lateral_error, {{
                  good: 0.10,
                  warn: 0.30,
                  limit: 1.0,
                  format: (v) => v.toFixed(2),
                }});
                setCmdVelPanel(cmdVel, data.cmd_vel);
              }} catch (error) {{
                setSpeedPanel(cmdSpeed, null);
                setSpeedPanel(statusSpeed, null);
                setLineDetected(lineDetected, null);
                setScalarPanel(headingError, null, {{
                  good: 4,
                  warn: 15,
                  limit: 45,
                  format: (v) => v.toFixed(1),
                }});
                setScalarPanel(lateralError, null, {{
                  good: 0.10,
                  warn: 0.30,
                  limit: 1.0,
                  format: (v) => v.toFixed(2),
                }});
                setCmdVelPanel(cmdVel, null);
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
