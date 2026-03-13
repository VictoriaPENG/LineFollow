#!/usr/bin/env python3
"""巡线角度检测节点。"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


def make_binary(bgr, line_is_white=True, blur_ksize=5, thresh=-1, morph_ksize=3):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    k = max(1, int(blur_ksize))
    if k % 2 == 0:
        k += 1
    gray = cv2.GaussianBlur(gray, (k, k), 0)

    if thresh >= 0:
        ttype = cv2.THRESH_BINARY if line_is_white else cv2.THRESH_BINARY_INV
        _, bin_img = cv2.threshold(gray, int(thresh), 255, ttype)
    else:
        ttype = (
            cv2.THRESH_BINARY | cv2.THRESH_OTSU
            if line_is_white
            else cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU
        )
        _, bin_img = cv2.threshold(gray, 0, 255, ttype)

    mk = max(1, int(morph_ksize))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (mk, mk))
    bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, kernel, iterations=1)
    bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, kernel, iterations=1)
    return bin_img


def detect_line_and_angle(bin_img, n_windows=9, margin=60, minpix=50, kp=1.0, draw=False):
    h, w = bin_img.shape[:2]

    y0 = (2 * h) // 3
    bottom = bin_img[y0:h, :]
    hist = np.sum(bottom > 0, axis=0).astype(np.int32)
    base_x = int(np.argmax(hist))

    nw = max(3, int(n_windows))
    win_h = h // nw
    margin = max(10, int(margin))
    minpix = max(10, int(minpix))

    current_x = base_x
    centers = []
    vis = None
    if draw:
        vis = cv2.cvtColor(bin_img, cv2.COLOR_GRAY2BGR)

    for i in range(nw):
        win_y_low = max(0, h - (i + 1) * win_h)
        win_y_high = min(h, h - i * win_h)
        win_x_low = max(0, current_x - margin)
        win_x_high = min(w, current_x + margin)
        if win_x_high <= win_x_low or win_y_high <= win_y_low:
            continue

        roi = bin_img[win_y_low:win_y_high, win_x_low:win_x_high]
        nz = cv2.findNonZero(roi)
        if nz is not None and len(nz) >= minpix:
            pts = nz.reshape(-1, 2)
            cx = int(np.mean(pts[:, 0])) + win_x_low
            cy = int(np.mean(pts[:, 1])) + win_y_low
            centers.append((cx, cy))
            current_x = cx
            if draw:
                cv2.rectangle(vis, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 2)
                cv2.circle(vis, (cx, cy), 4, (0, 0, 255), -1)
        elif draw:
            cv2.rectangle(vis, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)

    if len(centers) < 3:
        return False, 0.0, 0.0, vis

    centers_np = np.array(centers, dtype=np.float32).reshape(-1, 1, 2)
    vx, vy, x0f, y0f = cv2.fitLine(centers_np, cv2.DIST_L2, 0, 0.01, 0.01).flatten()

    angle = math.atan2(float(vx), float(-vy))
    angle_deg = angle * 180.0 / math.pi
    steer_deg = -float(kp) * angle_deg

    if draw and vis is not None:
        eps = 1e-6
        k = 1e6 if abs(vy) < eps else (vx / vy)
        y1, y2 = h - 1, 0
        x1 = int(round(x0f + (y1 - y0f) * k))
        x2 = int(round(x0f + (y2 - y0f) * k))
        cv2.line(vis, (x1, y1), (x2, y2), (0, 255, 255), 2)
        txt = f"angle={angle_deg:.2f} deg  steer={steer_deg:.2f} deg"
        cv2.putText(vis, txt, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    return True, angle_deg, steer_deg, vis


class LineFollowAngleNode(Node):
    """检测图像中的巡线角度。"""

    def __init__(self):
        super().__init__("line_follow_angle_node")

        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("line_is_white", False)
        self.declare_parameter("blur_ksize", 5)
        self.declare_parameter("thresh", -1)
        self.declare_parameter("morph_ksize", 3)
        self.declare_parameter("n_windows", 9)
        self.declare_parameter("margin", 60)
        self.declare_parameter("minpix", 50)
        self.declare_parameter("kp", 1.0)
        self.declare_parameter("show_debug", False)

        self.bridge = CvBridge()
        img_topic = self.get_parameter("image_topic").value
        self.sub = self.create_subscription(Image, img_topic, self.on_image, 10)
        self.pub_steer = self.create_publisher(Float32, "/line_follow/steer_angle_deg", 10)
        self.pub_angle = self.create_publisher(Float32, "/line_follow/line_angle_deg", 10)

        self.get_logger().info(f"Subscribed to {img_topic}, publishing /line_follow/steer_angle_deg")

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge convert failed: {e}")
            return

        line_is_white = bool(self.get_parameter("line_is_white").value)
        blur_ksize = int(self.get_parameter("blur_ksize").value)
        thresh = int(self.get_parameter("thresh").value)
        morph_ksize = int(self.get_parameter("morph_ksize").value)
        n_windows = int(self.get_parameter("n_windows").value)
        margin = int(self.get_parameter("margin").value)
        minpix = int(self.get_parameter("minpix").value)
        kp = float(self.get_parameter("kp").value)
        show = bool(self.get_parameter("show_debug").value)

        bin_img = make_binary(frame, line_is_white, blur_ksize, thresh, morph_ksize)
        ok, angle_deg, steer_deg, vis = detect_line_and_angle(
            bin_img, n_windows=n_windows, margin=margin, minpix=minpix, kp=kp, draw=show
        )

        if ok:
            m1 = Float32()
            m1.data = float(angle_deg)
            self.pub_angle.publish(m1)

            m2 = Float32()
            m2.data = float(steer_deg)
            self.pub_steer.publish(m2)

        if show:
            cv2.imshow("binary", bin_img)
            if vis is not None:
                cv2.imshow("detect", vis)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = LineFollowAngleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    main()
