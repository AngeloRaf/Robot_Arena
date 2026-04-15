#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gui_node.py — Interface web Flask du robot (écrit par Angelo).

Lancement normal (avec ROS 2) :
    python3 gui_node.py

Lancement en mode mock (sans ROS 2, pour développer seul) :
    MOCK=1 python3 gui_node.py          (Linux / Mac)
    $env:MOCK=1; python gui_node.py     (PowerShell Windows)
"""

import os
import threading
import time
import itertools

import cv2
import numpy as np
from flask import Flask, render_template, Response, jsonify, request, send_from_directory

# ── Détection du mode mock ──────────────────────────────────────────────────
MOCK_MODE = os.environ.get("MOCK", "0") == "1"

if not MOCK_MODE:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import String, Float32MultiArray

# ── État partagé entre ROS/mock et Flask (protégé par verrou) ───────────────
shared = {
    "last_frame":     None,
    "state":          "IDLE",
    "target_visible": False,
    "target_color":   "none",
    "target_x":       0.0,
    "target_area":    0.0,
    "connected":      True,
}
shared_lock = threading.Lock()

COLORS_VALID = {"red", "blue", "yellow", "green", "none"}


# ════════════════════════════════════════════════════════════════════════════
# Mode MOCK
# ════════════════════════════════════════════════════════════════════════════
class FakeCameraThread(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)

    def run(self):
        frame_num = 0
        while True:
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            img[:] = (20, 20, 30)

            for x in range(0, 640, 40):
                cv2.line(img, (x, 0), (x, 480), (30, 30, 45), 1)
            for y in range(0, 480, 40):
                cv2.line(img, (0, y), (640, y), (30, 30, 45), 1)

            with shared_lock:
                color_name = shared["target_color"]
                state = shared["state"]

            color_map = {
                "red":    (60,  60, 220),
                "blue":   (200, 80,  30),
                "yellow": (30, 200, 220),
                "green":  (30, 180,  60),
                "none":   (80,  80,  80),
            }
            cube_color = color_map.get(color_name, (80, 80, 80))
            cx = int(320 + 120 * np.sin(frame_num * 0.03))
            cy = int(240 + 80  * np.cos(frame_num * 0.02))
            cv2.rectangle(img, (cx - 40, cy - 40), (cx + 40, cy + 40), cube_color, -1)
            cv2.rectangle(img, (cx - 40, cy - 40), (cx + 40, cy + 40), (200, 200, 200), 2)
            cv2.putText(img, color_name.upper(), (cx - 30, cy + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            cv2.putText(img, f"[MOCK] STATE: {state}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 230, 180), 2)
            cv2.putText(img, time.strftime("%H:%M:%S"), (540, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (160, 160, 160), 1)

            with shared_lock:
                shared["last_frame"] = img
                shared["target_visible"] = color_name != "none"
                shared["target_x"]    = np.sin(frame_num * 0.03)
                shared["target_area"] = 0.15

            frame_num += 1
            time.sleep(1.0 / 15)


class FakeRosNode:
    def __init__(self):
        self._states = itertools.cycle(
            ["IDLE", "SEARCH", "APPROACH", "DELIVER", "RETURN"]
        )
        threading.Thread(target=self._tick_loop, daemon=True).start()

    def _tick_loop(self):
        while True:
            time.sleep(3)
            with shared_lock:
                shared["state"] = next(self._states)

    def send_color(self, color: str):
        print(f"[MOCK] /target_color ← {color}")
        with shared_lock:
            shared["target_color"] = color

    def destroy_node(self):
        pass


# ════════════════════════════════════════════════════════════════════════════
# Mode ROS 2 réel
# ════════════════════════════════════════════════════════════════════════════
if not MOCK_MODE:
    class GuiNode(Node):
        def __init__(self):
            super().__init__("gui_node")
            self.sub_image = self.create_subscription(
                Image, "/image_annotated", self.cb_image, 10)
            self.sub_state = self.create_subscription(
                String, "/robot_state", self.cb_state, 10)
            self.sub_target = self.create_subscription(
                Float32MultiArray, "/target", self.cb_target, 10)
            self.pub_color = self.create_publisher(String, "/target_color", 10)
            self.get_logger().info("gui_node prêt — interface Flask active")

        def cb_image(self, msg):
            img = np.frombuffer(msg.data, dtype=np.uint8)
            img = img.reshape(msg.height, msg.width, 3)
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            with shared_lock:
                shared["last_frame"] = bgr
                shared["connected"]  = True

        def cb_state(self, msg):
            with shared_lock:
                shared["state"] = msg.data

        def cb_target(self, msg):
            with shared_lock:
                shared["target_visible"] = bool(msg.data[0] > 0.5)
                shared["target_x"]       = float(msg.data[1]) if len(msg.data) > 1 else 0.0
                shared["target_area"]    = float(msg.data[2]) if len(msg.data) > 2 else 0.0

        def send_color(self, color: str):
            msg = String()
            msg.data = color
            self.pub_color.publish(msg)
            with shared_lock:
                shared["target_color"] = color

        def destroy_node(self):
            super().destroy_node()


# ════════════════════════════════════════════════════════════════════════════
# Flask — tout dans le même dossier que gui_node.py
# ════════════════════════════════════════════════════════════════════════════
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# template_folder ET static_folder pointent vers le dossier courant
app = Flask(
    __name__,
    template_folder=BASE_DIR,
    static_folder=BASE_DIR,
    static_url_path="/static",
)

ros_node = None

_BLANK_FRAME: bytes = cv2.imencode(
    ".jpg",
    np.zeros((480, 640, 3), dtype=np.uint8),
    [cv2.IMWRITE_JPEG_QUALITY, 60],
)[1].tobytes()


def mjpeg_generator():
    while True:
        with shared_lock:
            frame = shared["last_frame"]

        if frame is not None:
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            jpeg = buf.tobytes() if ok else _BLANK_FRAME
        else:
            jpeg = _BLANK_FRAME

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + jpeg
            + b"\r\n"
        )
        time.sleep(1.0 / 15)


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video_feed")
def video_feed():
    return Response(
        mjpeg_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/api/color", methods=["POST"])
def api_color():
    data  = request.get_json(silent=True) or {}
    color = data.get("color", "").lower()
    if color not in COLORS_VALID:
        return jsonify({"ok": False, "error": "invalid color"}), 400
    ros_node.send_color(color)
    return jsonify({"ok": True, "color": color})


@app.route("/api/stop", methods=["POST"])
def api_stop():
    ros_node.send_color("none")
    return jsonify({"ok": True})


@app.route("/api/state")
def api_state():
    with shared_lock:
        return jsonify({
            "state":          shared["state"],
            "target_color":   shared["target_color"],
            "target_visible": shared["target_visible"],
            "target_x":       shared["target_x"],
            "target_area":    shared["target_area"],
            "connected":      shared["connected"],
        })


# ════════════════════════════════════════════════════════════════════════════
# Lancement
# ════════════════════════════════════════════════════════════════════════════
def main():
    global ros_node

    if MOCK_MODE:
        print("=" * 50)
        print("  MODE MOCK — ROS 2 désactivé")
        print("  Interface disponible : http://localhost:5000")
        print("=" * 50)
        ros_node = FakeRosNode()
        FakeCameraThread().start()
        app.run(host="0.0.0.0", port=5000, threaded=True,
                debug=False, use_reloader=False)
    else:
        rclpy.init()
        ros_node = GuiNode()
        ros_thread = threading.Thread(
            target=rclpy.spin, args=(ros_node,), daemon=True
        )
        ros_thread.start()
        try:
            app.run(host="0.0.0.0", port=5000, threaded=True,
                    debug=False, use_reloader=False)
        finally:
            ros_node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()