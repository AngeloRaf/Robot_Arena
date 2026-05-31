#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gui_node.py — Interface web Flask
"""

import os
import threading
import time
import json

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request, send_from_directory

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

BASE_DIR = os.path.abspath(os.path.expanduser('~/robot_ws/src/robot_arena/robot_arena'))

COLOR_MAP = {
    'red':    'rouge',
    'blue':   'bleu',
    'yellow': 'jaune',
    'green':  'vert',
    'none':   'none',
}

shared = {
    'last_frame':     None,
    'state':          'CHERCHER',
    'target_visible': False,
    'target_color':   'none',
    'target_x':       0.0,
    'target_area':    0.0,
}
shared_lock = threading.Lock()

_BLANK_FRAME = cv2.imencode(
    '.jpg',
    np.zeros((480, 640, 3), dtype=np.uint8),
    [cv2.IMWRITE_JPEG_QUALITY, 60],
)[1].tobytes()


class GuiNode(Node):

    def __init__(self):
        super().__init__('gui_node')
        self.bridge = CvBridge()
        self._last_frame_time = 0

        self.create_subscription(Image,  '/image_annotated', self._cb_image, 10)
        self.create_subscription(String, '/robot_state',     self._cb_state, 10)
        self.create_subscription(String, '/cube_detections', self._cb_cubes, 10)

        self.pub_mission = self.create_publisher(String, '/mission', 10)
        self.get_logger().info('gui_node pret — http://0.0.0.0:5000')

    def _cb_image(self, msg):
        now = time.time()
        if now - self._last_frame_time < 0.08:
            return
        self._last_frame_time = now
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with shared_lock:
                shared['last_frame'] = frame.copy()
        except Exception as e:
            self.get_logger().warn(f'Image: {e}')

    def _cb_state(self, msg):
        try:
            data  = json.loads(msg.data)
            state = data.get('state', 'CHERCHER')
            color = data.get('target_color', 'none')

            state_map = {
                'CHERCHER':      'SEARCH',
                'ALIGNER':       'SEARCH',
                'APPROCHER':     'APPROACH',
                'POUSSER':       'APPROACH',
                'RETOUR_DEPART': 'RETURN',
                'CHERCHER_GOAL': 'DELIVER',
                'POUSSER_DROIT': 'DELIVER',
                'RETOUR':        'RETURN',
            }
            color_map_rev = {
                'rouge': 'red',
                'bleu':  'blue',
                'jaune': 'yellow',
                'vert':  'green',
                'none':  'none',
                '':      'none',
            }
            with shared_lock:
                shared['state']        = state_map.get(state, state)
                shared['target_color'] = color_map_rev.get(color, 'none')
        except Exception:
            pass

    def _cb_cubes(self, msg):
        try:
            data = json.loads(msg.data)
            dets = [d for d in data.get('detections', [])
                    if not d.get('ghost', False)]
            with shared_lock:
                color = shared['target_color']

            color_fr = COLOR_MAP.get(color, color)
            target   = next((d for d in dets if d['color'] == color_fr), None)

            if target:
                with shared_lock:
                    shared['target_visible'] = True
                    shared['target_x']       = target.get('x_norm', 0.0)
                    shared['target_area']    = min(target['area'] / 8000.0, 1.0)
            else:
                with shared_lock:
                    shared['target_visible'] = False
                    shared['target_x']       = 0.0
                    shared['target_area']    = 0.0
        except Exception:
            pass

    def send_mission(self, color_en: str):
        color_fr = COLOR_MAP.get(color_en, 'none')
        msg      = String()
        msg.data = json.dumps({'color': color_fr, 'goal_id': 0})
        self.pub_mission.publish(msg)
        with shared_lock:
            shared['target_color'] = color_en
        self.get_logger().info(f'Mission: {color_en} → {color_fr}')

    def send_stop(self):
        msg      = String()
        msg.data = json.dumps({'color': 'none', 'goal_id': -1})
        self.pub_mission.publish(msg)
        with shared_lock:
            shared['target_color']   = 'none'
            shared['target_visible'] = False


# ── Flask ─────────────────────────────────────────────────────────────
# Plus de static_folder ici — on gère tout manuellement avec send_from_directory
app      = Flask(__name__)
ros_node = None


def mjpeg_generator():
    last_sent = 0
    while True:
        now = time.time()
        if now - last_sent < 0.1:
            time.sleep(0.02)
            continue
        with shared_lock:
            frame = shared['last_frame']
        if frame is not None:
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
            jpeg = buf.tobytes() if ok else _BLANK_FRAME
        else:
            jpeg = _BLANK_FRAME
            time.sleep(0.05)
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n'
               + jpeg + b'\r\n')
        last_sent = now


@app.route('/')
def index():
    return send_from_directory(BASE_DIR, 'index.html')


@app.route('/static/<path:filename>')
def static_files(filename):
    return send_from_directory(BASE_DIR, filename)


@app.route('/video_feed')
def video_feed():
    return Response(mjpeg_generator(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/color', methods=['POST'])
def api_color():
    data  = request.get_json(silent=True) or {}
    color = data.get('color', '').lower()
    if color not in COLOR_MAP:
        return jsonify({'ok': False, 'error': 'invalid color'}), 400
    ros_node.send_mission(color)
    return jsonify({'ok': True, 'color': color})


@app.route('/api/stop', methods=['POST'])
def api_stop():
    ros_node.send_stop()
    return jsonify({'ok': True})


@app.route('/api/state')
def api_state():
    with shared_lock:
        return jsonify({
            'state':          shared['state'],
            'target_color':   shared['target_color'],
            'target_visible': shared['target_visible'],
            'target_x':       shared['target_x'],
            'target_area':    shared['target_area'],
            'connected':      True,
        })


def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = GuiNode()

    flask_thread = threading.Thread(
        target=lambda: app.run(
            host='0.0.0.0', port=5000,
            threaded=True, debug=False, use_reloader=False),
        daemon=True)
    flask_thread.start()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()