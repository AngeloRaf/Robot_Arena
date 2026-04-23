#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vision_node.py v8 — Pipeline HSV pur (style PyImageSearch)
Blur → HSV → inRange → erode/dilate → contours
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import math
import numpy as np
import json
import time

# ── Plages HSV calibrées lumière naturelle + muret blanc ──────────────
# Méthode PyImageSearch : GaussianBlur puis HSV pur, pas de LAB/CLAHE
HSV_COLORS = {
    'rouge': {
        'ranges': [
            ([0,   100, 80],  [8,   255, 255]),   # rouge chaud
            ([160, 100, 80],  [179, 255, 255]),    # rouge foncé
        ],
        'bgr': (0, 0, 255),
    },
    'bleu': {
        'ranges': [
            ([100, 150, 80],  [120, 255, 255]),
        ],
        'bgr': (255, 100, 0),
    },
    'jaune': {
        'ranges': [
            ([22, 100, 100], [35, 255, 255]),
        ],
        'bgr': (0, 255, 255),
    },
    'vert': {
        'ranges': [
            ([40, 80, 60],  [80, 255, 255]),
        ],
        'bgr': (0, 255, 0),
    },
}

MIN_AREA        = 400
MAX_AREA        = 22000
SMOOTHING       = 0.5
MEMORY_DURATION = 0.3


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()

        # Kernels morphologiques — style PyImageSearch
        self.kernel_erode  = np.ones((3, 3), np.uint8)
        self.kernel_dilate = np.ones((7, 7), np.uint8)

        self.tracked   = {}
        self.last_seen = {}

        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.create_subscription(Image, '/image_raw', self._cb_image, 10)
        self.pub_cubes     = self.create_publisher(String, '/cube_detections',  10)
        self.pub_aruco     = self.create_publisher(String, '/aruco_detections', 10)
        self.pub_annotated = self.create_publisher(Image,  '/image_annotated',  10)

        self.frame_count = 0
        self.fps_start   = time.time()
        self.current_fps = 0.0
        self.create_timer(3.0, self._log_fps)

        self.get_logger().info('vision_node v8 — HSV pur pret')

    def _log_fps(self):
        elapsed = time.time() - self.fps_start
        if elapsed > 0:
            self.current_fps = self.frame_count / elapsed
        self.get_logger().info(f'FPS: {self.current_fps:.1f}')
        self.frame_count = 0
        self.fps_start   = time.time()

    def _detect_color(self, frame_hsv, blurred_bgr, color_name, cfg):
        """
        Pipeline PyImageSearch :
        1. inRange sur HSV
        2. erode  → supprime le bruit
        3. dilate → rebouche les trous
        4. findContours → détections
        """
        mask = np.zeros(frame_hsv.shape[:2], dtype=np.uint8)
        for (low, high) in cfg['ranges']:
            mask |= cv2.inRange(frame_hsv,
                                np.array(low,  dtype=np.uint8),
                                np.array(high, dtype=np.uint8))

        # Deux passes erode + deux passes dilate (recommandé PyImageSearch)
        mask = cv2.erode(mask,  self.kernel_erode,  iterations=2)
        mask = cv2.dilate(mask, self.kernel_dilate, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        frame_w = frame_hsv.shape[1]

        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_AREA or area > MAX_AREA:
                continue
            x, y, w, h = cv2.boundingRect(c)
            ratio = w / h if h > 0 else 0
            if ratio < 0.25 or ratio > 4.0:
                continue
            if w < 8 or h < 8:
                continue
            cx, cy = x + w // 2, y + h // 2
            x_norm = (cx - frame_w / 2) / (frame_w / 2)
            detections.append({
                'color':  color_name,
                'x': cx, 'y': cy,
                'w': w,  'h': h,
                'area':   int(area),
                'x_norm': round(x_norm, 3),
            })
        return detections

    def _smooth(self, raw):
        now      = time.time()
        smoothed = []

        by_color = {}
        for d in raw:
            c = d['color']
            if c not in by_color or d['area'] > by_color[c]['area']:
                by_color[c] = d

        for color, det in by_color.items():
            self.last_seen[color] = now
            if color in self.tracked:
                prev = self.tracked[color]
                s    = SMOOTHING
                det['x']    = int(prev['x']    * s + det['x']    * (1 - s))
                det['y']    = int(prev['y']    * s + det['y']    * (1 - s))
                det['area'] = int(prev['area'] * s + det['area'] * (1 - s))
            self.tracked[color] = det.copy()
            smoothed.append(det)

        for color in list(self.tracked.keys()):
            if color not in by_color:
                age = now - self.last_seen.get(color, 0)
                if age < MEMORY_DURATION:
                    ghost          = self.tracked[color].copy()
                    ghost['ghost'] = True
                    smoothed.append(ghost)
                else:
                    del self.tracked[color]

        return smoothed

    def _detect_aruco(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

        h, w    = frame.shape[:2]
        markers = []

        if ids is not None and len(ids) > 0:
            for i, mid in enumerate(ids.flatten()):
                mc     = corners[i][0]
                cx     = int(mc[:, 0].mean())
                cy     = int(mc[:, 1].mean())
                wpx    = ((mc[0][0]-mc[1][0])**2 + (mc[0][1]-mc[1][1])**2)**0.5
                hpx    = ((mc[1][0]-mc[2][0])**2 + (mc[1][1]-mc[2][1])**2)**0.5
                size   = float((wpx + hpx) / 2.0)
                x_norm = (cx - w / 2) / (w / 2)

                markers.append({
                    'id':      int(mid),
                    'cx':      cx,
                    'cy':      cy,
                    'size':    round(size, 1),
                    'x_norm':  round(x_norm, 3),
                    'is_goal': int(mid) in [0, 2],
                })

                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.putText(
                    frame,
                    f'ID:{mid} {"GOAL" if int(mid) in [0, 2] else "MUR"}',
                    (cx - 30, cy - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        return markers

    def _cb_image(self, msg: Image):
        self.frame_count += 1

        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except ValueError:
            return

        h, w = frame.shape[:2]

        # Crop : 2/3 bas de l'image
        crop_top = h // 3
        roi      = frame[crop_top:, :, :].copy()

        # ── Pipeline PyImageSearch ──────────────────────────
        blurred = cv2.GaussianBlur(roi, (11, 11), 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        raw = []
        for color_name, cfg in HSV_COLORS.items():
            raw += self._detect_color(hsv, blurred, color_name, cfg)

        # Recalage y sur image complète
        for d in raw:
            d['y'] += crop_top

        smoothed = self._smooth(raw)
        smoothed.sort(key=lambda d: d['area'], reverse=True)

        # ── Détection ArUco ─────────────────────────────────
        aruco_markers = self._detect_aruco(frame)

        # ── Filtrer cubes trop proches d'un ArUco ───────────
        aruco_positions = [(m['cx'], m['cy']) for m in aruco_markers]
        filtered = []
        for d in smoothed:
            too_close = False
            for (ax, ay) in aruco_positions:
                dist_px = math.hypot(d['x'] - ax, d['y'] - ay)
                if dist_px < 80:
                    too_close = True
                    break
            if not too_close:
                filtered.append(d)
        smoothed = filtered

        # ── Annotation ──────────────────────────────────────
        for d in smoothed:
            if d.get('ghost'):
                continue
            bgr = HSV_COLORS.get(d['color'], {}).get('bgr', (255, 255, 255))
            x1  = d['x'] - d['w'] // 2
            y1  = d['y'] - d['h'] // 2
            cv2.rectangle(frame, (x1, y1), (x1 + d['w'], y1 + d['h']), bgr, 2)
            cv2.putText(frame, f"{d['color']} {d['area']}",
                        (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 2)

        # ── Publications ─────────────────────────────────────
        cube_msg      = String()
        cube_msg.data = json.dumps({
            'detections':   smoothed,
            'frame_width':  w,
            'frame_height': h,
            'fps':          round(self.current_fps, 1),
        })
        self.pub_cubes.publish(cube_msg)

        aruco_msg      = String()
        aruco_msg.data = json.dumps({'markers': aruco_markers})
        self.pub_aruco.publish(aruco_msg)

        self.pub_annotated.publish(
            self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()