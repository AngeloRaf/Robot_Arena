#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nav_node.py v5 — FSM navigation sans IMU, sans servo
Goal automatique : prend le premier ArUco ID 0 ou 2 visible
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import time
import math


class NavNode(Node):

    # ── Paramètres ───────────────────────────────────────────
    SEARCH_SPEED      = 0.4
    GOAL_SEARCH_SPEED = 0.35
    TURN_180_SPEED    = 2.0    # rad/s rotation rapide 180°
    TURN_180_DURATION = 9.0   # s — calibré sur le robot
    MAX_FWD_SPEED     = 0.4
    MIN_FWD_SPEED     = 0.25
    PUSH_SPEED        = 0.45
    PUSH_DURATION     = 3.0
    PUSH_DROIT_SPEED  = 0.28
    PUSH_DROIT_MIN    = 0.10
    ALIGN_TOL         = 35
    GOAL_ALIGN_TOL    = 25
    CLOSE_AREA      = 8000
    BLIND_SPOT_AREA = 6000
    LOST_TIMEOUT      = 5.0
    STATE_TIMEOUT     = 60.0
    CONFIRM_NEEDED    = 3
    DEPOT_STOP_SIZE   = 100
    DEPOT_BRAKE_SIZE  = 160
    RECUL_SPEED       = 0.30
    WALL_BRAKE_SIZE   = 200
    KP                = 0.003
    KI                = 0.0001
    KD                = 0.002
    MAX_ANGULAR       = 0.45
    ANGULAR_CORR      = 0.004

    CLOSE_AREA_COLOR  = {'jaune': 8000}
    BLIND_AREA_COLOR  = {'jaune': 6000}
    GOAL_IDS          = [0, 2]
    WALL_IDS          = [1, 3]

    def __init__(self):
        super().__init__('nav_node')

        self.state            = 'CHERCHER'
        self.state_start_time = time.time()
        self.last_loop_time   = time.time()

        self.target_color    = None
        self.detections      = []
        self.real_detections = []
        self.frame_width     = 640
        self.last_cube_area  = 0
        self.last_cube_color = None
        self.last_cube_time  = 0.0

        self.confirm_count   = 0
        self.confirm_color   = None

        self.distance_forward    = 0.0
        self.blind_advance_start = None
        self._blind_start_x      = 0.0
        self._blind_start_y      = 0.0
        self.push_start          = None
        self.push_color          = None

        # RETOUR_DEPART
        self._retour_phase         = None
        self._retour_turn_start    = None
        self._retour_adv_start     = None
        self._retour_start_heading = 0.0

        # Odométrie
        self.odom_x       = 0.0
        self.odom_y       = 0.0
        self.odom_heading = 0.0
        self.has_odom     = False
        self._state_start_x = 0.0
        self._state_start_y = 0.0

        # ArUco
        self.aruco_markers   = []
        self.goal_marker_id  = -1
        self.aruco_offset_px = 0.0
        self.aruco_size      = 0.0
        self.aruco_age       = 99.0

        self.delivery_dist = 0.0

        self.integral   = 0.0
        self.prev_error = 0.0

        self.create_subscription(String, '/mission',          self._cb_mission, 10)
        self.create_subscription(String, '/cube_detections',  self._cb_cubes,   10)
        self.create_subscription(String, '/aruco_detections', self._cb_aruco,   10)
        self.create_subscription(String, '/odom_simple',      self._cb_odom,    10)

        self.pub_cmd   = self.create_publisher(Twist,  '/cmd_vel',     10)
        self.pub_state = self.create_publisher(String, '/robot_state', 10)

        self.create_timer(0.1, self._loop)
        self.create_timer(0.5, self._publish_state)

        self.get_logger().info('nav_node v5 pret')

    # ═══════════════════════════════════════════════════════
    # Callbacks
    # ═══════════════════════════════════════════════════════

    def _cb_mission(self, msg):
        try:
            data  = json.loads(msg.data)
            color = data.get('color', '').lower()
        except Exception as e:
            self.get_logger().warn(f'Mission invalide: {e}')
            return

        if color == 'none' or color == '':
            self._stop()
            self.target_color = None
            self._change_state('CHERCHER')
            return

        self.target_color = color
        self.get_logger().info(f'Mission reçue: {color}')
        self._change_state('CHERCHER')

    def _cb_cubes(self, msg):
        try:
            data = json.loads(msg.data)
            self.detections      = data.get('detections', [])
            self.frame_width     = data.get('frame_width', 640)
            self.real_detections = [d for d in self.detections
                                    if not d.get('ghost', False)]
            if self.target_color:
                targets = [d for d in self.real_detections
                           if d['color'] == self.target_color]
                if targets:
                    best = targets[0]
                    self.last_cube_area  = best['area']
                    self.last_cube_color = best['color']
                    self.last_cube_time  = time.time()
        except Exception:
            pass

    def _cb_aruco(self, msg):
        try:
            data = json.loads(msg.data)
            self.aruco_markers = data.get('markers', [])

            if self.goal_marker_id >= 0:
                for m in self.aruco_markers:
                    if m['id'] == self.goal_marker_id:
                        self.aruco_offset_px = (m.get('x_norm', 0)
                                                * (self.frame_width / 2))
                        self.aruco_size  = m.get('size', 0)
                        self.aruco_age   = 0.0
                        return
                self.aruco_age = 99.0
            else:
                self.aruco_age = 99.0
        except Exception:
            pass

    def _cb_odom(self, msg):
        try:
            data = json.loads(msg.data)
            self.odom_x       = data.get('x', 0.0)
            self.odom_y       = data.get('y', 0.0)
            self.odom_heading = data.get('heading_deg', 0.0)
            self.has_odom     = True
        except Exception:
            pass

    # ═══════════════════════════════════════════════════════
    # Utilitaires
    # ═══════════════════════════════════════════════════════

    def _cmd(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.pub_cmd.publish(msg)

    def _stop(self):
        self._cmd(0.0, 0.0)

    def _elapsed(self):
        return time.time() - self.state_start_time

    def _change_state(self, new_state):
        self.get_logger().info(f'[{self.state}] → [{new_state}]')
        self.state            = new_state
        self.state_start_time = time.time()
        self.last_loop_time   = time.time()
        self.integral         = 0.0
        self.prev_error       = 0.0
        self._state_start_x   = self.odom_x
        self._state_start_y   = self.odom_y

        if new_state == 'CHERCHER':
            self.confirm_count = 0
            self.confirm_color = None

        if new_state == 'RETOUR_DEPART':
            self._retour_phase      = 'TURN'
            self._retour_turn_start = time.time()
            self._retour_adv_start  = None

        if new_state == 'CHERCHER_GOAL':
            self.goal_marker_id = -1
            self.aruco_age      = 99.0

        if new_state == 'POUSSER_DROIT':
            self.delivery_dist = 0.0

        self._publish_state()

    def _pid(self, error):
        self.integral   = max(-1000, min(1000, self.integral + error))
        derivative      = error - self.prev_error
        self.prev_error = error
        out = (self.KP * error
               + self.KI * self.integral
               + self.KD * derivative)
        return max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, out))

    def _get_cube(self):
        if not self.real_detections:
            return None
        if self.target_color:
            targets = [d for d in self.real_detections
                       if d['color'] == self.target_color]
            return targets[0] if targets else None
        center = self.frame_width // 2
        return min(self.real_detections, key=lambda d: abs(d['x'] - center))

    def _get_any_goal_marker(self):
        for m in self.aruco_markers:
            if m['id'] in self.GOAL_IDS:
                return m
        return None

    def _close_area(self):
        return self.CLOSE_AREA_COLOR.get(self.target_color, self.CLOSE_AREA)

    def _blind_area(self):
        return self.BLIND_AREA_COLOR.get(
            self.target_color or self.last_cube_color,
            self.BLIND_SPOT_AREA)

    def _in_blind_spot(self):
        no_target = (
            not any(d['color'] == self.target_color
                    for d in self.real_detections)
            if self.target_color
            else len(self.real_detections) == 0
        )
        was_close = self.last_cube_area > self._blind_area()
        recent    = (time.time() - self.last_cube_time) < 2.5
        return no_target and was_close and recent

    def _target_lost(self):
        if not self.target_color:
            return True
        if any(d['color'] == self.target_color for d in self.real_detections):
            return False
        return time.time() - self.last_cube_time > self.LOST_TIMEOUT

    def _odom_dist(self):
        if not self.has_odom:
            return 0.0
        return math.hypot(self.odom_x - self._state_start_x,
                          self.odom_y - self._state_start_y)

    def _goal_visible(self):
        return self.goal_marker_id >= 0 and self.aruco_age < 1.5

    def _publish_state(self):
        msg = String()
        msg.data = json.dumps({
            'state':        self.state,
            'target_color': self.target_color or '',
            'goal_id':      self.goal_marker_id,
            'odom': {
                'x': round(self.odom_x, 3),
                'y': round(self.odom_y, 3),
            },
        })
        self.pub_state.publish(msg)

    # ═══════════════════════════════════════════════════════
    # Boucle principale
    # ═══════════════════════════════════════════════════════

    def _loop(self):
        handlers = {
            'CHERCHER':      self._state_chercher,
            'ALIGNER':       self._state_aligner,
            'APPROCHER':     self._state_approcher,
            'POUSSER':       self._state_pousser,
            'RETOUR_DEPART': self._state_retour_depart,
            'CHERCHER_GOAL': self._state_chercher_goal,
            'POUSSER_DROIT': self._state_pousser_droit,
            'RETOUR':        self._state_retour,
        }
        h = handlers.get(self.state)
        if h:
            h()

    # ═══════════════════════════════════════════════════════
    # États
    # ═══════════════════════════════════════════════════════

    def _state_chercher(self):
        if not self.target_color:
            self._stop()
            return

        cube = self._get_cube()
        if cube:
            if cube['color'] == self.confirm_color:
                self.confirm_count += 1
            else:
                self.confirm_color = cube['color']
                self.confirm_count = 1

            if self.confirm_count >= self.CONFIRM_NEEDED:
                self._stop()
                time.sleep(0.2)
                self.confirm_count = 0
                self.confirm_color = None
                self.get_logger().info(
                    f'[CHERCHER] {cube["color"]} confirmé area={cube["area"]}')
                self._change_state('ALIGNER')
                return
        else:
            self.confirm_count = 0
            self.confirm_color = None

        self._cmd(angular=self.SEARCH_SPEED)

    def _state_aligner(self):
        cube = self._get_cube()

        if not cube:
            if self._target_lost():
                self._stop()
                self.get_logger().warn('[ALIGNER] Cube perdu → CHERCHER')
                self._change_state('CHERCHER')
            return

        center_x = self.frame_width // 2
        error    = cube['x'] - center_x

        if abs(error) < self.ALIGN_TOL:
            self._stop()
            time.sleep(0.15)
            self.distance_forward = 0.0
            self.get_logger().info(f'[ALIGNER] OK error={error}px → APPROCHER')
            self._change_state('APPROCHER')
            return

        self._cmd(angular=-self._pid(error))

    def _state_approcher(self):
        now = time.time()
        dt  = now - self.last_loop_time
        self.last_loop_time = now

        # ── Blind spot ──────────────────────────────────────
        if self._in_blind_spot():
            if self.blind_advance_start is None:
                self.blind_advance_start = now
                self._blind_start_x = self.odom_x
                self._blind_start_y = self.odom_y
                self.get_logger().info('[APPROCHER] Blind spot → fonce 30cm')

            dist_blind    = math.hypot(
                self.odom_x - self._blind_start_x,
                self.odom_y - self._blind_start_y)
            elapsed_blind = now - self.blind_advance_start

            if dist_blind >= 0.30 or elapsed_blind > 5.0:
                self.blind_advance_start = None
                self.push_color = self.target_color
                self.push_start = time.time()
                self.get_logger().info(
                    f'[APPROCHER] 30cm OK dist_forward={self.distance_forward:.2f}m → POUSSER')
                self._change_state('POUSSER')
                return

            self._cmd(linear=self.PUSH_SPEED)
            self.distance_forward += self.PUSH_SPEED * dt
            return

        self.blind_advance_start = None

        # ── Cube perdu hors blind spot ───────────────────────
        cube = self._get_cube()
        if not cube:
            if self._target_lost():
                self._stop()
                self.get_logger().warn('[APPROCHER] Cube perdu → CHERCHER')
                self._change_state('CHERCHER')
                return
            self._cmd(linear=self.MIN_FWD_SPEED)
            self.distance_forward += self.MIN_FWD_SPEED * dt
            return

        if self._elapsed() > self.STATE_TIMEOUT:
            self._stop()
            self._change_state('CHERCHER')
            return

        # ── Cube visible → avancer en centrant ──────────────
        center_x = self.frame_width // 2
        error    = cube['x'] - center_x
        close_th = self._close_area()

        if cube['area'] > close_th:
            self.push_color = self.target_color
            self.push_start = time.time()
            self.get_logger().info(
                f'[APPROCHER] proche area={cube["area"]} → POUSSER')
            self._change_state('POUSSER')
            return

        angular    = -self._pid(error)
        area_ratio = min(cube['area'] / close_th, 1.0)
        speed      = max(self.MIN_FWD_SPEED,
                         self.MAX_FWD_SPEED * (1.0 - area_ratio * 0.55))

        if abs(error) > 100:
            self._cmd(linear=speed * 0.5, angular=angular)
            self.distance_forward += speed * 0.5 * dt
        else:
            self._cmd(linear=speed, angular=angular * (1.0 - area_ratio * 0.4))
            self.distance_forward += speed * dt

    def _state_pousser(self):
        """Pousse 3s puis RETOUR_DEPART."""
        elapsed = time.time() - self.push_start

        if elapsed > self.PUSH_DURATION:
            self._stop()
            time.sleep(0.3)
            self.get_logger().info(
                f'[POUSSER] Engagé dist_forward={self.distance_forward:.2f}m → RETOUR_DEPART')
            self._change_state('RETOUR_DEPART')
            return

        self._cmd(linear=self.PUSH_SPEED)

    def _state_retour_depart(self):
        """
        Phase TURN    : tourne 180° rapidement (TURN_180_SPEED, TURN_180_DURATION)
        Phase ADVANCE : avance distance_forward pour revenir au centre
        → CHERCHER_GOAL
        """
        if self._retour_phase == 'TURN':
            spin_elapsed = time.time() - self._retour_turn_start

            if spin_elapsed >= self.TURN_180_DURATION:
                self._stop()
                time.sleep(0.25)
                self._retour_phase     = 'ADVANCE'
                self._retour_adv_start = time.time()
                self._state_start_x    = self.odom_x
                self._state_start_y    = self.odom_y
                self.get_logger().info(
                    f'[RETOUR_DEPART] 180° OK → avance {self.distance_forward:.2f}m')
                return

            self._cmd(angular=self.TURN_180_SPEED)
            return

        if self._retour_phase == 'ADVANCE':
            dist_done   = self._odom_dist()
            elapsed_adv = time.time() - self._retour_adv_start
            target_dist = max(self.distance_forward * 0.85, 0.05)
            time_limit  = (target_dist / self.MIN_FWD_SPEED) + 4.0

            if dist_done >= target_dist or elapsed_adv > time_limit:
                self._stop()
                time.sleep(0.2)
                self.get_logger().info(
                    f'[RETOUR_DEPART] Centre OK dist={dist_done:.2f}m → CHERCHER_GOAL')
                self._change_state('CHERCHER_GOAL')
                return

            self._cmd(linear=self.MIN_FWD_SPEED)

    def _state_chercher_goal(self):
        """
        Tourne jusqu'à trouver ArUco goal ID 0 ou 2.
        Une fois centré → POUSSER_DROIT.
        """
        goal_marker = self._get_any_goal_marker()

        if goal_marker:
            self.goal_marker_id  = goal_marker['id']
            self.aruco_offset_px = (goal_marker.get('x_norm', 0)
                                    * (self.frame_width / 2))
            self.aruco_size      = goal_marker.get('size', 0)
            self.aruco_age       = 0.0
            offset = self.aruco_offset_px

            if abs(offset) < self.GOAL_ALIGN_TOL:
                self._stop()
                time.sleep(0.4)
                marker2 = self._get_any_goal_marker()
                if marker2:
                    offset2 = marker2.get('x_norm', 0) * (self.frame_width / 2)
                    if abs(offset2) > self.GOAL_ALIGN_TOL:
                        corr = 0.12 if offset2 > 0 else -0.12
                        self._cmd(angular=-corr)
                        return
                self.push_start    = time.time()
                self.delivery_dist = 0.0
                self.get_logger().info(
                    f'[CHERCHER_GOAL] Goal ID {self.goal_marker_id} centré → POUSSER_DROIT')
                self._change_state('POUSSER_DROIT')
                return

            corr = max(0.12, min(0.35,
                       abs(offset) / (self.frame_width / 2) * 0.5))
            self._cmd(angular=-corr if offset > 0 else corr)
            return

        self._cmd(angular=self.GOAL_SEARCH_SPEED)

    def _state_pousser_droit(self):
        """
        Avance vers goal avec décélération ArUco.
        Arrêt quand size >= DEPOT_BRAKE_SIZE ou timer.
        """
        now     = time.time()
        dt      = now - self.last_loop_time
        self.last_loop_time = now
        elapsed = now - self.push_start

        if self._goal_visible():
            size = self.aruco_size

            if elapsed > 1.5 and size >= self.DEPOT_BRAKE_SIZE:
                self._stop()
                self.get_logger().info(
                    f'[POUSSER_DROIT] Livré ! size={size:.0f}px '
                    f'dist={self.delivery_dist:.2f}m → RETOUR')
                self._change_state('RETOUR')
                return

            if size > self.DEPOT_STOP_SIZE:
                ratio = min((size - self.DEPOT_STOP_SIZE) /
                            max(self.DEPOT_BRAKE_SIZE - self.DEPOT_STOP_SIZE, 1),
                            1.0)
                speed = max(self.PUSH_DROIT_MIN,
                            self.PUSH_DROIT_SPEED * (1.0 - ratio * 0.65))
            else:
                speed = self.PUSH_DROIT_SPEED

            angular = max(-0.30, min(0.30,
                          -self.aruco_offset_px * self.ANGULAR_CORR))
            self._cmd(linear=speed, angular=angular)
            self.delivery_dist += speed * dt

        else:
            speed = self.PUSH_DROIT_MIN * 1.5
            self._cmd(linear=speed)
            self.delivery_dist += speed * dt

    def _state_retour(self):
        """Recule exactement delivery_dist."""
        elapsed    = self._elapsed()
        traveled   = self._odom_dist()
        time_limit = (self.delivery_dist / self.RECUL_SPEED) + 2.0

        done = (
            (self.has_odom and traveled >= self.delivery_dist * 0.90)
            or elapsed > time_limit
        )

        if elapsed > 3.0 and self.has_odom and traveled < 0.05:
            self.get_logger().warn('[RETOUR] Stall → fin forcée')
            done = True

        if done:
            self._stop()
            self.distance_forward = 0.0
            self.last_cube_area   = 0
            self.last_cube_color  = None
            self.target_color     = None
            self.push_color       = None
            self.goal_marker_id   = -1
            self.delivery_dist    = 0.0
            time.sleep(0.3)
            self.get_logger().info('[RETOUR] Dégagé → CHERCHER')
            self._change_state('CHERCHER')
            return

        self._cmd(linear=-self.RECUL_SPEED)

    def destroy_node(self):
        self._stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()