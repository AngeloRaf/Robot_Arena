#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gopigo3_driver.py — Pont ROS 2 ↔ GoPiGo3
Adapté de : robot_controller/gopigo3_driver.py (Damien)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
import easygopigo3 as easy
import time
import json
import math


class GoPiGo3Driver(Node):

    WHEEL_BASE = 0.117
    WHEEL_CIRC = 0.066 * math.pi
    RAMP_RATE  = 50
    MAX_SPEED  = 300
    SCALE      = 300

    def __init__(self):
        super().__init__('gopigo3_driver')

        self.gpg = easy.EasyGoPiGo3()
        self.gpg.set_speed(0)
        self.gpg.reset_encoders()

        self.current_left  = 0
        self.current_right = 0
        self.target_left   = 0
        self.target_right  = 0

        enc = self.gpg.read_encoders()
        self.left_enc_prev  = enc[0]
        self.right_enc_prev = enc[1]
        self.pos_x   = 0.0
        self.pos_y   = 0.0
        self.heading = 0.0

        self.last_cmd_time = time.time()

        self.create_subscription(Twist, '/cmd_vel', self._cb_cmd_vel, 10)
        self.pub_battery = self.create_publisher(Float32, '/battery', 10)
        self.pub_odom    = self.create_publisher(String,  '/odom_simple', 10)

        self.create_timer(0.05, self._motor_loop)
        self.create_timer(0.05, self._publish_odom)
        self.create_timer(5.0,  self._publish_battery)

        self._publish_battery()
        self.get_logger().info('gopigo3_driver pret')

    def _cb_cmd_vel(self, msg):
        linear  = msg.linear.x
        angular = msg.angular.z

        left  = linear - (angular * self.WHEEL_BASE / 2)
        right = linear + (angular * self.WHEEL_BASE / 2)

        self.target_left  = int(-left  * self.SCALE)
        self.target_right = int(-right * self.SCALE)

        self.target_left  = max(-self.MAX_SPEED, min(self.MAX_SPEED, self.target_left))
        self.target_right = max(-self.MAX_SPEED, min(self.MAX_SPEED, self.target_right))

        self.last_cmd_time = time.time()

    def _ramp(self, current, target):
        diff = target - current
        if abs(diff) <= self.RAMP_RATE:
            return target
        return current + self.RAMP_RATE if diff > 0 else current - self.RAMP_RATE

    def _motor_loop(self):
        if time.time() - self.last_cmd_time > 1.0:
            self.target_left  = 0
            self.target_right = 0

        self.current_left  = self._ramp(self.current_left,  self.target_left)
        self.current_right = self._ramp(self.current_right, self.target_right)

        # NOTE : LEFT/RIGHT inversés physiquement sur le GoPiGo3
        self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT,  self.current_right)
        self.gpg.set_motor_dps(self.gpg.MOTOR_RIGHT, self.current_left)

    def _publish_odom(self):
        try:
            left_enc, right_enc = self.gpg.read_encoders()

            dl = (left_enc  - self.left_enc_prev)  / 360.0 * self.WHEEL_CIRC
            dr = (right_enc - self.right_enc_prev) / 360.0 * self.WHEEL_CIRC

            self.left_enc_prev  = left_enc
            self.right_enc_prev = right_enc

            d_center = (dl + dr) / 2.0
            d_theta  = (dr - dl) / self.WHEEL_BASE

            self.heading += d_theta
            while self.heading >  math.pi: self.heading -= 2 * math.pi
            while self.heading < -math.pi: self.heading += 2 * math.pi

            self.pos_x += d_center * math.cos(self.heading)
            self.pos_y += d_center * math.sin(self.heading)

            msg = String()
            msg.data = json.dumps({
                'x':           round(self.pos_x, 3),
                'y':           round(self.pos_y, 3),
                'heading_deg': round(math.degrees(self.heading), 1),
                'left_enc':    left_enc,
                'right_enc':   right_enc,
            })
            self.pub_odom.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Odom erreur: {e}')

    def _publish_battery(self):
        msg = Float32()
        msg.data = float(self.gpg.volt())
        self.pub_battery.publish(msg)
        self.get_logger().info(f'Batterie: {msg.data:.1f}V')

    def destroy_node(self):
        self.gpg.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GoPiGo3Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()