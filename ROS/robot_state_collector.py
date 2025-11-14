#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from tm_msgs.msg import FeedbackState
import math
import time


class SingleRobotStateCollector(Node):

    def __init__(self, fps: int = 30):
        super().__init__('single_robot_state_collector')

        self.interval = 1.0 / max(1, fps)
        self.last_emit = 0.0

        self.joints_deg = None
        self.io = [0, 0, 0]

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 訂閱 joint_states
        self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint,
            qos_sensor
        )

        # 訂閱 feedback_states（取得 DO）
        self.create_subscription(
            FeedbackState,
            '/feedback_states',
            self._on_fb,
            10
        )

    def _on_joint(self, msg: JointState):
        if not msg.position or len(msg.position) < 6:
            return

        j6 = msg.position[:6]  # rad
        joints_deg = [round(math.degrees(j), 2) for j in j6]

        if all(j == 0.0 for j in joints_deg):
            return

        self.joints_deg = joints_deg

    def _on_fb(self, msg: FeedbackState):
        ee = list(msg.ee_digital_output) if msg.ee_digital_output else []
        cb = list(msg.cb_digital_output) if msg.cb_digital_output else []
        src = ee if ee else cb

        a = int(src[0]) if len(src) > 0 else 0
        b = int(src[1]) if len(src) > 1 else 0
        c = int(src[2]) if len(src) > 2 else 0
        self.io = [a, b, c]
