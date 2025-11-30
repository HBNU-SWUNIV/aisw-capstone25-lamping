#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray


class WarningSoundNode(Node):
    def __init__(self):
        super().__init__('warning_sound_node')

        # Subscriber
        self.sub = self.create_subscription(
            Detection2DArray,
            '/yolo/nearest_obstacle',
            self.callback,
            10
        )

        # State
        self.last_time = time.time()
        self.last_pos = None
        self.stationary_counter = 0

        # Parameters
        self.movement_threshold = 20.0   # px 이동 허용 범위
        self.required_duration = 2.0     # 초 단위, 정지 상태 유지 시간


    def callback(self, msg: Detection2DArray):
        if not msg.detections:
            self.stationary_counter = 0
            self.last_pos = None
            return

        det = msg.detections[0]
        if not det.results:
            return

        label_raw = det.results[0].hypothesis.class_id
        parts = label_raw.split('|')
        if len(parts) != 3:
            self.get_logger().warn(f"⚠ 잘못된 class_id 포맷: {label_raw}")
            return

        label, cx, cy = parts[0], float(parts[1]), float(parts[2])

        # 사람만 대상으로 판단
        if label != 'person':
            return

        # 이전 좌표와 비교
        if self.last_pos is not None:
            dx = abs(cx - self.last_pos[0])
            dy = abs(cy - self.last_pos[1])
            if dx + dy < self.movement_threshold:
                self.stationary_counter += 1
            else:
                self.stationary_counter = 0
        else:
            self.stationary_counter = 0

        self.last_pos = (cx, cy)

        # 일정 시간 이상 정지 시 부저 동작
        if self.stationary_counter * 0.1 >= self.required_duration:
            self.get_logger().warn("🚨 정지한 사람 감지됨 → 부저 울림!")
            self.trigger_buzzer(True)
        else:
            self.trigger_buzzer(False)

    def trigger_buzzer(self, state: bool):
        # 실제 부저 제어 코드 삽입 (GPIO HIGH/LOW, Serial 송신 등)
        if state:
            print("🔊 BUZZER ON")
        else:
            print("🔇 BUZZER OFF")


def main(args=None):
    rclpy.init(args=args)
    node = WarningSoundNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
