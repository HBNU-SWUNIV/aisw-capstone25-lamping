#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import yaml
import time
from typing import Dict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String

POSE_DB = os.path.expanduser("~/.ros/saved_poses.yaml")
MAP_FRAME = "map"

class TB3BookmarkBridge(Node):
    def __init__(self):
        super().__init__("tb3_bookmark_bridge")

        # 최신 AMCL 포즈 보관
        self._amcl_msg = None
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_cb,
            10
        )

        # 저장/이동 명령 구독 (이름을 문자열로 받음)
        self._save_sub = self.create_subscription(
            String, "/save_pose", self._on_save, 10
        )
        self._goto_sub = self.create_subscription(
            String, "/goto_pose", self._on_goto, 10
        )

        # Nav2가 구독하는 목표 포즈 토픽
        self._goal_pub = self.create_publisher(
            PoseStamped, "/goal_pose", 10
        )

        # 주기적으로 AMCL 수신 여부 로깅(선택)
        self._last_log = time.time()

        self.get_logger().info("TB3 Bookmark Bridge ready: /save_pose, /goto_pose, /goal_pose")

    # === 콜백들 ===
    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self._amcl_msg = msg
        # 너무 시끄럽지 않게 5초마다 상태 출력
        now = time.time()
        if now - self._last_log > 5.0:
            self.get_logger().info("AMCL pose OK (frame=%s)" % (msg.header.frame_id or ""))
            self._last_log = now

    def _on_save(self, name_msg: String):
        name = (name_msg.data or "").strip()
        if not name:
            self.get_logger().warn("/save_pose: not save empty name.")
            return
        if self._amcl_msg is None:
            self.get_logger().error("/save_pose: not yet received /amcl_pose.")
            return

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self._amcl_msg.header.frame_id or MAP_FRAME
        ps.pose = self._amcl_msg.pose.pose

        db = self._load_db()
        db[name] = {
            "frame_id": ps.header.frame_id,
            "x": ps.pose.position.x,
            "y": ps.pose.position.y,
            "z": ps.pose.position.z,
            "qx": ps.pose.orientation.x,
            "qy": ps.pose.orientation.y,
            "qz": ps.pose.orientation.z,
            "qw": ps.pose.orientation.w,
        }
        self._save_db(db)
        self.get_logger().info(f"complete: '{name}' → ({ps.pose.position.x:.3f}, {ps.pose.position.y:.3f}) frame={ps.header.frame_id}")

    def _on_goto(self, name_msg: String):
        name = (name_msg.data or "").strip()
        if not name:
            self.get_logger().warn("cancel /goto_pose:")
            return

        db = self._load_db()
        if name not in db:
            self.get_logger().error(f"not /goto_pose: '{name}' ")
            return
        v = db[name]

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = v.get("frame_id", MAP_FRAME)
        goal.pose.position.x = float(v["x"])
        goal.pose.position.y = float(v["y"])
        goal.pose.position.z = float(v["z"])
        goal.pose.orientation.x = float(v["qx"])
        goal.pose.orientation.y = float(v["qy"])
        goal.pose.orientation.z = float(v["qz"])
        goal.pose.orientation.w = float(v["qw"])

        self._goal_pub.publish(goal)
        self.get_logger().info(f"/goal_pose ← '{name}'")

    # === YAML DB ===
    def _load_db(self) -> Dict:
        if os.path.exists(POSE_DB):
            with open(POSE_DB, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        return {}

    def _save_db(self, db: Dict):
        os.makedirs(os.path.dirname(POSE_DB), exist_ok=True)
        with open(POSE_DB, "w", encoding="utf-8") as f:
            yaml.safe_dump(db, f, allow_unicode=True, sort_keys=True)

def main():
    rclpy.init()
    node = TB3BookmarkBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
