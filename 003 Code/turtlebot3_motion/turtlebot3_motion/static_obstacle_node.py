import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped


class StaticObstacleNode(Node):
    def __init__(self):
        super().__init__('static_obstacle_node')
        self.get_logger().info("🧱 static_obstacle_node with LiDAR started")

        # QoS
        qos_scan = QoSProfile(depth=10)
        qos_scan.reliability = ReliabilityPolicy.BEST_EFFORT

        qos_grid = QoSProfile(depth=10)
        qos_grid.reliability = ReliabilityPolicy.RELIABLE
        qos_grid.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Subscribers
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_scan)
        self.sub_yolo = self.create_subscription(Detection2DArray, '/yolo/detections_static', self.detection_callback, 10)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        # Publisher
        self.publisher = self.create_publisher(OccupancyGrid, '/virtual_obstacles', qos_grid)

        # State
        self.lidar_data = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.map_size = 200
        self.map_resolution = 0.1
        self.origin_x = 0.0
        self.origin_y = 0.0

        self.image_width = 640
        self.fov_deg = 60.0
        self.fov_rad = math.radians(self.fov_deg)

        self.wall_timeout = 30
        self.grid = np.zeros((self.map_size, self.map_size), dtype=np.int8)
        self.walls = {}

        # Timer
        self.create_timer(1.0, self.cleanup_walls)

    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg: LaserScan):
        self.lidar_data = msg

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = self.quaternion_to_yaw(q)

        self.origin_x = self.robot_x - (self.map_size * self.map_resolution) / 2.0
        self.origin_y = self.robot_y - (self.map_size * self.map_resolution) / 2.0

    def detection_callback(self, msg: Detection2DArray):
        if self.lidar_data is None:
            self.get_logger().warn('❗ LiDAR data not yet received.')
            return

        for det in msg.detections:
            try:
                label_str = det.results[0].hypothesis.class_id
                label, cx_str, _ = label_str.split('|')
                cx = float(cx_str)
            except Exception as e:
                self.get_logger().warn(f"❗ Failed to parse detection label: {e}")
                continue

            scan = self.lidar_data
            angle_min = scan.angle_min
            angle_increment = scan.angle_increment
            num_ranges = len(scan.ranges)

            angle_offset = -(cx / self.image_width - 0.5) * self.fov_rad
            fov_bias = math.radians(0)
            target_angle = angle_offset + fov_bias

            target_index = int((target_angle - angle_min) / angle_increment) % num_ranges
            if target_index < 0 or target_index >= num_ranges:
                self.get_logger().warn(f"⚠ Invalid LiDAR index: {target_index}")
                continue

            distance = scan.ranges[target_index]
            self.get_logger().info(f"📏 LiDAR 거리 = {distance:.2f}m, index = {target_index}")

            if math.isnan(distance) or distance <= 0.0:
                self.get_logger().warn(f"⚠ Invalid LiDAR distance at index {target_index}")
                continue

            # 로컬 -> 월드 좌표 변환
            local_x = distance * math.cos(target_angle)
            local_y = distance * math.sin(target_angle)
            world_x = self.robot_x + (local_x * math.cos(self.robot_yaw) - local_y * math.sin(self.robot_yaw))
            world_y = self.robot_y + (local_x * math.sin(self.robot_yaw) + local_y * math.cos(self.robot_yaw))

            grid_x = int((world_x - self.origin_x) / self.map_resolution)
            grid_y = int((world_y - self.origin_y) / self.map_resolution)

            if not (0 <= grid_x < self.map_size and 0 <= grid_y < self.map_size):
                self.get_logger().warn(f"❗ Invalid grid ({grid_x}, {grid_y})")
                continue

            key = (grid_x, grid_y)
            if key not in self.walls:
                self.get_logger().info(f"🧱 Adding wall at grid {key}")
                self.add_wall(grid_x, grid_y)

    def add_wall(self, x, y):
        self.grid[y, x] = 100
        self.walls[(x, y)] = time.time()
        self.publish_grid()

    def publish_grid(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_size
        msg.info.height = self.map_size
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = self.grid.flatten().tolist()
        self.publisher.publish(msg)

    def cleanup_walls(self):
        now = time.time()
        expired = [key for key, t in self.walls.items() if now - t > self.wall_timeout]
        for key in expired:
            x, y = key
            self.grid[y, x] = 0
            del self.walls[key]


def main(args=None):
    rclpy.init(args=args)
    node = StaticObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
