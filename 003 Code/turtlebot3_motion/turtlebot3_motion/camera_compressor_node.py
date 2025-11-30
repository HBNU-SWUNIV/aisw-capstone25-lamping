import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_compressor_node')

        # 퍼블리셔 생성
        self.publisher_ = self.create_publisher(
            CompressedImage,
            '/camera/image_compressed',
            10
        )

        # 10Hz 타이머
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 카메라 초기화 (디폴트 장치 0)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("❌ 카메라를 열 수 없습니다.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠ 프레임 캡처 실패")
            return

        # JPEG 압축
        msg = CompressedImage()
        msg.format = "jpeg"
        _, buffer = cv2.imencode('.jpg', frame)
        msg.data = buffer.tobytes()

        # 퍼블리시
        self.publisher_.publish(msg)
        self.get_logger().debug("📤 프레임 퍼블리시됨")


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
