import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraCheck(Node):
    def __init__(self):
        super().__init__('camera_check')

        # 퍼블리셔 생성 (비압축 이미지)
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)

        # 10Hz 타이머
        self.timer = self.create_timer(0.1, self.timer_callback)

        # OpenCV <-> ROS 변환용 브릿지
        self.bridge = CvBridge()

        # 카메라 초기화
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("fail.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("down")
            return

        # OpenCV 이미지를 ROS Image 메시지로 변환
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

        # 퍼블리시
        self.publisher_.publish(msg)
        self.get_logger().debug("good")


def main(args=None):
    rclpy.init(args=args)
    node = CameraCheck()
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
