import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2


class SimCameraBridge(Node):
    def __init__(self):
        super().__init__('sim_camera_bridge')

        self.bridge = CvBridge()

        # Gazebo 카메라 토픽 구독
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # YOLO가 쓰는 압축 이미지 퍼블리시
        self.pub = self.create_publisher(
            CompressedImage,
            '/camera/image_compressed',
            10
        )

    def image_callback(self, msg: Image):
        # ROS Image → OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # OpenCV → CompressedImage
        msg_out = CompressedImage()
        msg_out.format = "jpeg"
        _, buffer = cv2.imencode('.jpg', cv_image)
        msg_out.data = buffer.tobytes()

        self.pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = SimCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
