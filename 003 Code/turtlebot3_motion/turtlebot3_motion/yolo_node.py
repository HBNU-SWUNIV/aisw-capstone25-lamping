import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os


class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.bridge = CvBridge()

        # Camera image subscriber
        self.sub_image = self.create_subscription(
            CompressedImage,
            '/camera/image_compressed',
            self.image_callback,
            10
        )

        # Publishers
        self.pub_dynamic = self.create_publisher(Detection2DArray, '/yolo/detections_dynamic', 10)
        self.pub_static = self.create_publisher(Detection2DArray, '/yolo/detections_static', 10)
        self.pub_nearest = self.create_publisher(Detection2DArray, '/yolo/nearest_obstacle', 10)

        # Load YOLO model (GPU)
        model_path = os.path.expanduser(
            "~/ros2_ws/src/turtlebot3_motion/yolo11n.pt"
        )
        self.model = YOLO(model_path).cuda()
        self.model.conf = 0.5

    def image_callback(self, msg: CompressedImage):
        # Decode image
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if cv_image is None:
            self.get_logger().warn("❗ 이미지 디코딩 실패")
            return

        # YOLO detection
        results = self.model(cv_image)
        boxes = results[0].boxes

        # Visualization
        annotated = results[0].plot()
        cv2.imshow("YOLOv11 Detection", annotated)
        cv2.waitKey(1)

        if boxes is None or boxes.cls is None:
            return

        cls_list = boxes.cls.cpu().numpy().astype(int)
        conf_list = boxes.conf.cpu().numpy()
        xyxy_list = boxes.xyxy.cpu().numpy()

        max_area = -1
        best_det = None
        best_label = None

        dynamic_msg = Detection2DArray()
        static_msg = Detection2DArray()
        dynamic_msg.header = msg.header
        static_msg.header = msg.header

        for box, conf, cls in zip(xyxy_list, conf_list, cls_list):
            if cls not in self.model.names:
                self.get_logger().warn(f"⚠️ Unknown class ID: {cls}")
                continue

            label = self.model.names[cls]
            x1, y1, x2, y2 = box
            area = (x2 - x1) * (y2 - y1)
            cx = float((x1 + x2) / 2.0)
            cy = float((y1 + y2) / 2.0)

            det2d = Detection2D()
            det2d.header = msg.header
            det2d.bbox.size_x = float(x2 - x1)
            det2d.bbox.size_y = float(y2 - y1)

            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = f"{label}|{cx:.1f}|{cy:.1f}"
            hypo.hypothesis.score = float(conf)
            det2d.results.append(hypo)

            if area > max_area:
                max_area = area
                best_det = det2d
                best_label = label

        # Publish nearest obstacle
        if best_det:
            arr = Detection2DArray()
            arr.header = msg.header
            arr.detections.append(best_det)
            self.pub_nearest.publish(arr)

            if best_label == 'person':	
                dynamic_msg.detections.append(best_det)
                self.pub_dynamic.publish(dynamic_msg)
                self.get_logger().info(f"[Dynamic] 1 detection sent")
            else:
                static_msg.detections.append(best_det)
                self.pub_static.publish(static_msg)
                self.get_logger().info(f"[Static] 1 detection sent")

            self.get_logger().info(
                f"📤 전송: {best_label} / center=({cx:.1f}, {cy:.1f}) "
                f"/ size=({best_det.bbox.size_x:.1f}, {best_det.bbox.size_y:.1f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
