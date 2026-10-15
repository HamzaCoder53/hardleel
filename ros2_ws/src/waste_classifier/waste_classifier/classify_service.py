import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_srvs.srv import Trigger

from waste_classifier.classifier import WasteModel

PIPELINE = (
    "libcamerasrc ! video/x-raw,width=640,height=480,framerate=30/1 "
    "! videoconvert ! video/x-raw,format=BGR "
    "! appsink drop=true max-buffers=1 sync=false"
)


class ClassifyService(Node):
    def __init__(self):
        super().__init__('waste_classifier')
        share = get_package_share_directory('waste_classifier')
        self.declare_parameter('model_path', f'{share}/models/waste_model.tflite')
        self.declare_parameter('labels_path', f'{share}/models/labels.txt')
        self.declare_parameter('norm', 'raw')
        self.declare_parameter('warmup_frames', 20)

        self.model = WasteModel(
            self.get_parameter('model_path').value,
            self.get_parameter('labels_path').value,
            norm=self.get_parameter('norm').value,
        )
        self.warmup = self.get_parameter('warmup_frames').value
        self.create_service(Trigger, 'classify', self.classify_callback)
        self.get_logger().info(
            f'Ready. Labels: {self.model.labels}. Call the /classify service.')

    def grab(self):
        cap = cv2.VideoCapture(PIPELINE, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            return None
        frame = None
        try:
            for _ in range(self.warmup):  # let auto-exposure settle
                ok, f = cap.read()
                if ok:
                    frame = f
        finally:
            cap.release()
        return frame

    def classify_callback(self, request, response):
        try:
            frame = self.grab()
            if frame is None:
                response.success = False
                response.message = 'camera error'
                return response
            label, conf, _ = self.model.predict(frame)
            response.success = True
            response.message = f'{label}:{conf:.2f}'
            self.get_logger().info(f'Result: {response.message}')
        except Exception as e:
            response.success = False
            response.message = f'error: {e}'
            self.get_logger().error(response.message)
        return response


def main():
    rclpy.init()
    node = ClassifyService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
