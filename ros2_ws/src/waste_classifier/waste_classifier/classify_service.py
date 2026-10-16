import os
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from waste_classifier.classifier import CLASS_TO_COLOR, WasteModel, center_square, decide

PIPELINE = (
    "libcamerasrc ! video/x-raw,width=640,height=480,framerate=30/1 "
    "! videoconvert ! video/x-raw,format=BGR "
    "! appsink drop=true max-buffers=1 sync=false"
)


class ClassifyService(Node):
    """/classify (std_srvs/Trigger): grab frames, classify, answer "<class>:<confidence>:<bay colour>".

    class is plastic | metal | organic | unknown (unknown = nothing confident in view).
    Also publishes /waste/class and /waste/color (std_msgs/String) for the mission node.
    """

    def __init__(self):
        super().__init__('waste_classifier')
        share = get_package_share_directory('waste_classifier')
        self.declare_parameter('model_path', f'{share}/models/waste_model.tflite')
        self.declare_parameter('labels_path', f'{share}/models/labels.txt')
        self.declare_parameter('norm', 'raw')
        self.declare_parameter('warmup_frames', 20)      # let auto-exposure settle
        self.declare_parameter('vote_frames', 5)         # frames averaged per answer
        self.declare_parameter('min_conf', 0.75)
        self.declare_parameter('min_margin', 0.25)
        self.declare_parameter('save_dir', '')           # save every capture (for retraining data)

        p = lambda n: self.get_parameter(n).value
        self.model = WasteModel(p('model_path'), p('labels_path'), norm=p('norm'))
        self.warmup, self.votes = int(p('warmup_frames')), int(p('vote_frames'))
        self.min_conf, self.min_margin = float(p('min_conf')), float(p('min_margin'))
        self.save_dir = p('save_dir')
        if self.save_dir:
            os.makedirs(self.save_dir, exist_ok=True)

        self.pub_class = self.create_publisher(String, '/waste/class', 10)
        self.pub_color = self.create_publisher(String, '/waste/color', 10)
        self.create_service(Trigger, 'classify', self.classify_callback)
        self.get_logger().info(
            f'Ready. Labels: {self.model.labels}. Call the /classify service. '
            f'Accept if conf >= {self.min_conf} and margin >= {self.min_margin}.')

    def grab(self):
        """Return `vote_frames` frames taken after the warm-up, or [] on camera error."""
        cap = cv2.VideoCapture(PIPELINE, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            return []
        frames = []
        try:
            for i in range(self.warmup + self.votes):
                ok, f = cap.read()
                if ok and i >= self.warmup:
                    frames.append(f)
        finally:
            cap.release()
        return frames

    def classify_callback(self, request, response):
        t0 = time.time()
        try:
            frames = self.grab()
            if not frames:
                response.success = False
                response.message = 'camera error'
                return response
            crops = [center_square(f) for f in frames]
            probs = np.mean([self.model.predict(c)[2] for c in crops], axis=0)
            label, conf = decide(self.model.labels, probs, self.min_conf, self.min_margin)
            color = CLASS_TO_COLOR.get(label, 'unknown')
            if self.save_dir:
                name = f'{time.strftime("%Y%m%d_%H%M%S")}_{label}.jpg'
                cv2.imwrite(os.path.join(self.save_dir, name), crops[-1])
            self.pub_class.publish(String(data=label))
            self.pub_color.publish(String(data=color))
            response.success = label != 'unknown'
            response.message = f'{label}:{conf:.2f}:{color}'
            self.get_logger().info(
                f'Result: {response.message}  probs={np.round(probs, 2).tolist()}  ({time.time() - t0:.1f} s)')
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
