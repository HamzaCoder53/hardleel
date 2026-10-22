import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CompressedImage

PIPELINE = (
    "libcamerasrc ! video/x-raw,width=640,height=480,framerate=30/1 "
    "! videoconvert ! video/x-raw,format=BGR "
    "! appsink drop=true max-buffers=1 sync=false"
)

class PiCam(Node):
    def __init__(self):
        super().__init__("pi_cam")
        # frames sent per second; 10 is plenty for the classifier and keeps Wi-Fi free for the lidar
        fps = float(self.declare_parameter("fps", 30.0).value)
        self.quality = int(self.declare_parameter("jpeg_quality", 80).value)
        self.pub = self.create_publisher(
            CompressedImage, "/camera/image_raw/compressed", QoSProfile(depth=1))
        self.cap = cv2.VideoCapture(PIPELINE, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open GStreamer pipeline")
        self.timer = self.create_timer(1 / fps, self.tick)

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            return
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.format = "jpeg"
        msg.data = buf.tobytes()
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = PiCam()
    try:
        rclpy.spin(node)
    finally:
        node.cap.release()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
