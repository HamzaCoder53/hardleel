"""Pi Camera Module 3 -> /camera/image_raw/compressed (JPEG).

Params: width/height (320x240), fps (5), jpeg_quality (80), rotate_deg (0|90|180|270, for a camera mounted
rolled). The sensor is asked for width x height at fps directly so the Pi does not decode/encode frames it
throws away (30 fps 640x480 used ~70 % of a core). If the camera refuses that mode, it falls back to 640x480
and scales down in software.
"""
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CompressedImage

PIPELINE = (
    "libcamerasrc ! video/x-raw,width={w},height={h},framerate={fps}/1 "
    "! videoconvert ! video/x-raw,format=BGR "
    "! appsink drop=true max-buffers=1 sync=false"
)
ROTATE = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}


class PiCam(Node):
    def __init__(self):
        super().__init__("pi_cam")
        p = self.declare_parameter
        fps = float(p("fps", 5.0).value)
        self.size = (int(p("width", 320).value), int(p("height", 240).value))
        self.quality = int(p("jpeg_quality", 80).value)
        rot = int(p("rotate_deg", 0).value) % 360
        self.rotate = ROTATE.get(rot)
        self.pub = self.create_publisher(
            CompressedImage, "/camera/image_raw/compressed", QoSProfile(depth=1))
        rate = max(1, int(round(fps)))
        self.cap = cv2.VideoCapture(PIPELINE.format(w=self.size[0], h=self.size[1], fps=rate), cv2.CAP_GSTREAMER)
        self.resize = False
        if not self.cap.isOpened():
            self.get_logger().warn(f"camera refused {self.size[0]}x{self.size[1]}@{rate}: using 640x480 + resize")
            self.cap = cv2.VideoCapture(PIPELINE.format(w=640, h=480, fps=rate), cv2.CAP_GSTREAMER)
            self.resize = True
        if not self.cap.isOpened():
            raise RuntimeError("Could not open GStreamer pipeline")
        self.get_logger().info(f"camera {self.size[0]}x{self.size[1]} @ {fps} fps, rotate={rot} deg")
        self.timer = self.create_timer(1 / fps, self.tick)

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            return
        if self.resize:
            frame = cv2.resize(frame, self.size, interpolation=cv2.INTER_AREA)
        if self.rotate is not None:
            frame = cv2.rotate(frame, self.rotate)
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
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
