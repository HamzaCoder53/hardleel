#!/usr/bin/env python3
"""
NOT USED - the installed executable is `classify_service` (classify_service.py). Kept for reference only:
its picamera2 / rpicam-still backends are not installed on this Pi (it uses GStreamer libcamerasrc).

waste_classifier_service.py - ON-DEMAND waste classification (camera is NOT running all the time)

Each call to the service:  open camera -> let exposure/focus settle -> grab 1 frame
                           -> close camera -> classify -> return color

    plastic -> blue      organic -> white      metal -> black      (nothing in view -> "none")

Service    : /classify_waste   (std_srvs/Trigger)   response.message = "blue" | "white" | "black" | "none" | "unknown"
Publishes  : /waste/class, /waste/color            (std_msgs/String, same result)

Do NOT run camera_ros at the same time - only one program can own the camera.

Camera backends (parameter `backend`):
    picamera2  Python library (python3-picamera2). Fastest, keeps everything in Python.
    rpicam     calls `rpicam-still` (or `libcamera-still`) as a subprocess. Works if those tools exist.
    auto       try picamera2 first, then rpicam   (default)

Parameters: model_path, labels_path (labels.txt from training - recommended), labels, backend, width, height, settle_time, conf_threshold,
            crop_square, save_dir (save every captured frame, handy for collecting retraining data)
"""
import os
import shutil
import subprocess
import tempfile
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from tflite_runtime.interpreter import Interpreter
except ImportError:
    try:
        from ai_edge_litert.interpreter import Interpreter
    except ImportError:
        from tensorflow.lite.python.interpreter import Interpreter

CLASS_TO_COLOR = {"plastic": "blue", "organic": "white", "metal": "black", "empty": "none"}


class WasteClassifierService(Node):
    def __init__(self):
        super().__init__("waste_classifier_service")
        self.declare_parameter("model_path", "/home/pi/waste_model.tflite")
        self.declare_parameter("labels_path", "")        # path to labels.txt written by the training script
        self.declare_parameter("labels", ["metal", "organic", "plastic"])   # used only if labels_path is empty
        self.declare_parameter("backend", "auto")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("settle_time", 1.0)       # seconds for exposure / autofocus to settle
        self.declare_parameter("conf_threshold", 0.60)
        self.declare_parameter("crop_square", True)      # centre-crop to a square (training images were square)
        self.declare_parameter("save_dir", "")

        p = lambda n: self.get_parameter(n).value
        if p("labels_path"):
            with open(p("labels_path")) as f:
                self.labels = [ln.strip() for ln in f if ln.strip()]
        else:
            self.labels = list(p("labels"))
        self.backend = p("backend")
        self.w, self.h = int(p("width")), int(p("height"))
        self.settle = float(p("settle_time"))
        self.thresh = float(p("conf_threshold"))
        self.crop_square = bool(p("crop_square"))
        self.save_dir = p("save_dir")
        if self.save_dir:
            os.makedirs(self.save_dir, exist_ok=True)

        self.interp = Interpreter(model_path=p("model_path"), num_threads=4)
        self.interp.allocate_tensors()
        self.inp = self.interp.get_input_details()[0]
        self.out = self.interp.get_output_details()[0]
        self.in_h, self.in_w = self.inp["shape"][1:3]
        n_out = int(self.out["shape"][-1])
        if n_out != len(self.labels):
            raise RuntimeError(f"model has {n_out} outputs but {len(self.labels)} labels {self.labels} - "
                               "pass the labels.txt from training with -p labels_path:=...")

        self.lock = threading.Lock()
        self.pub_class = self.create_publisher(String, "/waste/class", 10)
        self.pub_color = self.create_publisher(String, "/waste/color", 10)
        self.create_service(Trigger, "/classify_waste", self._on_request)
        self.get_logger().info(f"Ready. Call /classify_waste. labels={self.labels} backend={self.backend}")

    # ---------------- camera (opened only for the duration of one capture) ----------------
    def _capture_picamera2(self) -> np.ndarray:
        from picamera2 import Picamera2
        cam = Picamera2()
        try:
            cfg = cam.create_still_configuration(main={"size": (self.w, self.h), "format": "RGB888"})
            cam.configure(cfg)
            cam.start()
            try:                                   # autofocus on Camera Module 3; ignored on fixed-focus cams
                cam.autofocus_cycle()
            except Exception:
                pass
            time.sleep(self.settle)
            frame = cam.capture_array()            # "RGB888" is stored B,G,R in memory
            return cv2.cvtColor(frame[:, :, :3], cv2.COLOR_BGR2RGB)
        finally:
            cam.stop()
            cam.close()                            # releases the camera for the next call

    def _capture_rpicam(self) -> np.ndarray:
        exe = shutil.which("rpicam-still") or shutil.which("libcamera-still")
        if exe is None:
            raise RuntimeError("neither rpicam-still nor libcamera-still found")
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "frame.jpg")
            subprocess.run([exe, "-n", "-t", str(int(self.settle * 1000) + 500),
                            "--width", str(self.w), "--height", str(self.h), "-o", path],
                           check=True, timeout=15, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            bgr = cv2.imread(path)
        if bgr is None:
            raise RuntimeError("rpicam-still produced no image")
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def capture(self) -> np.ndarray:
        order = {"picamera2": [self._capture_picamera2], "rpicam": [self._capture_rpicam],
                 "auto": [self._capture_picamera2, self._capture_rpicam]}[self.backend]
        err = None
        for fn in order:
            try:
                return fn()
            except Exception as e:                 # try next backend
                err = e
                self.get_logger().warn(f"{fn.__name__} failed: {e}")
        raise RuntimeError(f"camera capture failed: {err}")

    # ---------------- classification ----------------
    def classify(self, rgb: np.ndarray):
        if self.crop_square:
            h, w = rgb.shape[:2]
            s = min(h, w)
            y0, x0 = (h - s) // 2, (w - s) // 2
            rgb = rgb[y0:y0 + s, x0:x0 + s]
        x = cv2.resize(rgb, (self.in_w, self.in_h), interpolation=cv2.INTER_AREA)
        x = x.astype(np.uint8 if self.inp["dtype"] == np.uint8 else np.float32)   # raw 0-255, scaling is inside the model
        self.interp.set_tensor(self.inp["index"], x[None, ...])
        self.interp.invoke()
        probs = self.interp.get_tensor(self.out["index"])[0].astype(np.float32)
        if self.out["dtype"] == np.uint8:
            scale, zp = self.out["quantization"]
            probs = (probs - zp) * scale
        i = int(np.argmax(probs))
        return self.labels[i], float(probs[i]), probs

    def _on_request(self, req, res):
        with self.lock:                            # one capture at a time
            t0 = time.time()
            try:
                rgb = self.capture()
            except Exception as e:
                self.get_logger().error(str(e))
                res.success, res.message = False, f"camera error: {e}"
                return res
            label, conf, probs = self.classify(rgb)
            if conf < self.thresh:
                label = "unknown"
            color = CLASS_TO_COLOR.get(label, "unknown")

            if self.save_dir:
                cv2.imwrite(os.path.join(self.save_dir, f"{int(time.time())}_{label}.jpg"),
                            cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            self.pub_class.publish(String(data=label))
            self.pub_color.publish(String(data=color))
            self.get_logger().info(
                f"{label} ({conf:.2f}) -> {color}   probs={np.round(probs, 2).tolist()}   took {time.time() - t0:.1f}s")
            res.success = color not in ("unknown", "none")
            res.message = color
            return res


def main():
    rclpy.init()
    node = WasteClassifierService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
