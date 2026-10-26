# Implementation Plan: End-to-End Waste Classifier Monitor Test (`clip_monitor_test.py`)

## Goal Description
Develop `clip_monitor_test.py` for ROS 2 Jazzy (Python 3.12, rclpy) to test the robot's zero-shot CLIP waste classifier end-to-end. The robot's forward camera points at a laptop monitor running this script. The script displays test images full-screen one after another on the monitor, invokes the robot's `/classify` ROS 2 service for each image, collects the response, monitors optional topic `/classifier/result`, tracks accuracy and latency, displays real-time progress, and exports final metrics and confusion tables to stdout and a CSV file.

## Interfaces & Architecture
- **ROS 2 Service Client**: `/classify` (`std_srvs/srv/Trigger`).
  - Response `message`: `"<label>:<confidence>:<colour>"`.
  - Response `success`: `True` if successful, `False` if no camera frames captured.
- **ROS 2 Topic Subscriber**: `/classifier/result` (`std_msgs/msg/String`, JSON payload).
  - Keys: `label`, `confidence`, `per_label`, `latency_s`. Subscribed passively to capture rich per-class probabilities without blocking.
- **Display Pipeline**: OpenCV (`cv2.namedWindow`, `cv2.WINDOW_NORMAL`, `cv2.WND_PROP_FULLSCREEN`).
  - Aspect-ratio preserving letterboxing onto a black canvas matching screen or window dimensions.
  - Compatible with WSLg (X11/Wayland).
  - Responsive event loop handling keyboard controls (`q`/Esc to quit early while preserving results, space to pause/resume).
- **Interactive Sequence**:
  1. 5-second countdown on screen ("Aim the robot camera at this screen - starting in N s").
  2. For each image (repeated `--repeats` times):
     - Display letterboxed image.
     - Settle wait for `--settle` seconds.
     - Asynchronous `/classify` service call with `--timeout`.
     - Output real-time status line per image.
  3. Final reporting:
     - Scored accuracy, mean and max round-trip latency.
     - Confusion matrix (expected x predicted, including `unknown`).
     - List of classification misses.
     - CSV output containing all test results.

## Verification
- Code syntax and lint check.
- Parameter validation (`--settle`, `--timeout`, `--repeats`, `--out`, `--window-size`, `--no-fullscreen`).
- Graceful error handling if `/classify` is not available after 10 s.
