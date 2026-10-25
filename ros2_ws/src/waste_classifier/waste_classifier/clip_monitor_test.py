#!/usr/bin/env python3
"""clip_monitor_test.py - End-to-end monitor test for robot waste classifier.

Shows test images full-screen on a monitor facing the robot's camera, invokes
the ROS 2 /classify service for each image, and evaluates classification accuracy.

Usage:
    python3 clip_monitor_test.py <folder> [--settle 1.5] [--timeout 10]
                                 [--repeats 1] [--out results.csv]
                                 [--window-size WxH] [--no-fullscreen]
"""

import argparse
import csv
import json
import os
import sys
import time
from collections import Counter

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

VALID_EXTS = ('.jpg', '.jpeg', '.png', '.webp')
VALID_EXPECTED = {'plastic', 'metal', 'organic', 'fabric', 'empty'}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='End-to-end waste classifier monitor tester for ROS 2 Jazzy.'
    )
    parser.add_argument('folder', help='Path to test images folder')
    parser.add_argument('--settle', type=float, default=1.5,
                        help='Settle time in seconds before calling /classify (default: 1.5)')
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='Service call timeout in seconds (default: 10.0)')
    parser.add_argument('--repeats', type=int, default=1,
                        help='Number of repetitions per image (default: 1)')
    parser.add_argument('--out', default='results.csv',
                        help='Output CSV file path (default: results.csv)')
    parser.add_argument('--window-size', default=None,
                        help='Display window resolution WxH, e.g. 1920x1080')
    parser.add_argument('--no-fullscreen', action='store_true',
                        help='Disable OpenCV fullscreen mode')
    return parser.parse_args()


def extract_expected_label(filepath: str) -> str:
    """Extract expected label from filename prefix before the first '_' or '-'."""
    stem = os.path.splitext(os.path.basename(filepath))[0]
    idx_u = stem.find('_')
    idx_d = stem.find('-')
    indices = [i for i in (idx_u, idx_d) if i != -1]
    prefix = stem[:min(indices)].lower() if indices else stem.lower()
    return prefix if prefix in VALID_EXPECTED else ''


def letterbox(img: np.ndarray, target_w: int, target_h: int) -> np.ndarray:
    """Scale image to fit target resolution on a black canvas without distortion."""
    ih, iw = img.shape[:2]
    if ih <= 0 or iw <= 0 or target_w <= 0 or target_h <= 0:
        return np.zeros((max(1, target_h), max(1, target_w), 3), dtype=np.uint8)
    scale = min(target_w / iw, target_h / ih)
    nw = max(1, int(round(iw * scale)))
    nh = max(1, int(round(ih * scale)))
    interp = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(img, (nw, nh), interpolation=interp)
    canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)
    ox = (target_w - nw) // 2
    oy = (target_h - nh) // 2
    canvas[oy:oy + nh, ox:ox + nw] = resized
    return canvas


def draw_centered_text(canvas: np.ndarray, text: str, font_scale: float = 1.0,
                       color: tuple = (255, 255, 255), thickness: int = 2):
    """Draw centered text line onto canvas."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), _ = cv2.getTextSize(text, font, font_scale, thickness)
    ch, cw = canvas.shape[:2]
    x = max(10, (cw - tw) // 2)
    y = max(th + 10, (ch + th) // 2)
    cv2.putText(canvas, text, (x, y), font, font_scale, color, thickness, cv2.LINE_AA)


def get_window_size(win_name: str, fallback_w: int = 1920, fallback_h: int = 1080) -> tuple[int, int]:
    """Query actual OpenCV window dimensions or return fallbacks."""
    try:
        rect = cv2.getWindowImageRect(win_name)
        if rect[2] > 50 and rect[3] > 50:
            return rect[2], rect[3]
    except Exception:
        pass
    return fallback_w, fallback_h


def wait_responsive(duration_s: float, win_name: str, current_canvas: np.ndarray) -> bool:
    """Wait duration_s handling UI events. Returns True if user requested early exit."""
    t_end = time.monotonic() + max(0.0, duration_s)
    while time.monotonic() < t_end:
        rem_ms = max(1, min(25, int((t_end - time.monotonic()) * 1000)))
        k = cv2.waitKey(rem_ms) & 0xFF
        if k in (27, ord('q'), ord('Q')):
            return True
        if k == ord(' '):
            # Pause loop
            pause_start = time.monotonic()
            rem = max(0.0, t_end - pause_start)
            paused_canvas = current_canvas.copy()
            draw_centered_text(paused_canvas, 'PAUSED - Press SPACE to resume, Q/Esc to quit',
                               font_scale=0.9, color=(0, 255, 255), thickness=2)
            cv2.imshow(win_name, paused_canvas)
            while True:
                pk = cv2.waitKey(50) & 0xFF
                if pk in (27, ord('q'), ord('Q')):
                    return True
                if pk == ord(' '):
                    cv2.imshow(win_name, current_canvas)
                    t_end = time.monotonic() + rem
                    break
    return False


class MonitorTestNode(Node):
    """ROS 2 node interacting with /classify and /classifier/result."""

    def __init__(self):
        super().__init__('clip_monitor_test')
        self.cli = self.create_client(Trigger, '/classify')
        self.sub = self.create_subscription(String, '/classifier/result', self._on_result, 10)
        self.latest_topic_json = None

    def _on_result(self, msg: String):
        self.latest_topic_json = msg.data


def main() -> int:
    args = parse_args()

    if not os.path.isdir(args.folder):
        print(f"ERROR: Folder '{args.folder}' does not exist.", file=sys.stderr)
        return 1

    img_files = sorted([
        os.path.join(args.folder, f) for f in os.listdir(args.folder)
        if os.path.splitext(f)[1].lower() in VALID_EXTS
    ])
    if not img_files:
        print(f"ERROR: No images (*.jpg, *.jpeg, *.png, *.webp) found in '{args.folder}'.", file=sys.stderr)
        return 1

    # Target window resolution
    target_w, target_h = 1920, 1080
    if args.window_size:
        try:
            ws, hs = args.window_size.lower().split('x')
            target_w, target_h = int(ws), int(hs)
        except Exception:
            print(f"WARNING: Invalid --window-size '{args.window_size}', using 1920x1080.", file=sys.stderr)

    # Initialize ROS 2
    rclpy.init()
    node = MonitorTestNode()

    # Setup display
    win_name = 'Waste Classifier Monitor Test'
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    if not args.no_fullscreen:
        cv2.setWindowProperty(win_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    elif args.window_size:
        cv2.resizeWindow(win_name, target_w, target_h)

    records = []
    try:
        # Check service availability
        print("Connecting to service '/classify' (waiting up to 10 s)...")
        if not node.cli.wait_for_service(timeout_sec=10.0):
            print("ERROR: Service '/classify' not available after 10 s. Exiting.", file=sys.stderr)
            return 1
        print("Connected to classifier service.\n")

        # 5-second positioning countdown
        w, h = get_window_size(win_name, target_w, target_h)
        for s in range(5, 0, -1):
            w, h = get_window_size(win_name, w, h)
            cd_canvas = np.zeros((h, w, 3), dtype=np.uint8)
            msg = f'Aim the robot camera at this screen - starting in {s} s'
            fscale = max(0.7, min(w / 1280.0, 1.5))
            draw_centered_text(cd_canvas, msg, font_scale=fscale, color=(255, 255, 255), thickness=2)
            cv2.imshow(win_name, cd_canvas)
            if wait_responsive(1.0, win_name, cd_canvas):
                print('\nTest aborted by user during countdown.')
                return 0

        total_runs = len(img_files) * max(1, args.repeats)
        cur_idx = 0
        user_aborted = False

        # Evaluation loop
        for img_path in img_files:
            fname = os.path.basename(img_path)
            expected = extract_expected_label(img_path)
            raw_img = cv2.imread(img_path)
            if raw_img is None:
                print(f"WARNING: Failed to read image '{img_path}', skipping.", file=sys.stderr)
                continue

            for _ in range(max(1, args.repeats)):
                cur_idx += 1
                w, h = get_window_size(win_name, target_w, target_h)
                canvas = letterbox(raw_img, w, h)
                cv2.imshow(win_name, canvas)

                # Wait settle time
                if wait_responsive(args.settle, win_name, canvas):
                    user_aborted = True
                    break

                # Call service asynchronously and spin
                node.latest_topic_json = None
                req = Trigger.Request()
                t_call = time.perf_counter()
                future = node.cli.call_async(req)

                rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
                lat_s = time.perf_counter() - t_call

                if future.done():
                    try:
                        res = future.result()
                        srv_ok = res.success
                        msg = res.message
                    except Exception as err:
                        srv_ok = False
                        msg = f'error:0.0:{err}'
                else:
                    srv_ok = False
                    msg = 'timeout:0.0:'

                # Parse message (<label>:<confidence>:<colour>)
                parts = msg.split(':')
                got = parts[0].strip().lower() if len(parts) > 0 and parts[0].strip() else 'unknown'
                try:
                    conf = float(parts[1]) if len(parts) > 1 else 0.0
                except ValueError:
                    conf = 0.0
                colour = parts[2].strip() if len(parts) > 2 else ''

                # Optional topic result JSON
                per_label_json = ''
                if node.latest_topic_json:
                    try:
                        parsed = json.loads(node.latest_topic_json)
                        per_label_json = json.dumps(parsed.get('per_label', parsed))
                    except Exception:
                        per_label_json = node.latest_topic_json

                # Determine line status
                if expected:
                    is_hit = (got == expected)
                    tag = 'OK' if is_hit else 'MISS'
                else:
                    tag = '-'

                record = {
                    'file': fname,
                    'expected': expected,
                    'got': got,
                    'confidence': conf,
                    'colour': colour,
                    'latency_s': lat_s,
                    'success': srv_ok,
                    'per_label_json': per_label_json,
                }
                records.append(record)

                print(f'[{cur_idx:>3}/{total_runs:<3}]  {fname:<32}  '
                      f'exp={expected or "-":<8}  got={got:<8}  '
                      f'conf={conf:4.2f}  lat={lat_s:5.3f}s  {tag}')

            if user_aborted:
                print('\nTest aborted by user.')
                break

    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

    # Print summary report
    print('\n' + '=' * 60)
    print('SUMMARY REPORT')
    print('=' * 60)

    scored = [r for r in records if r['expected']]
    if scored:
        ok_count = sum(1 for r in scored if r['got'] == r['expected'])
        acc = (ok_count / len(scored)) * 100.0
        print(f'Accuracy over scored images: {ok_count}/{len(scored)} ({acc:.1f}%)')
    else:
        print('Accuracy: N/A (no scored images)')

    if records:
        latencies = [r['latency_s'] for r in records]
        mean_lat = sum(latencies) / len(latencies)
        max_lat = max(latencies)
        print(f'Latency: mean = {mean_lat:.3f} s, max = {max_lat:.3f} s (over {len(records)} calls)')

    # Confusion matrix
    if scored:
        conf_tab = Counter((r['expected'], r['got']) for r in scored)
        rows = sorted({r['expected'] for r in scored})
        cols = sorted({r['got'] for r in scored} | {'unknown'})
        row_w = max(max(len(r) for r in rows), 10) + 2
        col_w = max(max(len(c) for c in cols), 8) + 2

        print('\nConfusion table (rows: expected, cols: got):')
        hdr = f"{'expected \\ got':<{row_w}}" + ''.join(f'{c:>{col_w}}' for c in cols)
        print(hdr)
        print('-' * len(hdr))
        for r in rows:
            print(f'{r:<{row_w}}' + ''.join(f'{conf_tab[(r, c)]:>{col_w}d}' for c in cols))

    # Misses
    misses = [r for r in scored if r['got'] != r['expected']]
    if misses:
        print(f'\nMisses ({len(misses)}):')
        for m in misses:
            print(f"  - {m['file']}: expected '{m['expected']}', got '{m['got']}' "
                  f"(conf={m['confidence']:.2f}, lat={m['latency_s']:.3f}s)")
    elif scored:
        print('\nMisses: None (100% accuracy)')

    # Export CSV
    csv_cols = ['file', 'expected', 'got', 'confidence', 'colour', 'latency_s', 'success', 'per_label_json']
    try:
        with open(args.out, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=csv_cols)
            writer.writeheader()
            for r in records:
                writer.writerow({
                    'file': r['file'],
                    'expected': r['expected'],
                    'got': r['got'],
                    'confidence': f"{r['confidence']:.4f}",
                    'colour': r['colour'],
                    'latency_s': f"{r['latency_s']:.4f}",
                    'success': r['success'],
                    'per_label_json': r['per_label_json'],
                })
        print(f'\nResults written to: {args.out}')
    except Exception as e:
        print(f'\nERROR: Failed to write CSV output to {args.out}: {e}', file=sys.stderr)
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
