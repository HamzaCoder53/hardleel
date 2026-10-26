"""Camera v2 (final stack item 3, switch camera_v2 in pi_bringup, OFF by default). ONE GStreamer pipeline:

    libcamerasrc 1920x1080@30 NV12 -> v4l2convert (ISP copy) -> I420 -> tee
      +-> queue -> v4l2h264enc (Pi 4 HARDWARE encoder) -> h264parse -> matroskamux -> filesink
             video/<start time>/video.mkv: Matroska stays playable when cut off by a crash / power loss
             (fragmented mp4mux throttled the encoder to ~12 fps here; splitmuxsink stalled - Pi, 26 Oct)
      +-> queue(1, leaky) -> appsink 'frames' (every frame, released at once): Python keeps every Nth frame for the
             lores ROS stream (/camera/image_raw/compressed, 320x240 @ 5 fps, as before) and, on request, a burst of
             full-resolution frames for a still. The video never pauses.
Why this shape (measured on the Pi, 26 Oct): the encoder cannot take libcamera's buffers directly (0 bytes in any
io-mode), and any branch that HOLDS buffers (videorate, videoscale, a queued valve, splitmuxsink) starves the
converter's small pool and the video stalls; a branch that drains at once does not.
Frame times: a probe AFTER the encoder queue (its own thread) stamps every encoded frame: /camera/frame_info
(sensor_msgs/TimeReference: header.stamp = ROS time, source = frame number) + video/.../frames.csv, so frames line up
with lidar/IMU in the bag.
Stills: /camera/still_request (String, JSON {"label": "screen"} or just the label) -> settle_s (0.5 s), burst (3)
full frames, sharpest kept (variance of the Laplacian), stills/<time>_<label>.jpg (q 95), /camera/still_info (JSON:
file, label, time, sharpness, pose from TF map->base_link or null).
Focus: libcamerasrc 0.2 only has auto-focus-mode: 'auto' = one AF cycle at start, 'manual' = fixed default lens.
A set 0.5-1.5 m distance or an AF cycle per stop needs a newer libcamera / picamera2 (not on Ubuntu 24.04).
Output: ~/runs/<run>/{video,stills} once /run/id is known (run_recorder), else ~/camera/.
CPU (Pi 4): 1080p30 ~80 % of one core for the copy + encode; 720p30 (width:=1280 height:=720) ~40 %.
"""
import csv
import json
import math
import os
import threading
import time

import cv2
import gi
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, TimeReference
from std_msgs.msg import String

gi.require_version('Gst', '1.0')
from gi.repository import Gst  # noqa: E402

ROTATE = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def sharpness(bgr):
    """Variance of the Laplacian on a half-size grey image (bigger = sharper)."""
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    g = cv2.resize(g, (g.shape[1] // 2, g.shape[0] // 2), interpolation=cv2.INTER_AREA)
    return float(cv2.Laplacian(g, cv2.CV_64F).var())


def pick_sharpest(frames):
    scores = [sharpness(f) for f in frames]
    return int(np.argmax(scores)), scores


def i420_to_bgr(data, w, h, out_size=None):
    """I420 bytes -> BGR. out_size (w, h): cheap stride-decimation of the planes first (lores stream)."""
    a = np.frombuffer(data, dtype=np.uint8)
    if out_size is None:
        return cv2.cvtColor(a[:w * h * 3 // 2].reshape(h * 3 // 2, w), cv2.COLOR_YUV2BGR_I420)
    sx, sy = max(1, w // out_size[0]), max(1, h // out_size[1])
    sx = sy = max(1, min(sx, sy)) // 2 * 2 or 1          # even step keeps the chroma planes aligned
    y = a[:w * h].reshape(h, w)[::sy, ::sx]
    u = a[w * h:w * h * 5 // 4].reshape(h // 2, w // 2)[::sy, ::sx]
    v = a[w * h * 5 // 4:w * h * 3 // 2].reshape(h // 2, w // 2)[::sy, ::sx]
    yh, yw = y.shape
    u, v = u[:yh // 2, :yw // 2], v[:yh // 2, :yw // 2]
    yuv = np.concatenate([y[:u.shape[0] * 2, :u.shape[1] * 2].ravel(), u.ravel(), v.ravel()])
    bgr = cv2.cvtColor(yuv.reshape(u.shape[0] * 3, u.shape[1] * 2), cv2.COLOR_YUV2BGR_I420)
    return cv2.resize(bgr, out_size, interpolation=cv2.INTER_AREA) if bgr.shape[1::-1] != tuple(out_size) else bgr


def pipeline_string(p, video_path):
    w, h, fps = p['width'], p['height'], p['fps']
    af = {'auto': 'AfModeAuto', 'manual': 'AfModeManual', 'continuous': 'AfModeContinuous'}[p['focus_mode']]
    s = (f'libcamerasrc auto-focus-mode={af} ! video/x-raw,width={w},height={h},framerate={fps}/1,format=NV12 ! '
         f'{p["convert"]} ! video/x-raw,format=I420 ! tee name=t ')
    if p['record_video']:
        s += (f't. ! queue name=encq leaky=downstream max-size-buffers=4 ! v4l2h264enc output-io-mode=mmap '
              f'extra-controls=controls,video_bitrate={p["bitrate"]},h264_i_frame_period={fps} ! '
              f'video/x-h264,level=(string)4 ! h264parse ! matroskamux ! '
              f'filesink location={video_path} ')
    # no per-frame Python callback (cost ~30 % of a core): the sink keeps only the NEWEST frame (drop, 1 buffer) and
    # Python pulls it at the lores rate / during a still burst
    s += ('t. ! queue leaky=downstream max-size-buffers=1 ! '
          'appsink name=frames emit-signals=false max-buffers=1 drop=true sync=false async=false')
    return s


class CameraV2(Node):
    def __init__(self):
        super().__init__('camera_v2')
        d = lambda n, v: self.declare_parameter(n, v).value
        self.p = {'width': int(d('width', 1920)), 'height': int(d('height', 1080)), 'fps': int(d('fps', 30)),
                  'bitrate': int(d('bitrate', 8000000)), 'lores_width': int(d('lores_width', 320)),
                  'lores_height': int(d('lores_height', 240)), 'lores_fps': float(d('lores_fps', 5.0)),
                  'jpeg_quality': int(d('jpeg_quality', 80)), 'rotate_deg': int(d('rotate_deg', 0)) % 360,
                  'burst': int(d('burst', 3)), 'settle_s': float(d('settle_s', 0.5)),
                  'focus_mode': str(d('focus_mode', 'auto')), 'record_video': bool(d('record_video', True)),
                  'convert': str(d('convert', 'v4l2convert'))}
        self.rotate = ROTATE.get(self.p['rotate_deg'])
        self.every = max(1, int(round(self.p['fps'] / max(self.p['lores_fps'], 0.1))))
        self.root = os.path.expanduser('~/camera')
        self.pub_lores = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', QoSProfile(depth=1))
        self.pub_frame = self.create_publisher(TimeReference, '/camera/frame_info', 50)
        self.pub_still = self.create_publisher(String, '/camera/still_info', 10)
        self.create_subscription(String, '/run/id', self.on_run, LATCHED)
        self.create_subscription(String, '/camera/still_request', self.on_still_request, 10)
        self.tf = None
        try:
            from tf2_ros import Buffer, TransformListener
            self.tf = Buffer()
            TransformListener(self.tf, self)
        except ImportError:
            pass
        self.lock = threading.Lock()
        self.burst_left, self.burst = 0, []
        self.sample_n = self.frame_n = 0
        self.closing = self.eos_done = False
        Gst.init(None)
        self.start_pipeline()

    def out_dir(self, sub):
        d = os.path.join(self.root, sub)
        os.makedirs(d, exist_ok=True)
        return d

    # ------------------------------------------------------------------ pipeline
    def start_pipeline(self):
        vdir = self.out_dir(time.strftime('video/%Y%m%d_%H%M%S'))
        self.frames_csv = open(os.path.join(vdir, 'frames.csv'), 'w', newline='')
        self.frames_w = csv.writer(self.frames_csv)
        self.frames_w.writerow(['frame', 'ros_time', 'gst_pts_ns'])
        desc = pipeline_string(self.p, os.path.join(vdir, 'video.mkv'))
        self.get_logger().info(f'pipeline: {desc}')
        self.pipe = Gst.parse_launch(desc)
        self.sink = self.pipe.get_by_name('frames')
        threading.Thread(target=self.pull_loop, daemon=True).start()
        encq = self.pipe.get_by_name('encq')
        if encq is not None:                   # AFTER the queue: Python never blocks the camera thread
            encq.get_static_pad('src').add_probe(Gst.PadProbeType.BUFFER, self.on_video_frame)
        bus = self.pipe.get_bus()
        bus.add_signal_watch()
        bus.connect('message', self.on_bus)
        threading.Thread(target=self.gst_loop, daemon=True).start()
        self.pipe.set_state(Gst.State.PLAYING)
        self.get_logger().info(f'camera v2: {self.p["width"]}x{self.p["height"]}@{self.p["fps"]}, video '
                               f'{"on" if self.p["record_video"] else "off"} -> {vdir}; lores every {self.every} frames')

    def gst_loop(self):
        from gi.repository import GLib
        self.glib_loop = GLib.MainLoop()
        self.glib_loop.run()

    def on_bus(self, _bus, msg):
        if msg.type == Gst.MessageType.EOS:
            self.eos_done = True
        elif msg.type == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            self.get_logger().error(f'GStreamer: {err} ({dbg})')
        elif msg.type == Gst.MessageType.WARNING:
            w, _ = msg.parse_warning()
            self.get_logger().warn(f'GStreamer: {w}', throttle_duration_sec=5.0)

    def on_video_frame(self, _pad, info):
        if not self.closing:
            try:
                buf = info.get_buffer()
                now = self.get_clock().now()
                self.frame_n += 1
                pts = buf.pts if buf.pts != Gst.CLOCK_TIME_NONE else 0
                m = TimeReference()
                m.header.stamp = now.to_msg()
                m.header.frame_id = 'camera'
                m.time_ref.sec, m.time_ref.nanosec = int(pts // 1_000_000_000), int(pts % 1_000_000_000)
                m.source = str(self.frame_n)
                self.pub_frame.publish(m)
                self.frames_w.writerow([self.frame_n, f'{now.nanoseconds * 1e-9:.6f}', pts])
                if self.frame_n % 300 == 0:
                    self.frames_csv.flush()
            except Exception:                      # ROS context gone at Ctrl-C: never break the video branch
                pass
        return Gst.PadProbeReturn.OK

    def pull_loop(self):
        """Lores at lores_fps from the newest frame; during a still burst, consecutive full frames (~30 fps)."""
        period = 1.0 / max(self.p['lores_fps'], 0.1)
        next_lores = time.monotonic()
        last_pts = None
        while not self.closing:
            with self.lock:
                want_full = self.burst_left > 0
            now = time.monotonic()
            if not want_full and now < next_lores:
                time.sleep(min(next_lores - now, 0.02))
                continue
            sample = self.sink.emit('try-pull-sample', 50 * Gst.MSECOND)
            if sample is None:
                continue
            buf = sample.get_buffer()
            if want_full and buf.pts == last_pts:
                continue
            last_pts = buf.pts
            caps = sample.get_caps().get_structure(0)
            w, h = caps.get_value('width'), caps.get_value('height')
            ok, mi = buf.map(Gst.MapFlags.READ)
            if not ok:
                continue
            try:
                if want_full:
                    full = i420_to_bgr(mi.data, w, h)
                else:
                    small = i420_to_bgr(mi.data, w, h, (self.p['lores_width'], self.p['lores_height']))
            finally:
                buf.unmap(mi)
            if want_full:
                with self.lock:
                    if self.burst_left > 0:
                        self.burst.append(full)
                        self.burst_left -= 1
            else:
                next_lores += period
                if next_lores < time.monotonic():
                    next_lores = time.monotonic() + period
                self.publish_lores(small)

    def publish_lores(self, frame):
        if self.rotate is not None:
            frame = cv2.rotate(frame, self.rotate)
        ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.p['jpeg_quality']])
        if ok:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            msg.format = 'jpeg'
            msg.data = jpg.tobytes()
            try:
                self.pub_lores.publish(msg)
            except Exception:
                pass

    # ------------------------------------------------------------------ stills
    def on_run(self, m):
        if m.data:
            self.root = os.path.expanduser(os.path.join('~/runs', m.data))
            self.get_logger().info(f'run {m.data}: stills -> {self.root}/stills (video keeps its file)')

    def on_still_request(self, m):
        txt = m.data.strip()
        try:
            label = json.loads(txt).get('label', 'still') if txt.startswith('{') else (txt or 'still')
        except ValueError:
            label = 'still'
        threading.Thread(target=self.take_still, args=(label,), daemon=True).start()

    def take_still(self, label):
        time.sleep(self.p['settle_s'])                              # the robot has just stopped: let it settle
        with self.lock:
            self.burst, self.burst_left = [], self.p['burst']
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            with self.lock:
                if self.burst_left <= 0:
                    break
            time.sleep(0.02)
        with self.lock:
            frames, self.burst_left = list(self.burst), 0
        if not frames:
            self.get_logger().error(f'still {label}: no frames')
            return
        k, scores = pick_sharpest(frames)
        img = frames[k] if self.rotate is None else cv2.rotate(frames[k], self.rotate)
        path = os.path.join(self.out_dir('stills'), time.strftime('%Y%m%d_%H%M%S') + f'_{label}.jpg')
        cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, 95])
        info = {'file': path, 'label': label, 'time': self.get_clock().now().nanoseconds * 1e-9,
                'size': [img.shape[1], img.shape[0]], 'sharpness': round(scores[k], 1),
                'burst_scores': [round(s, 1) for s in scores], 'pose': self.pose()}
        self.pub_still.publish(String(data=json.dumps(info)))
        self.get_logger().info(f'still {label}: {path} (sharpness {scores[k]:.0f} of {len(frames)})')

    def pose(self):
        if self.tf is None:
            return None
        try:
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return [round(t.transform.translation.x, 3), round(t.transform.translation.y, 3), round(yaw, 3)]

    def shutdown(self):
        self.closing = True
        self.frames_csv.flush()
        self.pipe.send_event(Gst.Event.new_eos())                   # MKV cues/duration written properly
        end = time.monotonic() + 3.0
        while not self.eos_done and time.monotonic() < end:
            time.sleep(0.05)
        done = threading.Event()                                    # set_state(NULL) can block in libcamera:
        threading.Thread(target=lambda: (self.pipe.set_state(Gst.State.NULL), done.set()), daemon=True).start()
        done.wait(3.0)                                              # MKV is playable even if this is cut short
        self.frames_csv.close()
        if hasattr(self, 'glib_loop'):
            self.glib_loop.quit()


def main():
    import signal

    def on_term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)                 # same clean stop as Ctrl-C (scripts, systemd, kill)
    rclpy.init()
    n = CameraV2()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.closing = True
        try:
            n.frames_csv.flush()
        except ValueError:
            pass
        t = threading.Thread(target=n.shutdown, daemon=True)        # EOS + NULL can hang inside libcamera:
        t.start()
        t.join(6.0)                                                 # bounded; MKV + CSV are safe by now
        if t.is_alive():
            n.get_logger().warn('camera pipeline did not stop in 6 s: forcing exit (video file is still playable)')
            os._exit(0)
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
