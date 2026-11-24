#!/usr/bin/env python3
"""Live camera stream from the Pi to the laptop's web browser (Camera Module 3 / IMX708, Ubuntu libcamera).

Run on the Pi:   python3 ~/pi_tools/cam_stream.py            (port 8080, 640x480)
Open on laptop:  http://hardleel.local:8080                  (any browser; Ctrl+C on the Pi to stop)

Only ONE program can use the camera: stop the AI service (waste_classifier_service) before streaming,
and stop this stream before running the AI / the behaviour tree.

How: gst-launch-1.0 libcamerasrc -> JPEG frames on stdout -> HTTP multipart (MJPEG) server. Uses Ubuntu's
libcamera (the ROS build crashes, 24 Nov). Fallback when GStreamer's libcamerasrc is missing: one `cam`
capture per frame (slow, ~1 fps).
"""
import http.server
import os
import shutil
import socketserver
import subprocess
import sys
import tempfile
import threading
import time

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
W, H = 640, 480
latest = {'jpg': None, 'n': 0}
lock = threading.Condition()
ENV = {k: v for k, v in os.environ.items() if k != 'LD_LIBRARY_PATH'}   # Ubuntu libcamera, not the ROS build


def publish(jpg):
    with lock:
        latest['jpg'], latest['n'] = jpg, latest['n'] + 1
        lock.notify_all()


def gst_reader():
    cmd = ['gst-launch-1.0', '-q', 'libcamerasrc', '!', f'video/x-raw,width={W},height={H},framerate=15/1',
           '!', 'videoconvert', '!', 'jpegenc', 'quality=70', '!', 'fdsink', 'fd=1']
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, env=ENV)
    buf = b''
    while True:
        chunk = p.stdout.read(65536)
        if not chunk:
            return p.wait()
        buf += chunk
        while True:                                   # cut complete JPEGs (FFD8 ... FFD9)
            s = buf.find(b'\xff\xd8')
            e = buf.find(b'\xff\xd9', s + 2)
            if s < 0 or e < 0:
                break
            publish(buf[s:e + 2])
            buf = buf[e + 2:]


def cam_reader():                                     # fallback: one `cam` still per frame
    import numpy as np
    import cv2
    while True:
        with tempfile.TemporaryDirectory() as d:
            subprocess.run(['cam', '-c1', '--capture=3', '-s', f'width={W},height={H},pixelformat=BGR888',
                            f'--file={d}/f#.bin'], env=ENV, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            fs = sorted(f for f in os.listdir(d) if f.endswith('.bin'))
            if fs:
                rgb = np.fromfile(os.path.join(d, fs[-1]), np.uint8).reshape(H, W, 3)
                ok, jpg = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                if ok:
                    publish(jpg.tobytes())
        time.sleep(0.2)


def reader():
    if shutil.which('gst-launch-1.0'):
        t0 = time.time()
        rc = gst_reader()
        if latest['n'] > 0 or time.time() - t0 > 10:
            print(f'GStreamer stream ended (exit {rc})', flush=True)
            return
        print('GStreamer libcamerasrc did not work - falling back to `cam` (slow)', flush=True)
    cam_reader()


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == '/':
            page = (f'<html><body style="background:#222;color:#ddd;font-family:sans-serif">'
                    f'<h3>hardleel camera ({W}x{H})</h3><img src="/stream" style="max-width:100%"></body></html>')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(page.encode())
            return
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        seen = -1
        try:
            while True:
                with lock:
                    lock.wait_for(lambda: latest['n'] != seen, timeout=5)
                    jpg, seen = latest['jpg'], latest['n']
                if jpg is None:
                    continue
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                 + str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            pass


class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


if __name__ == '__main__':
    threading.Thread(target=reader, daemon=True).start()
    print(f'camera stream: open http://hardleel.local:{PORT} on the laptop (Ctrl+C to stop)', flush=True)
    try:
        Server(('0.0.0.0', PORT), Handler).serve_forever()
    except KeyboardInterrupt:
        pass
