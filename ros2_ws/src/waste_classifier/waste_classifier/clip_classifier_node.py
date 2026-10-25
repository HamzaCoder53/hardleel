#!/usr/bin/env python3
"""ROS node `clip_classifier`: zero-shot CLIP waste classification on the laptop (logic in clip_core.py).

Topics: in  /camera/image_raw/compressed (CompressedImage, best effort; only the latest frame is kept)
        out /classifier/result (String JSON {label, confidence, colour, per_label, n_frames, latency_s, stamp})
Service: /classify (std_srvs/Trigger) -> message 'label:conf:colour' (e.g. 'plastic:0.82:blue'); collects n_frames
         NEW frames after the request, votes; none arrive -> success=False, 'unknown:0.00:'
Params: see config/clip_classifier.yaml (model, pretrained, cache_dir, offline, num_threads, n_frames,
        frame_timeout_s, min_conf, rotate_deg, crop_frac, image_topic, prompts.*, label_map.*, colours.*)
Run with a MultiThreadedExecutor so frames keep arriving while /classify waits.
"""
import io
import json
import time

import rclpy
from PIL import Image
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger

from waste_classifier import clip_core as core


class ClipClassifierNode(Node):
    def __init__(self):
        super().__init__('clip_classifier', automatically_declare_parameters_from_overrides=True)
        self.cbg = ReentrantCallbackGroup()

        def p(name, default):
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
            return self.get_parameter(name).value

        self.n_frames = int(p('n_frames', 5))
        self.frame_timeout_s = float(p('frame_timeout_s', 4.0))
        self.min_conf = float(p('min_conf', 0.45))  # PLACEHOLDER
        topic = p('image_topic', '/camera/image_raw/compressed')
        cfg = {k: p(k, d) for k, d in [
            ('model', 'ViT-B-32'), ('pretrained', 'laion2b_s34b_b79k'), ('cache_dir', '~/.cache/clip_weights'),
            ('offline', True), ('num_threads', 4), ('rotate_deg', 0), ('crop_frac', 1.0)]}
        for prefix in ('prompts', 'label_map', 'colours'):
            cfg[prefix] = {k: v.value for k, v in self.get_parameters_by_prefix(prefix).items()}
        if not cfg['prompts']:
            raise RuntimeError('no prompts.* params; pass --params-file config/clip_classifier.yaml')
        self.label_map, self.colours = cfg['label_map'], cfg['colours']

        t0 = time.time()
        self.clf = core.ClipClassifier(cfg, log=self.get_logger().info)
        self.clf.classify_image(Image.new('RGB', (320, 240)))  # warm-up
        self.get_logger().info('model ready in %.1f s (incl. warm-up)' % (time.time() - t0))

        self.latest = None  # (arrival monotonic s, CompressedImage)
        self.create_subscription(CompressedImage, topic, self._on_image, qos_profile_sensor_data,
                                 callback_group=self.cbg)
        self.pub = self.create_publisher(String, '/classifier/result', 10)
        self.create_service(Trigger, '/classify', self._on_classify, callback_group=self.cbg)
        self.get_logger().info('ready: /classify (%d frames, min_conf %.2f), image topic %s'
                               % (self.n_frames, self.min_conf, topic))

    def _on_image(self, msg):
        self.latest = (time.monotonic(), msg)

    def _decode(self, msg):
        return Image.open(io.BytesIO(bytes(msg.data))).convert('RGB')

    def _on_classify(self, req, res):
        t0 = time.monotonic()
        scores, last_t = [], t0
        while len(scores) < self.n_frames and time.monotonic() - t0 < self.frame_timeout_s:
            cur = self.latest
            if cur is None or cur[0] <= last_t:  # only frames newer than the request / the previous one
                time.sleep(0.02)
                continue
            last_t = cur[0]
            try:
                scores.append(self.clf.classify_image(self._decode(cur[1])))
            except Exception as e:
                self.get_logger().warn('frame skipped: %s' % e)
        if not scores:
            self.get_logger().warn('no frames within %.1f s' % self.frame_timeout_s)
            res.success, res.message = False, 'unknown:0.00:'
            return res
        label, conf, per_label = core.vote(scores, self.label_map, self.min_conf)
        colour = self.colours.get(label, '')
        latency = time.monotonic() - t0
        self.get_logger().info('result %s conf %.2f colour "%s" (%d frames, %.2f s) %s' % (
            label, conf, colour, len(scores), latency,
            {k: round(v, 2) for k, v in per_label.items()}))
        out = {'label': label, 'confidence': round(conf, 4), 'colour': colour,
               'per_label': {k: round(v, 4) for k, v in per_label.items()},
               'n_frames': len(scores), 'latency_s': round(latency, 3), 'stamp': time.time()}
        self.pub.publish(String(data=json.dumps(out)))
        res.success, res.message = True, '%s:%.2f:%s' % (label, conf, colour)
        return res


def main():
    rclpy.init()
    node = ClipClassifierNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
