#!/usr/bin/env python3
"""Event logger: stamps /mission/event, writes run JSON+CSV after every event, forwards to a judges sender.

Topics: in /mission/event (String JSON {"event","detail"} or a bare event name), /mission/state (String).
Params: robot_id (HARDLEEL), log_dir (~/logs/runs), sender (stub|http), sender_url (''), print_period_s (2.0),
        sender_timeout_s (1.0).
Files: <log_dir>/run_<YYYYmmdd_HHMMSS>_<robot_id>.json / .csv (new per node start).
"""
import json
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from robot_slam.event_log_core import RunLog, format_summary
from robot_slam.judges_sender import make_sender


def parse_event(text):
    """-> (event, detail dict); tolerates a bare event name."""
    text = (text or '').strip()
    try:
        d = json.loads(text)
        if isinstance(d, dict) and d.get('event'):
            det = d.get('detail')
            return str(d['event']), det if isinstance(det, dict) else ({} if det is None else {'value': det})
        if isinstance(d, str) and d:
            return d, {}
    except ValueError:
        pass
    return text, {}


class EventLogger(Node):
    def __init__(self):
        super().__init__('event_logger')
        p = self.declare_parameter
        p('robot_id', 'HARDLEEL')               # PLACEHOLDER until organisers assign one
        p('log_dir', '~/logs/runs')
        p('sender', 'stub')
        p('sender_url', '')
        p('sender_timeout_s', 1.0)
        p('print_period_s', 2.0)

        def g(n):
            return self.get_parameter(n).value
        self.robot_id = str(g('robot_id'))
        d = os.path.expanduser(str(g('log_dir')))
        stamp = time.strftime('%Y%m%d_%H%M%S')
        base = os.path.join(d, 'run_%s_%s' % (stamp, self.robot_id))
        self.json_path, self.csv_path = base + '.json', base + '.csv'
        self.log = RunLog(self.robot_id, d)
        self.log.run_id = 'run_%s_%s' % (stamp, self.robot_id)
        self.sender = make_sender(str(g('sender')), url=str(g('sender_url')),
                                  timeout=float(g('sender_timeout_s')), logger=self.get_logger().info)
        self.state = ''
        self.create_subscription(String, '/mission/event', self._on_event, 50)
        self.create_subscription(String, '/mission/state', self._on_state, 10)
        per = float(g('print_period_s'))
        if per > 0:
            self.create_timer(per, self._print)
        self._write()
        self.get_logger().info('event_logger: writing %s' % self.json_path)

    def _write(self):
        try:
            self.log.write(self.json_path, self.csv_path)
        except Exception as e:      # noqa: BLE001
            self.get_logger().error('log write failed: %s' % e)

    def _on_state(self, msg):
        self.state = msg.data

    def _on_event(self, msg):
        event, detail = parse_event(msg.data)
        if not event:
            return
        occ = self.log.record(event, detail, time.time(), self.get_clock().now().nanoseconds * 1e-9)
        self._write()
        try:
            self.sender.send_event(self.robot_id, event, {'detail': detail, 'wall_iso': occ['wall_iso'],
                                                          't_rel': occ['t_rel']})
        except Exception as e:      # noqa: BLE001
            self.get_logger().warn('sender failed: %s' % e)
        self.get_logger().info('event %s%s t_rel=%s %s' % (event, '' if occ['is_first'] else ' (repeat)',
                                                          occ['t_rel'], json.dumps(detail, default=str)))

    def _print(self):
        self.get_logger().info('\n' + format_summary(self.log, self.state, time.time()))

    def finish(self):
        self._write()
        try:
            self.sender.send_summary(self.robot_id, self.log.to_dict()['required_summary'])
        except Exception:       # noqa: BLE001
            pass
        print(format_summary(self.log, self.state, time.time()), flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = EventLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
