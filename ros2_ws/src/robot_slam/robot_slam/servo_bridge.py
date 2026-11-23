"""Servo on the Nano 33 BLE (firmware/imu_nano33: servo on D9), driven through imu_serial_node (24 Nov).

The IMU node owns the Nano's USB port, so servo commands go out through the same serial port:
  Pi -> Nano   S,<angle> (0..180) | S,off | S,?        Nano -> Pi   S,<angle> | S,off  (confirmation)

ROS interface (created by imu_serial_node):
  /servo/cmd     std_msgs/String   "grab" | "release" | "off" | "<angle>"   (the mission behaviour tree uses this)
  /servo/grab    std_srvs/Trigger  move to servo_grab_angle, success once the Nano confirms
  /servo/release std_srvs/Trigger  move to servo_release_angle
  /servo/off     std_srvs/Trigger  stop driving the servo (limp)
  /servo/state   std_msgs/String   the Nano's last confirmation, e.g. "120" or "off"
Parameters (live): servo_grab_angle (120, placeholder), servo_release_angle (0, placeholder).
Test:  ros2 service call /servo/grab std_srvs/srv/Trigger
       ros2 topic pub --once /servo/cmd std_msgs/msg/String "{data: '90'}"
"""
import threading
import time

from std_msgs.msg import String
from std_srvs.srv import Trigger


class ServoBridge:
    def __init__(self, node, ser):
        self.node, self.ser = node, ser
        self.lock = threading.Lock()
        self.reply = None
        self.reply_event = threading.Event()
        node.declare_parameter('servo_grab_angle', 120)     # placeholder until the gripper is built
        node.declare_parameter('servo_release_angle', 0)    # placeholder
        self.state_pub = node.create_publisher(String, '/servo/state', 10)
        node.create_subscription(String, '/servo/cmd', self.on_cmd, 10)
        node.create_service(Trigger, '/servo/grab', lambda req, res: self.on_srv('grab', res))
        node.create_service(Trigger, '/servo/release', lambda req, res: self.on_srv('release', res))
        node.create_service(Trigger, '/servo/off', lambda req, res: self.on_srv('off', res))
        node.get_logger().info(f'servo: /servo/cmd + /servo/grab ({self.angle("grab")} deg) '
                               f'/servo/release ({self.angle("release")} deg) /servo/off')

    def angle(self, name):
        return int(self.node.get_parameter(f'servo_{name}_angle').value)

    def to_line(self, text):
        """'grab' | 'release' | 'off' | '<angle>' -> 'S,...' (None if not understood)."""
        t = text.strip().lower()
        if t in ('grab', 'release'):
            return f'S,{max(0, min(180, self.angle(t)))}'
        if t == 'off':
            return 'S,off'
        try:
            return f'S,{max(0, min(180, int(float(t))))}'
        except ValueError:
            return None

    def send(self, text, wait_s=0.0):
        line = self.to_line(text)
        if line is None:
            self.node.get_logger().warn(f'servo: unknown command "{text}" (grab | release | off | <angle>)')
            return None
        self.reply_event.clear()
        with self.lock:
            try:
                self.ser.write((line + '\n').encode())
            except Exception as e:  # noqa: BLE001 - serial gone
                self.node.get_logger().error(f'servo: write failed: {e}')
                return None
        if wait_s > 0 and self.reply_event.wait(wait_s):
            return self.reply
        return None

    def on_cmd(self, msg):
        self.node.get_logger().info(f'servo: {msg.data} -> {self.to_line(msg.data)}')
        self.send(msg.data)

    def on_srv(self, name, res):
        line = self.to_line(name)
        reply = self.send(name, wait_s=1.0)
        res.success = reply is not None and line is not None and reply == line[2:]
        res.message = f'{line} -> Nano: {reply}' if reply else f'{line} sent, no confirmation from the Nano (old firmware?)'
        return res

    def on_reply(self, line):
        """Called by imu_serial_node for every 'S,...' line from the Nano."""
        self.reply = line.split(',', 1)[1].strip() if ',' in line else ''
        self.reply_event.set()
        self.state_pub.publish(String(data=self.reply))
