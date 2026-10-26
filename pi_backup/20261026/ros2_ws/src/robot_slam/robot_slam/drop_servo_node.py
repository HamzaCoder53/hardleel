"""ROS node `drop_servo`: service /dump (std_srvs/Trigger), topic /drop/state (String).

Run: ros2 run robot_slam drop_servo_node --ros-args --params-file config/drop_servo.yaml
Default backend pico: the servos hang off the Pico (GP12 + GP13, one angle), commanded through motor_bridge
(/motor/servo_cmd, confirmed by /motor/servo_state "done,<deg>"). pi_pwm/lgpio backends drive Pi GPIO servos instead.
"""
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .dropper import Dropper
from .servo_backends import NullBackend, make_backend
from .servo_core import DEFAULTS


class DropServoNode(Node):
    def __init__(self):
        super().__init__('drop_servo')
        cfg = {k: self.declare_parameter(k, v).value for k, v in DEFAULTS.items()}
        backend = NullBackend()
        if cfg['has_drop_bin']:
            try:
                backend = make_backend(cfg['backend'], [cfg['servo_a_gpio'], cfg['servo_b_gpio']], node=self,
                                       min_us=cfg['min_us'], max_us=cfg['max_us'], range_deg=cfg['range_deg'],
                                       dps=cfg['pico_dps'], timeout_s=cfg['pico_timeout_s'])
            except Exception as e:  # never crash on hardware
                self.get_logger().error('backend init failed, using null: %s' % e)
        self.get_logger().info('has_drop_bin=%s backend=%s' % (cfg['has_drop_bin'], type(backend).__name__))
        self.backend = backend
        self.pub = self.create_publisher(String, '/drop/state', 10)
        self.dropper = Dropper(backend, cfg, on_state=self._state)
        self._state('idle')
        cbg = ReentrantCallbackGroup()   # /dump and start-up may block waiting for the Pico's SV,done
        self.create_service(Trigger, '/dump', self._dump, callback_group=cbg)
        if cfg['close_on_start']:        # after spinning starts, so the pico backend can hear SV,done
            self._start_timer = self.create_timer(2.0, self._close_on_start, callback_group=cbg)

    def _close_on_start(self):
        self._start_timer.cancel()
        ok, msg = self.dropper.close_flaps()
        if not ok:
            self.get_logger().warning('close on start: %s' % msg)

    def _state(self, s):
        self.pub.publish(String(data=s))

    def _dump(self, req, res):
        res.success, res.message = self.dropper.dump()
        self.get_logger().info('/dump: %s' % res.message)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = DropServoNode()
    ex = MultiThreadedExecutor(num_threads=3)   # a blocked /dump must not starve /motor/servo_state
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.backend.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
