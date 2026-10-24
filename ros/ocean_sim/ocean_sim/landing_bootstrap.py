"""Connect the velocity/enable command path before releasing paused physics."""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from ros_gz_interfaces.srv import ControlWorld
from std_msgs.msg import Bool, String


class LandingBootstrap(Node):
    def __init__(self):
        super().__init__('landing_bootstrap')
        self.command = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.enable = self.create_publisher(Bool, '/drone/enable', 10)
        self.create_subscription(String, '/landing/status', self.status, 10)
        self.client = self.create_client(ControlWorld, '/world/ocean/control')
        self.ready_since = None
        self.last_controller_status = None
        self.future = None
        self.started = time.monotonic()
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.1, self.tick, clock=self.steady)

    def status(self, _message):
        self.last_controller_status = time.monotonic()

    def tick(self):
        if time.monotonic() - self.started > 60:
            raise RuntimeError('Landing startup timed out: check controller and world-control bridge.')
        if self.future is not None:
            if not self.future.done():
                return
            result = self.future.result()
            if result is not None and result.success:
                self.get_logger().info('Controller connected; unpaused the landing scene.')
                self.timer.cancel()
                return
            self.future = None
            self.ready_since = None
        self.command.publish(Twist())
        self.enable.publish(Bool(data=True))
        ready = (self.last_controller_status is not None and
                 time.monotonic() - self.last_controller_status < 2.0 and
                 self.client.service_is_ready() and
                 self.command.get_subscription_count() > 0 and
                 self.enable.get_subscription_count() > 0)
        if ready:
            if self.ready_since is None:
                self.ready_since = time.monotonic()
            if time.monotonic() - self.ready_since >= 1.0:
                request = ControlWorld.Request()
                request.world_control.pause = False
                self.future = self.client.call_async(request)
        else:
            self.ready_since = None


def main(args=None):
    rclpy.init(args=args)
    node = LandingBootstrap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
