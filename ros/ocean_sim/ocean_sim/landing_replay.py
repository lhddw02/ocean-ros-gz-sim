"""Asynchronous, paused world-reset handshake for a fresh landing flight."""
import time

from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.srv import ControlWorld
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String


class LandingReplay:
    def __init__(self, node, clear_flight):
        self.node = node
        self.clear_flight = clear_flight
        self.client = node.create_client(ControlWorld, '/world/ocean/control')
        self.publisher = node.create_publisher(String, '/landing/replay_status', 1)
        self.request_subscription = node.create_subscription(
            Bool, '/landing/replay', self.request, 1)
        self.clock_subscription = node.create_subscription(
            Clock, '/clock', self.clock, qos_profile_sensor_data)
        self.phase = None
        self.error = None
        self.future = None
        self.deadline = None
        self.armed_since = None
        self.clock_sequence = 0
        self.request_clock_sequence = 0
        self.last_clock = None
        self.last_clock_wall = None
        self.replay_count = 0
        self.last_publish = -1.0

    @property
    def holding(self):
        return self.phase is not None or self.error is not None

    @property
    def arm_motors(self):
        return self.error is None and self.phase in ('arming', 'resuming')

    def clock(self, message):
        self.clock_sequence += 1
        self.last_clock = message.clock.sec + message.clock.nanosec * 1e-9
        self.last_clock_wall = time.monotonic()

    def request(self, message):
        if not message.data or self.phase is not None:
            return
        if not self.client.service_is_ready():
            self.fail('World control is unavailable; wait for Gazebo, then retry.')
            return
        if self.last_clock is None:
            self.fail('No simulation clock; wait for Gazebo, then retry.')
            return
        self.error = None
        self.phase = 'resetting'
        self.deadline = time.monotonic() + 30.0
        self.request_clock_sequence = self.clock_sequence
        # The success response only acknowledges queuing. tick() also waits for
        # a new zero-time clock sample before touching the old flight's latch.
        request = ControlWorld.Request()
        request.world_control.pause = True
        request.world_control.reset.all = True
        self.future = self.client.call_async(request)
        self.node.command_publisher.publish(Twist())
        self.node.enable_publisher.publish(Bool(data=False))
        self.node.get_logger().info('Replay requested: pausing and resetting the scene.')
        self.publish(force=True)

    def fail(self, reason):
        self.error = reason
        self.phase = None
        self.future = None
        self.node.command_publisher.publish(Twist())
        self.node.enable_publisher.publish(Bool(data=False))
        # Keep guidance inhibited and motors disabled after any partial reset.
        # A failed unpause is ambiguous, so ask Gazebo to pause again as well.
        if self.client.service_is_ready():
            request = ControlWorld.Request()
            request.world_control.pause = True
            self.client.call_async(request)
        self.node.get_logger().error('Replay failed: ' + reason)
        self.publish(force=True)

    def completed(self):
        if self.future is None or not self.future.done():
            return False
        try:
            result = self.future.result()
        except Exception as error:
            self.fail('World control request failed: ' + str(error))
            return False
        if result is None or not result.success:
            self.fail('Gazebo rejected world control; retry the replay button.')
            return False
        return True

    def tick(self):
        steady = time.monotonic()
        if self.phase is not None and steady > self.deadline:
            self.fail('Timed out waiting for reset/arming; guidance held, pause requested.')
        if self.phase == 'resetting' and self.completed():
            # Reject pre-reset DDS clock samples, including the old paused time.
            reset_applied = (self.clock_sequence > self.request_clock_sequence and
                             self.last_clock is not None and self.last_clock <= 0.05 and
                             abs(self.node.get_clock().now().nanoseconds * 1e-9) <= 0.05)
            if reset_applied:
                self.clear_flight()
                self.phase = 'arming'
                self.armed_since = steady
                self.future = None
                self.node.get_logger().info('Replay reset applied; arming at paused time zero.')
        elif self.phase == 'arming':
            connected = (self.node.command_publisher.get_subscription_count() > 0 and
                         self.node.enable_publisher.get_subscription_count() > 0)
            if not connected:
                self.armed_since = steady
            elif steady - self.armed_since >= 1.0:
                # Reset is a new automatic flight even if AUTOLAND=false was
                # used for the initial hover. The normal enabled parameter can
                # still pause guidance again later.
                self.node.set_parameters([Parameter('enabled', value=True)])
                request = ControlWorld.Request()
                request.world_control.pause = False
                self.future = self.client.call_async(request)
                self.phase = 'resuming'
        elif self.phase == 'resuming' and self.completed():
            self.clear_flight()
            self.replay_count += 1
            self.phase = None
            self.future = None
            self.node.get_logger().info(f'Replay {self.replay_count}: fresh landing started.')
        self.publish()

    def publish(self, force=False):
        steady = time.monotonic()
        if not force and steady - self.last_publish < 0.2:
            return
        self.last_publish = steady
        if self.error:
            value = 'error: ' + self.error
        elif self.phase is not None:
            value = 'arming' if self.phase == 'resuming' else self.phase
        elif self.last_clock is None or self.node.last_image_time is None:
            value = 'starting'
        elif self.node.landed:
            value = 'landed'
        else:
            value = 'replaying' if self.node.p('enabled') else 'ready'
        self.publisher.publish(String(data=value))
