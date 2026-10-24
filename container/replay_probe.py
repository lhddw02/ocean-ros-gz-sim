#!/usr/bin/env python3
"""Exercise landing replay without restarting any Gazebo or Docker process.

Run against a fresh landing.launch.py scene started with autoland:=false.
The actual GUI button's widget is tested separately; this probe publishes
its native Gazebo Transport request through the same bridge and real physics.
Independent landing_probe.py subprocesses assess the three completed flights.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from ros_gz_interfaces.srv import ControlWorld
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ReplayProbe:
    def __init__(self, args):
        self.args = args
        self.output = Path(args.output).resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.started = time.monotonic()
        self.node = rclpy.create_node('ocean_replay_probe')
        latest = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                            reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.gz_node = None
        self.gz_boolean = None
        if args.request_transport == 'gz':
            from gz.transport13 import Node as GzNode
            from gz.msgs10.boolean_pb2 import Boolean as GzBoolean
            self.gz_node = GzNode()
            self.gz_boolean = GzBoolean
            self.replay_request = self.gz_node.advertise('/landing/replay', GzBoolean)
        else:
            self.replay_request = self.node.create_publisher(Bool, '/landing/replay', 10)
        self.control = self.node.create_client(ControlWorld, '/world/ocean/control')
        self.subscriptions = [
            self.node.create_subscription(Clock, '/clock', self.on_clock, latest),
            self.node.create_subscription(String, '/landing/status', self.on_status, latest),
            self.node.create_subscription(String, '/landing/replay_status', self.on_phase, latest),
            self.node.create_subscription(Image, '/drone/camera/image_raw', self.on_image, latest),
        ]
        self.clock = None
        self.last_clock_advance = None
        self.status = {}
        self.status_wall = None
        self.phase = None
        self.rewinds = []
        self.timeline = []
        self.requests = []
        self.completed = []
        self.last_transition = None
        self.image_count = 0
        self.epoch_images = {}
        self.active_process = None
        self.active_log = None
        self.last_progress = self.started

    def event(self, name, **values):
        self.timeline.append({'event': name, 'wall_elapsed_s': round(
            time.monotonic() - self.started, 3), 'sim_time': self.clock, **values})

    def on_clock(self, message):
        value = seconds(message.clock)
        if self.clock is not None and value < self.clock - 0.1:
            rewind = {'from_s': self.clock, 'to_s': value,
                      'wall_elapsed_s': round(time.monotonic() - self.started, 3)}
            self.rewinds.append(rewind)
            self.event('clock_rewind', **rewind)
        if self.clock is None or value > self.clock:
            self.last_clock_advance = time.monotonic()
        self.clock = value

    def on_status(self, message):
        value = json.loads(message.data)
        if not isinstance(value, dict) or 'replay_count' not in value:
            raise RuntimeError('Landing status lacks replay_count; rebuild the replay-enabled image')
        self.status = value
        self.status_wall = time.monotonic()
        transition = (value.get('replay_count'), value.get('replay_phase'),
                      value.get('state'), value.get('motors_enabled'))
        if transition != self.last_transition:
            self.last_transition = transition
            self.event('controller_transition', replay_count=transition[0],
                       replay_phase=transition[1], state=transition[2],
                       motors_enabled=transition[3], detections=value.get('detections'),
                       image_age_s=value.get('image_age_s'))
            print(f'Replay {transition[0]}: {transition[2]} '
                  f'(phase={transition[1]}, motors={transition[3]})', flush=True)

    def on_phase(self, message):
        if message.data != self.phase:
            self.phase = message.data
            self.event('button_status', value=self.phase)

    def on_image(self, message):
        self.image_count += 1
        # Count only frames whose timestamp belongs to the currently observed
        # simulation epoch; queued images from the previous flight do not count.
        stamp = seconds(message.header.stamp)
        if self.clock is not None and -0.15 <= self.clock - stamp <= 0.5:
            epoch = len(self.rewinds)
            entry = self.epoch_images.setdefault(epoch, {'frames': 0,
                                                        'first_stamp_s': stamp})
            entry['frames'] += 1
            entry['last_stamp_s'] = stamp

    def spin(self):
        rclpy.spin_once(self.node, timeout_sec=0.02)
        now = time.monotonic()
        if now - self.started >= self.args.total_timeout:
            raise RuntimeError('Overall replay verification deadline exceeded')
        error = self.status.get('replay_error')
        if error:
            raise RuntimeError('Controller replay failed: ' + str(error))
        if now - self.last_progress >= 10:
            print(f'Replay check: count={self.status.get("replay_count")}, '
                  f'clock={self.clock}, state={self.status.get("state")}, '
                  f'rewinds={len(self.rewinds)}', flush=True)
            self.last_progress = now

    def wait(self, predicate, description, timeout=35.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.spin()
            if time.monotonic() >= deadline:
                raise RuntimeError('Timed out waiting for ' + description)

    def delay(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.spin()

    def request(self, number, reason, burst=False, complete=False):
        expected_previous = number - 1
        if self.status.get('replay_count') != expected_previous:
            raise RuntimeError(f'Expected replay_count={expected_previous} before replay {number}')
        if self.status.get('replay_phase') is not None:
            raise RuntimeError('Another replay is still in progress before request')
        self.wait(self.replay_connected,
                  'the replay request subscription')
        before = len(self.rewinds)
        request = {'number': number, 'reason': reason, 'burst_messages': 5 if burst else 1,
                   'requested_at_sim_time': self.clock,
                   'requested_from_state': self.status.get('state'),
                   'request_transport': self.args.request_transport}
        self.requests.append(request)
        self.event('request', **request)
        for index in range(request['burst_messages']):
            if self.args.request_transport == 'gz':
                message = self.gz_boolean()
                message.data = True
                if not self.replay_request.publish(message):
                    raise RuntimeError('Gazebo Transport rejected the replay request')
            else:
                self.replay_request.publish(Bool(data=True))
            if burst and index < 4:
                self.delay(0.05)
        self.wait(lambda: len(self.rewinds) == before + 1 and
                  self.status.get('replay_phase') == 'arming' and
                  self.clock is not None and self.clock <= 0.05,
                  f'replay {number} reset and paused arming')
        request['arming_sim_time'] = self.clock
        request['arming_detections'] = self.status.get('detections')
        request['arming_image_age_s'] = self.status.get('image_age_s')
        request['rewind'] = self.rewinds[-1]
        if request['rewind']['to_s'] > 0.05:
            raise RuntimeError('World reset did not rewind the clock to zero')
        if request['arming_detections'] != 0 or request['arming_image_age_s'] is not None:
            raise RuntimeError('The paused replay retained image/detection state from the old flight')
        if complete:
            self.start_landing_probe(number)
        self.wait(lambda: self.status.get('replay_count') == number and
                  self.status.get('replay_phase') is None and
                  self.clock is not None and self.clock > 0.05,
                  f'replay {number} resumed with the expected count')
        self.wait(lambda: self.status.get('replay_count') == number and
                  self.status.get('target_fresh') is True and
                  self.status.get('detections', 0) >= 5 and
                  self.epoch_images.get(len(self.rewinds), {}).get('frames', 0) >= 5,
                  f'fresh rendered camera detections in replay {number}')
        request['fresh_detection_sim_time'] = self.clock
        if complete:
            self.finish_landing_probe(number)

    def replay_connected(self):
        if self.args.request_transport == 'gz':
            return self.replay_request.has_connections()
        return self.replay_request.get_subscription_count() > 0

    def start_landing_probe(self, number):
        if self.active_process is not None:
            raise RuntimeError('An independent landing probe is already running')
        directory = self.output / f'cycle-{number}'
        directory.mkdir(parents=True, exist_ok=True)
        self.active_log = (directory / 'probe.log').open('w', encoding='utf-8')
        command = [sys.executable, str(Path(__file__).with_name('landing_probe.py')),
                   '--output', str(directory), '--wall-timeout', str(self.args.landing_timeout),
                   '--sim-timeout', '100', '--settle-seconds', '2']
        self.active_process = subprocess.Popen(command, stdout=self.active_log,
                                               stderr=subprocess.STDOUT)
        self.event('independent_probe_started', replay=number,
                   phase=self.status.get('replay_phase'), directory=directory.name)

    def finish_landing_probe(self, number):
        process = self.active_process
        self.wait(lambda: process.poll() is not None,
                  f'physical landing verification for replay {number}',
                  timeout=self.args.landing_timeout + 10)
        result = process.returncode
        self.active_log.close()
        self.active_log = self.active_process = None
        directory = self.output / f'cycle-{number}'
        report_path = directory / 'landing-report.json'
        if not report_path.is_file():
            raise RuntimeError(f'Replay {number} independent probe produced no report; see {directory}/probe.log')
        report = json.loads(report_path.read_text(encoding='utf-8'))
        if result != 0 or report.get('passed') is not True:
            raise RuntimeError(f'Replay {number} physical landing failed: {report.get("error")}; '
                               f'see {directory}/landing-report.json')
        if self.status.get('replay_count') != number or self.status.get('state') != 'landed':
            raise RuntimeError('Landing finished in an unexpected replay epoch/state')
        if self.status.get('motors_enabled') is not False:
            raise RuntimeError('Landing completed without switching motors off')
        self.completed.append({'replay': number, 'passed': True,
                               'report': f'cycle-{number}/landing-report.json',
                               'final_drone_in_station_frame_m': report.get(
                                   'final_drone_in_station_frame_m'),
                               'settled_simulation_seconds': report.get('settled_simulation_seconds'),
                               'physical_foot_station_contact_messages': report.get(
                                   'physical_foot_station_contact_messages')})
        self.event('independent_probe_passed', replay=number)

    def pause(self):
        self.wait(lambda: self.control.service_is_ready(), 'world-control service')
        request = ControlWorld.Request()
        request.world_control.pause = True
        future = self.control.call_async(request)
        self.wait(future.done, 'Gazebo pause acknowledgement')
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('Gazebo rejected the paused-world replay prerequisite')
        self.wait(lambda: self.last_clock_advance is not None and
                  time.monotonic() - self.last_clock_advance >= 0.8,
                  'simulation clock to stop while paused')
        before = self.clock
        self.delay(0.4)
        if self.clock is None or abs(self.clock - before) > 0.01:
            raise RuntimeError('Simulation clock advanced despite pausing physics')
        self.event('physics_confirmed_paused', paused_sim_time=self.clock)

    def run(self):
        self.wait(lambda: self.status.get('state') == 'disabled_hover' and
                  self.status.get('replay_count') == 0 and
                  self.status.get('target_fresh') is True and
                  self.clock is not None and self.clock > 0.5,
                  'fresh AUTOLAND=false scene with a visible AprilTag', timeout=60)
        self.event('initial_scene_ready')
        # One physical landing after a burst of clicks tests busy-state dedupe.
        self.request(1, 'fresh hover; burst of five requests', burst=True, complete=True)
        # A completed flight must replay, and an airborne flight must also reset.
        self.request(2, 'replay after physical touchdown')
        self.wait(lambda: self.clock is not None and self.clock >= 3.0 and
                  self.status.get('state') in ('aligning', 'descending', 'flaring') and
                  self.status.get('motors_enabled') is True,
                  'replay 2 airborne before the midflight interruption')
        self.request(3, 'interrupt an active airborne landing', complete=True)
        self.pause()
        self.request(4, 'replay while physics is paused', complete=True)
        if len(self.rewinds) != 4 or len(self.requests) != 4:
            raise RuntimeError('Replay requests caused missing or duplicate world resets')
        if self.status.get('replay_count') != 4:
            raise RuntimeError('Replay count changed unexpectedly after the fourth reset')
        if [entry['replay'] for entry in self.completed] != [1, 3, 4]:
            raise RuntimeError('Not all three requested complete landings passed independently')

    def stop_child(self):
        if self.active_process is not None and self.active_process.poll() is None:
            self.active_process.terminate()
            try:
                self.active_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.active_process.kill()
                self.active_process.wait(timeout=5)
        if self.active_log is not None:
            self.active_log.close()

    def report(self, passed, error=None):
        data = {
            'passed': passed, 'error': error,
            'wall_elapsed_s': round(time.monotonic() - self.started, 3),
            'request_transport': self.args.request_transport,
            'test_scope': ('native Gazebo replay request through the GUI transport bridge'
                           if self.args.request_transport == 'gz' else 'ROS replay backend') +
                          ', real reset/arming, rendered tag, and physical touchdown; '
                          'GUI widget and unchanged process identities checked separately',
            'requests': self.requests, 'clock_rewinds': self.rewinds,
            'completed_landings': self.completed,
            'fresh_camera_frames_by_epoch': self.epoch_images,
            'final_controller_status': self.status, 'timeline': self.timeline,
        }
        (self.output / 'replay-report.json').write_text(
            json.dumps(data, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        print(('PASS' if passed else 'FAIL') + ': ' + (error or
              'request burst, completed-flight replay, midflight replay, and paused-world replay; '
              'three independent physical landings passed'), flush=True)
        print('Evidence directory:', self.output, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    default = '/recordings/replay-check-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    parser.add_argument('--output', default=default)
    parser.add_argument('--landing-timeout', type=float, default=180)
    parser.add_argument('--total-timeout', type=float, default=660)
    parser.add_argument('--request-transport', choices=('gz', 'ros'), default='gz',
                        help='native Gazebo Transport (the GUI path) or direct ROS backend')
    args = parser.parse_args()
    if min(args.landing_timeout, args.total_timeout) <= 0:
        parser.error('Timeouts must be positive')
    rclpy.init(args=[])
    probe = None
    result = 1
    try:
        probe = ReplayProbe(args)
        probe.run()
        probe.report(True)
        result = 0
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) or type(exc).__name__
        if probe is not None:
            probe.report(False, error)
        else:
            print('FAIL: replay probe initialization: ' + error, file=sys.stderr, flush=True)
    finally:
        if probe is not None:
            probe.stop_child()
            probe.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    sys.exit(main())
