#!/usr/bin/env python3
"""Verify rendered-tag guidance and physical landing; never command the drone.

Ground-truth model poses are used only here to independently assess the result.
The landing controller itself guides from the rendered camera and contacts.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def relative_position(station, drone):
    """Express the drone origin in the moving station's full 3-D frame."""
    q = station.orientation
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = float(np.linalg.norm(values))
    if not np.isfinite(values).all() or norm < 1e-8:
        raise ValueError('Invalid station quaternion')
    x, y, z, w = values / norm
    rotation = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])
    delta = np.array([drone.position.x - station.position.x,
                      drone.position.y - station.position.y,
                      drone.position.z - station.position.z])
    result = rotation.T @ delta
    if not np.isfinite(result).all():
        raise ValueError('Non-finite ground-truth pose')
    return tuple(float(value) for value in result)


def valid_contact(first, second):
    def foot(name):
        return 'landing_drone' in name and any(
            item in name for item in ('foot_fl', 'foot_fr', 'foot_rl', 'foot_rr'))
    def station(name):
        return 'floating_station' in name and 'hull_collision' in name
    return ((foot(first) and station(second)) or
            (foot(second) and station(first)))


def decode_image(message):
    formats = {'rgb8': (3, cv2.COLOR_RGB2BGR), 'bgr8': (3, None),
               'rgba8': (4, cv2.COLOR_RGBA2BGR), 'bgra8': (4, cv2.COLOR_BGRA2BGR),
               'mono8': (1, cv2.COLOR_GRAY2BGR)}
    if message.encoding not in formats:
        raise ValueError('Unsupported image encoding: ' + message.encoding)
    channels, conversion = formats[message.encoding]
    if message.step < message.width * channels or len(message.data) < message.step * message.height:
        raise ValueError('Truncated camera image')
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    pixels = rows[:, :message.width * channels].reshape(message.height, message.width, channels)
    if channels == 1:
        pixels = pixels[:, :, 0]
    return cv2.cvtColor(pixels, conversion) if conversion is not None else pixels.copy()


class LandingProbe:
    def __init__(self, args):
        self.args = args
        self.output = Path(args.output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.node = rclpy.create_node('ocean_landing_probe')
        self.started = time.monotonic()
        self.clock = None
        self.first_clock = None
        self.last_clock_wall = None
        self.station = self.drone = None
        self.station_wall = self.drone_wall = None
        self.station_anchor = None
        self.station_heights = []
        self.max_station_drift = 0.0
        self.status = {}
        self.status_wall = None
        self.status_count = 0
        self.states = []
        self.frames = self.camera_infos = self.tag_poses = 0
        self.independent_detections = 0
        self.contact_messages = self.foot_contact_messages = 0
        self.last_contact_clock = self.last_contact_wall = None
        self.first_contact_clock = None
        self.first_motor_off_clock = None
        self.calibration = None
        self.last_image = None
        self.last_image_wall = None
        self.last_detection_wall = 0.0
        self.last_image_stamp = None
        self.near_saved = False
        self.first_saved = False
        self.image_errors = []
        self.malformed_status = 0
        self.settled_since = None
        self.settled_seconds = 0.0
        self.last_relative = None
        self.relative_samples = []
        self.last_sample_clock = None
        self.last_progress = self.started
        self.families = {'tag16h5': cv2.aruco.DICT_APRILTAG_16h5,
                         'tag25h9': cv2.aruco.DICT_APRILTAG_25h9,
                         'tag36h11': cv2.aruco.DICT_APRILTAG_36h11}
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        cv2.setNumThreads(1)
        q = qos_profile_sensor_data
        self.subscriptions = [
            self.node.create_subscription(Clock, '/clock', self.on_clock, q),
            self.node.create_subscription(PoseStamped, '/station/pose', self.on_station, q),
            self.node.create_subscription(PoseStamped, '/drone/pose', self.on_drone, q),
            self.node.create_subscription(String, '/landing/status', self.on_status, q),
            self.node.create_subscription(CameraInfo, '/drone/camera/camera_info', self.on_info, q),
            self.node.create_subscription(Image, '/drone/camera/image_raw', self.on_image, q),
            self.node.create_subscription(PoseStamped, '/landing/tag_pose', self.on_tag_pose, q),
            self.node.create_subscription(Contacts, '/drone/contacts', self.on_contacts, q),
        ]

    def on_clock(self, message):
        value = seconds(message.clock)
        if self.clock is not None and value < self.clock - 0.1:
            raise RuntimeError('Simulation clock reset during verification')
        if self.first_clock is None:
            self.first_clock = value
        if self.clock is None or value > self.clock:
            self.last_clock_wall = time.monotonic()
        self.clock = value

    def on_station(self, message):
        self.station = message.pose
        self.station_wall = time.monotonic()
        p = message.pose.position
        if not all(math.isfinite(value) for value in (p.x, p.y, p.z)):
            raise RuntimeError('Station pose became non-finite')
        if self.station_anchor is None:
            self.station_anchor = (p.x, p.y)
        self.max_station_drift = max(self.max_station_drift, math.hypot(
            p.x - self.station_anchor[0], p.y - self.station_anchor[1]))
        self.station_heights.append(float(p.z))

    def on_drone(self, message):
        self.drone = message.pose
        self.drone_wall = time.monotonic()

    def on_status(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict) or 'state' not in value:
                raise ValueError('Missing state')
        except (ValueError, TypeError):
            self.malformed_status += 1
            return
        self.status = value
        self.status_wall = time.monotonic()
        self.status_count += 1
        if not self.states or self.states[-1]['state'] != value['state']:
            transition = {'state': value['state'], 'sim_time': self.clock,
                          'wall_elapsed_s': round(self.status_wall - self.started, 3)}
            self.states.append(transition)
            print('Landing state:', value['state'], flush=True)
        if value.get('motors_enabled') is False and self.first_motor_off_clock is None:
            self.first_motor_off_clock = self.clock

    def on_info(self, message):
        matrix = np.array(message.k, dtype=float).reshape(3, 3)
        if (np.isfinite(matrix).all() and matrix[0, 0] > 0 and matrix[1, 1] > 0
                and message.width > 0 and message.height > 0):
            self.calibration = {'width': int(message.width), 'height': int(message.height),
                                'k': list(message.k), 'd': list(message.d)}
            self.camera_infos += 1

    def on_tag_pose(self, message):
        p = message.pose.position
        if all(math.isfinite(value) for value in (p.x, p.y, p.z)) and 0.03 < p.z < 100:
            self.tag_poses += 1

    def marker_detection(self, image):
        family = self.status.get('tag_family', self.args.tag_family)
        family = family if family in self.families else self.args.tag_family
        tag_id = self.status.get('tag_id', self.args.tag_id)
        dictionary = cv2.aruco.getPredefinedDictionary(self.families[family])
        corners, ids, _ = cv2.aruco.detectMarkers(
            cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), dictionary, parameters=self.parameters)
        selected = ids is not None and int(tag_id) in ids.flatten()
        return corners, ids, bool(selected)

    def save_snapshot(self, label, image):
        raw_path = self.output / (label + '-camera.png')
        annotated_path = self.output / (label + '-annotated.png')
        corners, ids, found = self.marker_detection(image)
        annotated = image.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
        state = self.status.get('state', 'no controller status yet')
        height = self.status.get('camera_height_m')
        text = f'{label} | {state} | independent tag: {found}'
        if isinstance(height, (int, float)) and math.isfinite(height):
            text += f' | camera height {height:.2f} m'
        cv2.rectangle(annotated, (0, 0), (annotated.shape[1], 34), (20, 20, 20), -1)
        cv2.putText(annotated, text, (10, 23), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (255, 255, 255), 1, cv2.LINE_AA)
        if not cv2.imwrite(str(raw_path), image) or not cv2.imwrite(str(annotated_path), annotated):
            raise RuntimeError('Could not write camera evidence into ' + str(self.output))
        metadata = {'sim_time': self.clock, 'image_stamp': self.last_image_stamp,
                    'status': self.status, 'independent_target_detected': found}
        (self.output / (label + '-metadata.json')).write_text(
            json.dumps(metadata, indent=2, allow_nan=False) + '\n', encoding='utf-8')

    def on_image(self, message):
        try:
            image = decode_image(message)
        except ValueError as exc:
            if str(exc) not in self.image_errors:
                self.image_errors.append(str(exc))
            return
        self.last_image = image
        self.last_image_wall = time.monotonic()
        self.last_image_stamp = seconds(message.header.stamp)
        self.frames += 1
        if self.last_image_wall - self.last_detection_wall >= 0.5:
            _, _, found = self.marker_detection(image)
            self.independent_detections += int(found)
            self.last_detection_wall = self.last_image_wall
        if not self.first_saved:
            self.save_snapshot('first', image)
            self.first_saved = True
        height = self.status.get('camera_height_m')
        if (not self.near_saved and isinstance(height, (int, float))
                and math.isfinite(height) and 0 < height < 1.0):
            self.save_snapshot('near', image)
            self.near_saved = True

    def on_contacts(self, message):
        self.contact_messages += 1
        if any(valid_contact(item.collision1.name, item.collision2.name)
               for item in message.contacts):
            self.foot_contact_messages += 1
            self.last_contact_clock = self.clock
            self.last_contact_wall = time.monotonic()
            if self.first_contact_clock is None:
                self.first_contact_clock = self.clock

    def criteria(self):
        now = time.monotonic()
        relative = None
        if self.station is not None and self.drone is not None:
            relative = relative_position(self.station, self.drone)
            self.last_relative = relative
        contact_recent = (self.last_contact_clock is not None and self.clock is not None
                          and self.clock - self.last_contact_clock <= 0.35
                          and now - self.last_contact_wall <= 2.0)
        pose_recent = (self.station_wall is not None and self.drone_wall is not None
                       and now - self.station_wall < 2 and now - self.drone_wall < 2)
        height_ok = relative is not None and abs(relative[2] - 0.80) <= 0.16
        centered = relative is not None and math.hypot(*relative[:2]) <= 0.35
        within_deck = (relative is not None and abs(relative[0]) < 1.15
                       and abs(relative[1]) < 1.15)
        heave = max(self.station_heights) - min(self.station_heights) if self.station_heights else 0
        return {
            'camera_frames_received': self.frames >= 10,
            'valid_camera_calibration': self.camera_infos > 0,
            'actual_tag_pose_messages': self.tag_poses >= 5,
            'controller_tag_detections': self.status.get('detections', 0) >= 10,
            'independently_detected_tag_in_raw_images': self.independent_detections >= 2,
            'physical_foot_station_contact': contact_recent and self.foot_contact_messages >= 3,
            'controller_landed': self.status.get('state') == 'landed',
            'motors_disabled': self.status.get('motors_enabled') is False,
            'status_reports_station_contact': self.status.get('foot_station_contact') is True,
            'fresh_controller_status': self.status_wall is not None and now - self.status_wall < 2,
            'fresh_ground_truth_poses': pose_recent,
            'ground_truth_center_within_0_35m': centered,
            'all_feet_inside_deck_margin': within_deck,
            'ground_truth_landing_height_0_80m': height_ok,
            'station_horizontal_drift_below_0_35m': self.max_station_drift < 0.35,
            'station_wave_heave_above_0_01m': heave > 0.01,
        }

    def run(self):
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            now = time.monotonic()
            elapsed = now - self.started
            criteria = self.criteria()
            if self.last_relative is not None:
                if self.clock is not None and (self.last_sample_clock is None or
                        self.clock - self.last_sample_clock >= 0.1):
                    self.relative_samples.append({'sim_time': self.clock,
                                                  'xyz_m': self.last_relative})
                    self.last_sample_clock = self.clock
                if self.last_relative[2] < -0.5:
                    raise RuntimeError('Drone fell below the station; landing dynamics or camera guidance failed')
                if any(abs(value) > 100 for value in self.last_relative):
                    raise RuntimeError('Drone escaped the simulation test area')
            if all(criteria.values()) and self.clock is not None:
                if self.settled_since is None:
                    self.settled_since = self.clock
                self.settled_seconds = self.clock - self.settled_since
                if self.settled_seconds >= self.args.settle_seconds:
                    return
            else:
                self.settled_since = None
                self.settled_seconds = 0.0
            if elapsed >= self.args.wall_timeout:
                raise RuntimeError('Wall-clock landing deadline exceeded')
            if self.clock is not None and self.clock - self.first_clock >= self.args.sim_timeout:
                raise RuntimeError('Simulation-time landing deadline exceeded')
            if elapsed > 20 and self.first_clock is None:
                raise RuntimeError('No ROS simulation clock; check Gazebo launch and clock bridge')
            if self.last_clock_wall is not None and now - self.last_clock_wall > 20:
                raise RuntimeError('Simulation clock stopped advancing; check Gazebo crash log')
            if elapsed > 30 and self.frames == 0:
                raise RuntimeError('No rendered downward-camera images; check Sensors/Ogre2 and image bridge')
            if self.last_image_wall is not None and now - self.last_image_wall > 20:
                raise RuntimeError('Downward-camera image stream stopped')
            if elapsed > 30 and self.status_count == 0:
                raise RuntimeError('No landing-controller status; check controller startup')
            if elapsed > 50 and self.tag_poses == 0:
                raise RuntimeError('No valid AprilTag pose; check texture, selected tag ID, visibility and calibration')
            if now - self.last_progress >= 10:
                missing = [key for key, value in criteria.items() if not value]
                print(f'Landing verification: frames={self.frames}, tag poses={self.tag_poses}, '
                      f'foot contacts={self.foot_contact_messages}, state={self.status.get("state")}; '
                      f'pending={", ".join(missing)}', flush=True)
                self.last_progress = now

    def report(self, success, error=None):
        if self.last_image is not None:
            self.save_snapshot('final', self.last_image)
        try:
            conditions = self.criteria()
        except (ValueError, TypeError):
            conditions = {'finite_ground_truth_poses': False}
        heave = max(self.station_heights) - min(self.station_heights) if self.station_heights else 0
        data = {
            'passed': success, 'error': error,
            'ground_truth_used_for': 'verification only; no flight commands are published by this probe',
            'wall_elapsed_s': round(time.monotonic() - self.started, 3),
            'simulation_elapsed_s': (self.clock - self.first_clock if self.clock is not None else None),
            'settled_simulation_seconds': self.settled_seconds,
            'criteria': conditions,
            'missing_criteria': [key for key, value in conditions.items() if not value],
            'camera_frames': self.frames, 'camera_info_messages': self.camera_infos,
            'camera_calibration': self.calibration, 'tag_pose_messages': self.tag_poses,
            'independent_tag_detection_frames': self.independent_detections,
            'contact_messages': self.contact_messages,
            'physical_foot_station_contact_messages': self.foot_contact_messages,
            'first_contact_sim_time': self.first_contact_clock,
            'first_motor_off_sim_time': self.first_motor_off_clock,
            'station_max_horizontal_drift_m': self.max_station_drift,
            'station_heave_range_m': heave,
            'final_drone_in_station_frame_m': self.last_relative,
            'final_status': self.status, 'state_transitions': self.states,
            'image_decode_errors': self.image_errors,
            'malformed_status_messages': self.malformed_status,
        }
        (self.output / 'landing-report.json').write_text(
            json.dumps(data, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        (self.output / 'relative-pose-samples.json').write_text(
            json.dumps(self.relative_samples, separators=(',', ':'), allow_nan=False) + '\n',
            encoding='utf-8')
        print(('PASS' if success else 'FAIL') + ': ' + (error or
              'rendered AprilTag detected; real feet/station contact; centered landing with motors off'),
              flush=True)
        print('Evidence directory:', self.output, flush=True)
        if not success:
            print('Missing criteria:', ', '.join(data['missing_criteria']), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='/tmp/ocean-landing-verification')
    parser.add_argument('--wall-timeout', type=float, default=180)
    parser.add_argument('--sim-timeout', type=float, default=100)
    parser.add_argument('--settle-seconds', type=float, default=2)
    parser.add_argument('--tag-family', choices=('tag16h5', 'tag25h9', 'tag36h11'), default='tag36h11')
    parser.add_argument('--tag-id', type=int, default=0)
    args = parser.parse_args()
    if min(args.wall_timeout, args.sim_timeout, args.settle_seconds) <= 0:
        parser.error('Timeouts and settling duration must be positive')
    rclpy.init(args=[])
    probe = None
    result = 1
    try:
        probe = LandingProbe(args)
        probe.run()
        probe.report(True)
        result = 0
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) or type(exc).__name__
        if probe is not None:
            probe.report(False, error)
        else:
            print('FAIL: probe initialization: ' + error, file=sys.stderr, flush=True)
    finally:
        if probe is not None:
            probe.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    sys.exit(main())
