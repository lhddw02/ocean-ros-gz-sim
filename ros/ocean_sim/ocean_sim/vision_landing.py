"""AprilTag camera pose estimation and contact-confirmed, vision-guided landing.

No station/world pose or GPS is consumed. All translational guidance comes from
the selected marker in the actual rendered, calibrated downward-camera images.
Gazebo's multicopter controller supplies the attitude/velocity inner loop.
"""

import json
import math
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy,
    QoSHistoryPolicy, QoSDurabilityPolicy,
)
from rclpy.clock import Clock, ClockType
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from ocean_sim.landing_geometry import (
    foot_station_contact,
    optical_to_body,
    quaternion_from_matrix,
    velocity_guidance,
)
from ocean_sim.landing_replay import LandingReplay


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class VisionLanding(Node):
    def __init__(self):
        super().__init__('vision_landing')
        defaults = {
            'enabled': True,
            'tag_family': 'tag36h11',
            'tag_id': 0,
            'tag_size': 0.4,
            'camera_optical_frame': 'drone/camera_optical_frame',
            'camera_position': [0.0, 0.0, -0.10],
            'camera_clearance': 0.35,
            'station_name': 'floating_station',
            'image_topic': '/drone/camera/image_raw',
            'camera_info_topic': '/drone/camera/camera_info',
            'contacts_topic': '/drone/contacts',
            'target_timeout': 0.35,
            'horizontal_gain': 0.85,
            'horizontal_limit': 0.55,
            'alignment_tolerance': 0.12,
            'alignment_hold': 0.65,
            'descend_rate': 0.35,
            'flare_rate': 0.10,
            'contact_hold': 0.30,
            'max_reprojection_error': 3.0,
            'debug_image_rate': 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('OpenCV-contrib is required: cv2.aruco is missing')
        # Leave CPU time for Gazebo / DDS; nested OpenCV worker pools otherwise
        # contend with the physics and rendering processes in a small WSL VM.
        cv2.setNumThreads(1)
        self.families = {
            'tag16h5': cv2.aruco.DICT_APRILTAG_16h5,
            'tag25h9': cv2.aruco.DICT_APRILTAG_25h9,
            'tag36h11': cv2.aruco.DICT_APRILTAG_36h11,
        }
        if self.p('tag_family') not in self.families:
            raise ValueError('Supported tag families: ' + ', '.join(self.families))
        self.dictionary = cv2.aruco.getPredefinedDictionary(self.families[self.p('tag_family')])
        if not 0 <= self.p('tag_id') < len(self.dictionary.bytesList):
            raise ValueError('tag_id is outside the selected dictionary')
        initial_validation = self._parameters_changed(self.get_parameters(list(defaults)))
        if not initial_validation.successful:
            raise ValueError(initial_validation.reason)
        self.detector_parameters = cv2.aruco.DetectorParameters_create()
        self.detector_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        self.intrinsic = None
        self.distortion = None
        self.image_size = None
        self.last_image_time = None
        self.last_tag_time = None
        self.offset = None
        self.camera_height = None
        self.reprojection_error = None
        self.align_since = None
        self.contact_since = None
        self.last_contact_time = None
        self.has_contact = False
        self.landed = False
        self.state = 'waiting_for_camera'
        self.last_now = 0.0
        self.detections = 0
        self.last_status_time = -1.0
        self.warned_encoding = False
        self.last_debug_time = -math.inf
        self.processing_ms = 0.0
        self.processing_ms_ema = 0.0
        self.image_callbacks = 0
        self.fallback_calls = 0
        self.image_arrival_age_s = None
        self.motors_enabled = True
        self.last_landed_image_time = -math.inf
        self.add_on_set_parameters_callback(self._parameters_changed)

        self.command_publisher = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.enable_publisher = self.create_publisher(Bool, '/drone/enable', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/landing/tag_pose', 10)
        self.debug_publisher = self.create_publisher(Image, '/landing/debug_image', 2)
        self.state_publisher = self.create_publisher(String, '/landing/state', 10)
        self.status_publisher = self.create_publisher(String, '/landing/status', 10)
        self.replay = LandingReplay(self, self._clear_flight)
        self.create_subscription(
            CameraInfo, self.p('camera_info_topic'), self._camera_info,
            qos_profile_sensor_data)
        latest_image_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(
            Image, self.p('image_topic'), self._image, latest_image_qos)
        self.create_subscription(
            Contacts, self.p('contacts_topic'), self._contacts,
            qos_profile_sensor_data)
        # The scene starts paused. Publish hover/enable before /clock advances,
        # while every measurement age and transition still uses simulation time.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self._control, clock=self.steady_clock)
        self.get_logger().info(
            f"Watching {self.p('tag_family')} ID {self.p('tag_id')}, black border "
            f"{self.p('tag_size'):.3f} m; optical right=-body Y, down=-body X")

    def p(self, name):
        return self.get_parameter(name).value

    def _clear_flight(self):
        """Forget the previous epoch's measurements and motor-off latch."""
        self.intrinsic = self.distortion = self.image_size = None
        self.last_tag_time = self.last_image_time = None
        self.offset = self.camera_height = self.reprojection_error = None
        self.align_since = self.contact_since = self.last_contact_time = None
        self.landed = self.has_contact = False
        self.detections = self.image_callbacks = self.fallback_calls = 0
        self.processing_ms = self.processing_ms_ema = 0.0
        self.image_arrival_age_s = None
        self.last_debug_time = -math.inf
        self.last_landed_image_time = -math.inf

    def _parameters_changed(self, parameters):
        effective = {'tag_family': self.p('tag_family'), 'tag_id': self.p('tag_id')}
        effective.update({parameter.name: parameter.value for parameter in parameters
                          if parameter.name in effective})
        if effective['tag_family'] not in self.families:
            return SetParametersResult(successful=False, reason='Unsupported AprilTag family')
        dictionary = cv2.aruco.getPredefinedDictionary(self.families[effective['tag_family']])
        if not 0 <= effective['tag_id'] < len(dictionary.bytesList):
            return SetParametersResult(successful=False,
                                       reason='tag_id is outside the selected dictionary')
        for parameter in parameters:
            if parameter.name in ('tag_size', 'camera_clearance', 'target_timeout',
                                   'horizontal_gain', 'horizontal_limit',
                                   'alignment_tolerance', 'alignment_hold',
                                   'descend_rate', 'flare_rate', 'contact_hold',
                                   'max_reprojection_error'):
                if (not isinstance(parameter.value, (int, float)) or
                        not math.isfinite(parameter.value) or parameter.value <= 0):
                    return SetParametersResult(successful=False,
                                               reason=parameter.name + ' must be finite and positive')
            if parameter.name == 'debug_image_rate':
                if (not math.isfinite(parameter.value) or
                        not 0 <= parameter.value <= 10):
                    return SetParametersResult(successful=False,
                                               reason='debug_image_rate must be 0 through 10 Hz')
            if parameter.name == 'camera_position':
                if (len(parameter.value) != 3 or
                        not all(math.isfinite(value) for value in parameter.value)):
                    return SetParametersResult(successful=False,
                                               reason='camera_position needs three finite values')
        if any(parameter.name in ('tag_family', 'tag_id', 'tag_size') for parameter in parameters):
            self.dictionary = dictionary
            self.last_tag_time = None
            self.offset = None
            self.align_since = None
        return SetParametersResult(successful=True)

    def _camera_info(self, message):
        intrinsic = np.array(message.k, dtype=np.float64).reshape(3, 3)
        if (not np.isfinite(intrinsic).all() or intrinsic[0, 0] <= 0 or
                intrinsic[1, 1] <= 0 or message.width == 0 or message.height == 0):
            return
        self.intrinsic = intrinsic
        self.distortion = np.array(message.d, dtype=np.float64)
        self.image_size = (message.width, message.height)

    def _decode(self, message):
        formats = {'rgb8': (3, cv2.COLOR_RGB2BGR),
                   'bgr8': (3, None),
                   'rgba8': (4, cv2.COLOR_RGBA2BGR),
                   'bgra8': (4, cv2.COLOR_BGRA2BGR),
                   'mono8': (1, cv2.COLOR_GRAY2BGR)}
        if message.encoding not in formats:
            if not self.warned_encoding:
                self.get_logger().error('Unsupported camera encoding: ' + message.encoding)
                self.warned_encoding = True
            return None
        channels, conversion = formats[message.encoding]
        if (message.step < message.width * channels or
                len(message.data) < message.step * message.height):
            return None
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.step)
        pixels = rows[:, :message.width * channels]
        pixels = pixels.reshape((message.height, message.width, channels))
        if channels == 1:
            pixels = pixels[:, :, 0]
        return cv2.cvtColor(pixels, conversion) if conversion is not None else pixels.copy()

    def _image(self, message):
        processing_start = time.perf_counter()
        # After motor shutdown, keep the preview/detector alive at 2 Hz and
        # leave time for contact, clock and replay callbacks. A cropped marker
        # can otherwise keep the expensive fallback running on every frame.
        if self.landed and not self.replay.holding:
            if processing_start - self.last_landed_image_time < 0.5:
                return
            self.last_landed_image_time = processing_start
        self.image_callbacks += 1
        self.image_arrival_age_s = (
            self.get_clock().now().nanoseconds * 1e-9 - stamp_seconds(message.header.stamp))
        try:
            self._process_image(message)
        finally:
            self.processing_ms = (time.perf_counter() - processing_start) * 1000.0
            self.processing_ms_ema = (0.15 * self.processing_ms +
                                      0.85 * self.processing_ms_ema)

    def _process_image(self, message):
        image_time = stamp_seconds(message.header.stamp)
        now = self.get_clock().now().nanoseconds * 1e-9
        # Queued pre-reset images have timestamps from the previous epoch.
        if image_time > now + 1.0:
            return
        image = self._decode(message)
        if image is None:
            return
        if self.replay.holding:
            self._debug_image(image, message)
            return
        self.last_image_time = image_time
        if self.intrinsic is None or self.image_size != (message.width, message.height):
            self._debug_image(image, message)
            return

        corners, ids, _ = cv2.aruco.detectMarkers(
            cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), self.dictionary,
            parameters=self.detector_parameters)
        if ids is None or self.p('tag_id') not in ids.flatten():
            self.fallback_calls += 1
            # A small marker can sit between threshold/raster samples. One
            # bounded alternate sampling pass improves distant-tag acquisition.
            # Convert the resulting corners to the ORIGINAL pixel calibration.
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            enlarged = cv2.resize(gray, None, fx=1.5, fy=1.5,
                                  interpolation=cv2.INTER_CUBIC)
            alternate_corners, alternate_ids, _ = cv2.aruco.detectMarkers(
                enlarged, self.dictionary, parameters=self.detector_parameters)
            if alternate_ids is not None and self.p('tag_id') in alternate_ids.flatten():
                scale = np.array([enlarged.shape[1] / gray.shape[1],
                                  enlarged.shape[0] / gray.shape[0]], dtype=np.float32)
                corners = [(item + 0.5) / scale - 0.5 for item in alternate_corners]
                ids = alternate_ids
        found = False
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            matches = np.where(ids.flatten() == self.p('tag_id'))[0]
            if len(matches):
                pixel_corners = corners[int(matches[0])].reshape(4, 2).astype(np.float64)
                size = self.p('tag_size') / 2.0
                # Required IPPE_SQUARE ordering: top-left, top-right,
                # bottom-right, bottom-left, in the marker's +Y-up frame.
                object_corners = np.array([[-size, size, 0], [size, size, 0],
                                           [size, -size, 0], [-size, -size, 0]],
                                          dtype=np.float64)
                success, rotation, translation = cv2.solvePnP(
                    object_corners, pixel_corners, self.intrinsic,
                    self.distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if success and np.isfinite(translation).all() and translation[2, 0] > 0:
                    # IPPE supplies the planar initial solution. In OpenCV 4.5,
                    # its off-axis frontal result may retain several pixels of
                    # residual; refine against the four observed corners.
                    rotation, translation = cv2.solvePnPRefineLM(
                        object_corners, pixel_corners, self.intrinsic,
                        self.distortion, rotation, translation)
                    projected, _ = cv2.projectPoints(
                        object_corners, rotation, translation,
                        self.intrinsic, self.distortion)
                    error = float(np.sqrt(np.mean(np.sum(
                        (projected.reshape(4, 2) - pixel_corners) ** 2, axis=1))))
                    if error <= self.p('max_reprojection_error'):
                        found = True
                        self._measurement(message, rotation, translation, error)
                        cv2.drawFrameAxes(image, self.intrinsic, self.distortion,
                                          rotation, translation, size * 0.65)

        self._debug_image(image, message, found)

    def _measurement(self, message, rotation, translation, error):
        vector = tuple(float(value) for value in translation.flatten())
        body = optical_to_body(vector, self.p('camera_position'))
        # A short filter limits subpixel PnP noise without relying on odometry.
        if self.offset is None or self.last_tag_time is None or (
                stamp_seconds(message.header.stamp) - self.last_tag_time >
                self.p('target_timeout')):
            self.offset = body
            self.camera_height = vector[2]
        else:
            alpha = 0.50
            self.offset = tuple(alpha * value + (1 - alpha) * old
                                for value, old in zip(body, self.offset))
            self.camera_height = alpha * vector[2] + (1 - alpha) * self.camera_height
        self.last_tag_time = stamp_seconds(message.header.stamp)
        self.reprojection_error = error
        self.detections += 1
        matrix, _ = cv2.Rodrigues(rotation)
        quaternion = quaternion_from_matrix(matrix)
        pose = PoseStamped()
        pose.header = message.header
        pose.header.frame_id = self.p('camera_optical_frame')
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = vector
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = quaternion
        self.pose_publisher.publish(pose)

    def _debug_image(self, image, original, target_visible=False):
        steady_now = time.monotonic()
        rate = self.p('debug_image_rate')
        if (rate <= 0 or self.debug_publisher.get_subscription_count() == 0 or
                steady_now - self.last_debug_time < 1.0 / rate):
            return
        self.last_debug_time = steady_now
        # A smaller preview limits ROS serialization / GUI traffic. Detection and
        # metric pose estimation above still use the full calibrated image.
        image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_AREA)
        title = f"{self.state.upper()} | {self.p('tag_family')} ID {self.p('tag_id')}"
        details = 'Target not visible'
        if target_visible:
            error = math.hypot(*self.offset[:2])
            details = f'Height {self.camera_height:.2f} m | Offset {error:.2f} m'
        details += ' | Motors ' + ('ON' if self.motors_enabled else 'OFF')
        cv2.rectangle(image, (0, 0), (640, 52), (25, 25, 25), -1)
        for row, text in enumerate((title, details)):
            cv2.putText(image, text, (10, 20 + row * 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.52, (255, 255, 255), 1, cv2.LINE_AA)
        # gz-gui8 ImageDisplay supports RGB_INT8, but not BGR_INT8.
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        debug = Image()
        debug.header = original.header
        debug.height, debug.width = image.shape[:2]
        debug.encoding = 'rgb8'
        debug.is_bigendian = False
        debug.step = debug.width * 3
        debug.data = np.ascontiguousarray(image).tobytes()
        self.debug_publisher.publish(debug)

    def _contacts(self, message):
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = stamp_seconds(message.header.stamp)
        # This callback may run before the clock callback after an expensive
        # camera frame. A small apparent future stamp is normal callback order;
        # whole old-epoch timestamps are still rejected after reset. During the
        # paused arming interval all queued contacts are discarded.
        if self.replay.holding or stamp > now + 1.0:
            return
        observed = stamp if stamp > 0 else now
        contact = any(foot_station_contact(
            item.collision1.name, item.collision2.name, self.p('station_name'))
            for item in message.contacts)
        # Each foot has its own sensor and all four share this topic. Empty
        # messages from a different foot must not erase another foot's contact.
        # Positive contact expires in _control if no sensor refreshes it.
        if contact:
            if (self.last_contact_time is None or
                    observed - self.last_contact_time > 0.20):
                self.contact_since = observed
            self.last_contact_time = (observed if self.last_contact_time is None else
                                      max(observed, self.last_contact_time))
            self.has_contact = True
            if self.contact_since is None:
                self.contact_since = observed

    def _set_state(self, state):
        if state != self.state:
            self.get_logger().info(f'Landing: {self.state} -> {state}')
            self.state = state
        self.state_publisher.publish(String(data=state))

    def _control(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now + 0.1 < self.last_now:
            # A world reset invalidates both old camera observations and contact.
            self._clear_flight()
        self.last_now = now
        self.replay.tick()
        command = Twist()
        fresh = (self.last_tag_time is not None and self.offset is not None and
                 -0.10 <= now - self.last_tag_time <= self.p('target_timeout'))
        error = math.hypot(*self.offset[:2]) if self.offset is not None else None
        current_contact = (self.has_contact and self.last_contact_time is not None and
                           now - self.last_contact_time < 0.20)
        if not current_contact:
            self.contact_since = None
            self.has_contact = False

        if self.replay.holding:
            state = 'replay_error' if self.replay.error else 'replay_' + self.replay.phase
        elif self.landed:
            state = 'landed'
        elif not self.p('enabled'):
            state = 'disabled_hover'
            self.align_since = None
        elif not fresh:
            state = ('waiting_for_camera' if self.last_image_time is None else
                     'waiting_for_tag' if self.detections == 0 else 'target_lost_hover')
            self.align_since = None
        else:
            centered = error <= self.p('alignment_tolerance')
            if centered:
                if self.align_since is None:
                    self.align_since = now
            else:
                self.align_since = None
            aligned = (self.align_since is not None and
                       now - self.align_since >= self.p('alignment_hold'))
            vx, vy, vz = velocity_guidance(
                self.offset, self.camera_height, self.p('camera_clearance'),
                self.p('horizontal_gain'), self.p('horizontal_limit'),
                self.p('descend_rate'), self.p('flare_rate'),
                self.p('alignment_tolerance'), aligned)
            state = 'aligning'
            if aligned:
                state = ('flaring' if self.camera_height < self.p('camera_clearance') + 0.45
                         else 'descending')
            command.linear.x, command.linear.y, command.linear.z = vx, vy, vz
            if current_contact and centered:
                state = 'touchdown'
                # Keep a gentle down request during the confirmation interval;
                # hovering here could lift a foot off the moving deck.
                command.linear.z = -self.p('flare_rate')
                if self.contact_since is not None and (
                        now - self.contact_since >= self.p('contact_hold')):
                    self.landed = True
                    state = 'landed'
                    command = Twist()

        # Missing images/target always command a hover; no stale/blind descent.
        # Actual sustained station/gear contact is the only motor-off trigger.
        self.command_publisher.publish(command)
        self.motors_enabled = (self.replay.arm_motors if self.replay.holding
                               else not self.landed)
        self.enable_publisher.publish(Bool(data=self.motors_enabled))
        self._set_state(state)
        heartbeat_time = self.steady_clock.now().nanoseconds * 1e-9
        if heartbeat_time - self.last_status_time >= 0.2:
            payload = {
                'state': state, 'tag_family': self.p('tag_family'),
                'tag_id': self.p('tag_id'),
                'tag_size_m': self.p('tag_size'), 'detections': self.detections,
                'target_fresh': fresh, 'lateral_error_m': error,
                'image_age_s': (now - self.last_image_time
                                if self.last_image_time is not None else None),
                'tag_age_s': (now - self.last_tag_time
                              if self.last_tag_time is not None else None),
                'camera_height_m': self.camera_height,
                'reprojection_error_px': self.reprojection_error,
                'processing_ms': self.processing_ms,
                'processing_ms_ema': self.processing_ms_ema,
                'image_arrival_age_s': self.image_arrival_age_s,
                'image_callbacks': self.image_callbacks,
                'fallback_calls': self.fallback_calls,
                'foot_station_contact': current_contact,
                'motors_enabled': self.motors_enabled, 'sim_time': now,
                'replay_count': self.replay.replay_count,
                'replay_phase': self.replay.phase,
                'replay_error': self.replay.error,
            }
            self.status_publisher.publish(String(data=json.dumps(payload)))
            self.last_status_time = heartbeat_time


def main(args=None):
    rclpy.init(args=args)
    node = VisionLanding()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # A clean controller shutdown must not leave a persistent descent request.
        if rclpy.ok():
            node.command_publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
