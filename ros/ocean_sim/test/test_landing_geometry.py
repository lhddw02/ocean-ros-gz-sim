"""Geometry/regression checks runnable without ROS or a display."""

import math
import unittest

from ocean_sim.landing_geometry import (
    foot_station_contact, optical_to_body, quaternion_from_matrix,
    velocity_guidance,
)


class LandingGeometryTest(unittest.TestCase):
    def test_camera_right_requires_body_right(self):
        # A tag to the right in the camera lies at negative body Y.
        self.assertEqual(optical_to_body((0.4, -0.2, 3.0)), (0.2, -0.4, -3.1))

    def test_centering_sign_and_vector_speed_limit(self):
        vx, vy, vz = velocity_guidance((2.0, -3.0, -4.0), 4.0)
        self.assertGreater(vx, 0)
        self.assertLess(vy, 0)
        self.assertLessEqual(math.hypot(vx, vy), 0.55 + 1e-9)
        self.assertEqual(vz, 0.0)

    def test_no_descent_without_alignment_or_when_outside_gate(self):
        self.assertEqual(velocity_guidance((0, 0, -3), 3)[2], 0)
        self.assertEqual(velocity_guidance((0.3, 0, -3), 3,
                                          allow_descent=True)[2], 0)

    def test_flare_remains_contact_seeking_and_bounded(self):
        far = velocity_guidance((0.02, 0, -3), 3, allow_descent=True)[2]
        close = velocity_guidance((0.02, 0, -0.5), 0.36,
                                  allow_descent=True)[2]
        self.assertEqual(far, -0.35)
        self.assertEqual(close, -0.10)

    def test_only_foot_and_station_contacts_authorize_shutdown(self):
        self.assertTrue(foot_station_contact('floating_station::deck',
                                            'drone::base_link::foot_fl'))
        self.assertTrue(foot_station_contact('drone::base_link::foot_rr',
                                            'floating_station::deck'))
        self.assertFalse(foot_station_contact('ground_plane', 'drone::foot_fl'))
        self.assertFalse(foot_station_contact('floating_station::deck',
                                             'drone::propeller'))

    def test_planar_tag_180_degree_quaternion_is_finite(self):
        q = quaternion_from_matrix(((1, 0, 0), (0, -1, 0), (0, 0, -1)))
        self.assertAlmostEqual(abs(q[0]), 1)
        self.assertAlmostEqual(sum(value * value for value in q), 1)


if __name__ == '__main__':
    unittest.main()
