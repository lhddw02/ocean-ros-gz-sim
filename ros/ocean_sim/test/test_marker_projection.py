"""Exercise the real AprilTag dictionary and PnP convention without ROS/Gazebo."""

import unittest

try:
    import cv2
    import numpy as np
    HAS_OPENCV = hasattr(cv2, 'aruco')
except ImportError:
    HAS_OPENCV = False


@unittest.skipUnless(HAS_OPENCV, 'Run in the image with OpenCV-contrib installed')
class MarkerProjectionTest(unittest.TestCase):
    def test_detected_marker_has_correct_metric_depth_and_offset(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        image = np.full((720, 960), 255, dtype=np.uint8)
        marker = cv2.aruco.drawMarker(dictionary, 7, 160)
        image[280:440, 440:600] = marker
        corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary)
        self.assertEqual(ids.flatten().tolist(), [7])
        half = 0.2
        points = np.array([[-half, half, 0], [half, half, 0],
                           [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
        intrinsic = np.array([[400, 0, 480], [0, 400, 360], [0, 0, 1]],
                             dtype=np.float64)
        success, rotation, translation = cv2.solvePnP(
            points, corners[0].reshape(4, 2).astype(np.float64), intrinsic,
            np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE)
        self.assertTrue(success)
        rotation, translation = cv2.solvePnPRefineLM(
            points, corners[0].reshape(4, 2).astype(np.float64), intrinsic,
            np.zeros(5), rotation, translation)
        self.assertTrue(np.isfinite(translation).all())
        self.assertAlmostEqual(translation[2, 0], 1.0, delta=0.015)
        self.assertAlmostEqual(translation[0, 0], 0.10, delta=0.005)
        self.assertAlmostEqual(translation[1, 0], 0.0, delta=0.005)


if __name__ == '__main__':
    unittest.main()
