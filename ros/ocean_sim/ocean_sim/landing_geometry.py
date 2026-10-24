"""Small, ROS-independent pieces of camera geometry and landing guidance.

Gazebo camera sensors look along +X, whereas OpenCV optical coordinates are
right/down/forward. The camera in this project is pitched down by pi / 2.
Consequently optical (x, y, z) becomes body (-y, -x, -z).
"""

import math


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def optical_to_body(vector, camera_position=(0.0, 0.0, -0.10)):
    """Transform an optical-frame position to the drone's FLU body frame."""
    x, y, z = vector
    return (-y + camera_position[0], -x + camera_position[1],
            -z + camera_position[2])


def quaternion_from_matrix(matrix):
    """Return ROS (x, y, z, w), including the 180-degree planar-tag case."""
    m = matrix
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        result = ((m[2][1] - m[1][2]) / s,
                  (m[0][2] - m[2][0]) / s,
                  (m[1][0] - m[0][1]) / s, 0.25 * s)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        result = (0.25 * s, (m[0][1] + m[1][0]) / s,
                  (m[0][2] + m[2][0]) / s,
                  (m[2][1] - m[1][2]) / s)
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        result = ((m[0][1] + m[1][0]) / s, 0.25 * s,
                  (m[1][2] + m[2][1]) / s,
                  (m[0][2] - m[2][0]) / s)
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        result = ((m[0][2] + m[2][0]) / s,
                  (m[1][2] + m[2][1]) / s, 0.25 * s,
                  (m[1][0] - m[0][1]) / s)
    norm = math.sqrt(sum(value * value for value in result))
    return tuple(value / norm for value in result)


def foot_station_contact(first, second, station_name='floating_station'):
    """Only landing-gear / station contact may authorize motor shutdown."""
    first, second = first.lower(), second.lower()
    station_name = station_name.lower()
    return ((station_name in first and 'foot_' in second) or
            (station_name in second and 'foot_' in first))


def velocity_guidance(body_offset, camera_height, camera_clearance=0.35,
                      horizontal_gain=0.85, horizontal_limit=0.55,
                      descend_rate=0.35, flare_rate=0.10,
                      alignment_tolerance=0.12, allow_descent=False):
    """Vision-only proportional centering and gated, bounded descent.

    Height is the measured optical-axis distance to the tag. Clearance is
    the camera-to-deck distance with feet touching. Descent slows near that
    geometric clearance but never substitutes for a physical contact test.
    """
    x, y = body_offset[:2]
    error = math.hypot(x, y)
    scale = min(1.0, horizontal_limit / max(horizontal_gain * error, 1e-9))
    vx, vy = horizontal_gain * x * scale, horizontal_gain * y * scale
    vz = 0.0
    if allow_descent and error <= alignment_tolerance:
        gap = max(0.0, camera_height - camera_clearance)
        # Retain a gentle downward request until real feet/deck contact arrives.
        vz = -min(descend_rate, max(flare_rate, 0.55 * gap))
    return vx, vy, vz
