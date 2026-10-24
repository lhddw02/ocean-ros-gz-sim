#!/usr/bin/env python3
"""Verify real Gazebo -> ROS data and a physical buoy response, with a deadline."""
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped


def main():
    rclpy.init()
    node = rclpy.create_node("ocean_smoke_probe")
    state = {"clocks": [], "heights": []}

    def clock_callback(message):
        state["clocks"].append(message.clock.sec + message.clock.nanosec * 1e-9)

    def pose_callback(message):
        clocks = state["clocks"]
        if clocks and clocks[-1] - clocks[0] >= 3:
            state["heights"].append(message.pose.position.z)

    subscriptions = [
        node.create_subscription(Clock, "/clock", clock_callback, qos_profile_sensor_data),
        node.create_subscription(PoseStamped, "/ocean/buoy/pose", pose_callback, qos_profile_sensor_data),
    ]
    deadline = time.monotonic() + 90
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
            clocks, heights = state["clocks"], state["heights"]
            if heights and abs(heights[-1]) > 2:
                raise RuntimeError("The buoy left the sea surface; buoyancy may have failed.")
            if (len(clocks) >= 2 and clocks[-1] - clocks[0] >= 8
                    and len(heights) >= 10 and max(heights) - min(heights) > 0.005):
                print(f"Clock advanced {clocks[-1] - clocks[0]:.2f} s; "
                      f"buoy height range {max(heights) - min(heights):.3f} m")
                return
        raise RuntimeError(
            f"Timed out: clock samples={len(state['clocks'])}, "
            f"buoy pose samples={len(state['heights'])}. "
            "Inspect the Gazebo plugin and bridge log.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
