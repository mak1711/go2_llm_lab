"""Send a goal to Nav2 (the NavigateToPose action) and check on it.
Nav2 must already be running. Coordinates are in the map frame
(GO2_NAV_FRAME, default "map", the frame RTAB-Map publishes).
Test it on its own:
    ros2 run go2_llm_lab send_goal 1.0 0.5          # x y  (metres)
    ros2 run go2_llm_lab send_goal 1.0 0.5 90       # x y yaw (degrees)
Ctrl+C cancels the goal.
"""
import math
import os
import sys
import time
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from go2_llm_lab.ros_utils import spin_in_background, wait_for
STATUS_NAMES = {
    GoalStatus.STATUS_ACCEPTED: "accepted",
    GoalStatus.STATUS_EXECUTING: "navigating",
    GoalStatus.STATUS_CANCELING: "canceling",
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}

class Nav2Goal(Node):
    """Needs a spinning executor (see ros_utils.spin_in_background)."""

    def __init__(self):
        super().__init__("go2_nav_goal")
        self.frame = os.environ.get("GO2_NAV_FRAME", "map")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.target = None

    def busy(self):
        return self.result_future is not None and not self.result_future.done()

    def send(self, x, y, yaw_deg=0.0):
        """Send a goal and wait only until Nav2 accepts it (the robot then drives on its own)."""
        if self.busy():
            raise RuntimeError("Already navigating. Cancel the current goal first.")
        if not self.client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError("Nav2 is not running (no navigate_to_pose action server).")
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        yaw = math.radians(yaw_deg)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.feedback = None
        future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        if not wait_for(future, 5.0) or not future.result().accepted:
            raise RuntimeError("Nav2 did not accept the goal.")
        self.goal_handle = future.result()
        self.result_future = self.goal_handle.get_result_async()
        self.target = (x, y, yaw_deg)
        return f"Goal accepted: x={x:.2f} y={y:.2f} yaw={yaw_deg:.0f} deg in frame '{self.frame}'."

    def _on_feedback(self, msg):
        self.feedback = msg.feedback

    def status(self):
        """Where the current goal stands: idle, navigating, succeeded, aborted or canceled."""
        if self.result_future is None:
            return {"state": "idle"}
        info = {"target": self.target}
        if self.result_future.done():
            info["state"] = STATUS_NAMES.get(self.result_future.result().status, "unknown")
            return info
        info["state"] = "navigating"
        if self.feedback is not None:
            info["distance_remaining_m"] = round(float(self.feedback.distance_remaining), 2)
        return info

    def cancel(self):
        if not self.busy():
            return "No goal is running."
        wait_for(self.goal_handle.cancel_goal_async(), 3.0)
        return "Goal canceled."

def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if len(args) < 2:
        print("Usage: ros2 run go2_llm_lab send_goal X Y [YAW_DEG]")
        return 1
    x, y = float(args[0]), float(args[1])
    yaw = float(args[2]) if len(args) > 2 else 0.0
    rclpy.init()
    nav = Nav2Goal()
    spin_in_background(nav)
    try:
        print(nav.send(x, y, yaw))
        while nav.busy():
            print(f"  {nav.status()}", end="\r", flush=True)
            time.sleep(0.5)
        print(f"\nResult: {nav.status()['state']}")
    except RuntimeError as err:
        print(err)
        return 1
    except KeyboardInterrupt:
        print("\n" + nav.cancel())
    finally:
        nav.destroy_node()
        rclpy.shutdown()
    return 0

if __name__ == "__main__":
    sys.exit(main())
