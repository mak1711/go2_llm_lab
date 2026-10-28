"""Small ROS 2 helpers shared by the lab code."""
import threading
import time
from rclpy.executors import MultiThreadedExecutor

def spin_in_background(*nodes):
    """Spin the given nodes in a background thread.
    Subscriptions and action callbacks then run on their own, and any other
    thread (for example an MCP tool) can publish or send goals at any time.
    """
    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return executor

def wait_for(future, timeout_s):
    """Wait until a ROS future is done (needs a spinning executor). Returns True if done."""
    end = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < end:
        time.sleep(0.05)
    return future.done()
