#!/usr/bin/env python3
"""Bridge Nav2's /cmd_vel to the Go2 sport API (/api/sport/request).
Only sends Move (api 1008) and StopMove (api 1003). The robot must already be
standing in normal sport mode (use the remote). Safety:
  * every command is clamped to max_vx / max_vy / max_wz,
  * if /cmd_vel goes quiet for `timeout` seconds, StopMove is sent once,
  * Ctrl+C sends StopMove before exiting,
  * dry_run:=true logs what would be sent and publishes nothing.
Run (after `source ~/unitree_ros2/setup.sh` and the workspace):
    ros2 run go2_llm_lab cmd_vel_bridge --ros-args -p dry_run:=true
    ros2 run go2_llm_lab cmd_vel_bridge
"""
import json
import signal
import time
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from unitree_api.msg import Request
API_ID_STOPMOVE = 1003
API_ID_MOVE = 1008

def clamp(value, limit):
    return max(-limit, min(limit, value))

class CmdVelToSport(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_sport')
        self.max_vx = self.declare_parameter('max_vx', 0.3).value
        self.max_vy = self.declare_parameter('max_vy', 0.2).value
        self.max_wz = self.declare_parameter('max_wz', 0.6).value
        self.timeout = self.declare_parameter('timeout', 0.5).value
        self.deadband = self.declare_parameter('deadband', 0.01).value
        self.dry_run = self.declare_parameter('dry_run', False).value
        rate = self.declare_parameter('rate', 20.0).value
        cmd_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        self.pub = None if self.dry_run else self.create_publisher(
            Request, '/api/sport/request', 10)
        self.create_subscription(Twist, cmd_topic, self.on_cmd, 10)
        self.create_timer(1.0 / rate, self.tick)
        self.last_cmd = None
        self.last_cmd_time = 0.0
        self.moving = False
        self.get_logger().info(
            f"{'DRY RUN - publishing nothing' if self.dry_run else 'LIVE - driving the robot'}: "
            f"{cmd_topic} -> /api/sport/request, limits vx={self.max_vx} vy={self.max_vy} "
            f"wz={self.max_wz}, timeout={self.timeout}s")

    def on_cmd(self, msg):
        self.last_cmd = (clamp(msg.linear.x, self.max_vx),
                         clamp(msg.linear.y, self.max_vy),
                         clamp(msg.angular.z, self.max_wz))
        self.last_cmd_time = time.monotonic()

    def send(self, api_id, parameter=''):
        req = Request()
        req.header.identity.id = time.monotonic_ns()
        req.header.identity.api_id = api_id
        req.parameter = parameter
        if self.pub is not None:
            self.pub.publish(req)

    def stop(self, reason):
        self.send(API_ID_STOPMOVE)
        self.moving = False
        self.get_logger().info(f'StopMove ({reason})')

    def tick(self):
        fresh = (self.last_cmd is not None
                 and time.monotonic() - self.last_cmd_time < self.timeout)
        if not fresh:
            if self.moving:
                self.stop('no /cmd_vel for %.1fs' % self.timeout)
            return
        vx, vy, wz = self.last_cmd
        if max(abs(vx), abs(vy), abs(wz)) < self.deadband:
            if self.moving:
                self.stop('zero command')
            return
        self.send(API_ID_MOVE, json.dumps({'x': vx, 'y': vy, 'z': wz}))
        if not self.moving:
            self.get_logger().info('Moving')
        self.moving = True
        self.get_logger().info(f'Move vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f}',
                               throttle_duration_sec=1.0)

def main():
    # Handle Ctrl+C / SIGTERM ourselves so StopMove is published before rclpy
    # shuts the context down (rclpy's own handler invalidates the publisher).
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = CmdVelToSport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            node.send(API_ID_STOPMOVE)
            time.sleep(0.05)
        node.get_logger().info('StopMove (shutdown)')
        node.destroy_node()
        rclpy.try_shutdown()
    return 0

if __name__ == '__main__':
    main()
