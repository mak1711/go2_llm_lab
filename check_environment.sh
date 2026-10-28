#!/usr/bin/env bash
# Checks prerequisites; does not reconfigure the student's ROS environment.
set -u
fail=0
for name in ros2 colcon python3; do
  if command -v "$name" >/dev/null 2>&1; then echo "OK command: $name";
  else echo "MISSING command: $name"; fail=1; fi
done
if [ "$fail" -eq 1 ]; then exit 1; fi
for pkg in nav2_bringup nav2_msgs rviz2 unitree_api unitree_go; do
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then echo "OK ROS package: $pkg";
  else echo "MISSING ROS package: $pkg"; fail=1; fi
done
python3 - <<'PYCHK'
import importlib
for name in ('mcp', 'httpx', 'rclpy', 'unitree_api.msg', 'unitree_go.msg'):
    try:
        importlib.import_module(name)
        print('OK Python import:', name)
    except Exception as exc:
        print('MISSING Python import:', name, '(', exc, ')')
        raise SystemExit(1)
PYCHK
if [ $? -ne 0 ]; then fail=1; fi
if [ "$fail" -eq 0 ]; then echo 'Environment checks passed.'; else echo 'Resolve missing prerequisites before starting the lab.'; fi
exit "$fail"
