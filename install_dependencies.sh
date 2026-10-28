#!/usr/bin/env bash
set -euo pipefail
sudo apt-get update
sudo apt-get install -y \
    python3-pip python3-colcon-common-extensions python3-yaml \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-rviz2 \
    ros-humble-tf2-ros ros-humble-nav2-msgs ros-humble-action-msgs \
    ros-humble-geometry-msgs
python3 -m pip install --user -r "$(dirname "$0")/requirements.txt"
echo 'Additional lab dependencies installed. Unitree ROS 2 messages must be supplied by your previous lab setup.'
