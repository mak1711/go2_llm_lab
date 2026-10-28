"""Nav2 for the Go2 (map from RTAB-Map), RViz, and the /cmd_vel -> sport API bridge.
    ros2 launch go2_llm_lab nav2.launch.py                  # plan only: the robot never moves
    ros2 launch go2_llm_lab nav2.launch.py bridge:=true     # the robot follows Nav2
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share = get_package_share_directory('go2_llm_lab')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    # 1. Nav2 itself: planner, controller, behaviors, BT navigator, costmaps, velocity smoother.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'false',      # real robot: use the real clock
            'autostart': 'true',          # configure and activate all Nav2 nodes automatically
        }.items(),
    )
    # 2. RViz with a layout that shows the map, costmaps, plan and robot footprint.
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(share, 'rviz', 'nav2.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    # 3. The bridge: Nav2 outputs /cmd_vel, the Go2 only understands sport API requests.
    bridge = Node(
        package='go2_llm_lab', executable='cmd_vel_bridge', name='cmd_vel_to_sport', output='screen',
        condition=IfCondition(LaunchConfiguration('bridge')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('bridge', default_value='false',
                              description='true = the robot moves (sends sport API commands)'),
        nav2,
        rviz,
        bridge,
    ])
