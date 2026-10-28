from glob import glob
import os
from setuptools import find_packages, setup
package_name = 'go2_llm_lab'
setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),
         glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Educational Go2 LLM, MCP, sport API and Nav2 control lab',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'walk_test = go2_llm_lab.robot_moves:main',
            'mcp_server = go2_llm_lab.mcp_server:main',
            'llm_agent = go2_llm_lab.agent:main',
            'cmd_vel_bridge = go2_llm_lab.cmd_vel_bridge:main',
            'send_goal = go2_llm_lab.nav_goal:main',
        ],
    },
)
