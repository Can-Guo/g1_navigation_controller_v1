from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'navigation_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='szm88',
    maintainer_email='1347893477@qq.com',
    description='Navigation system for Unitree G1 robot with Livox MID360 LiDAR - ROS2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Core navigation nodes (ROS2)
            'astar_planner = navigation_controller.nodes.astar_planner_ros2:main',
            'dwa_planner = navigation_controller.nodes.dwa_planner_ros2_mid360:main',
            'g1_position_control = navigation_controller.nodes.g1_position_control_ros2:main',
            'point_to_point_action_server = navigation_controller.nodes.point_to_point_action_server:main',
            
            # Test and demo nodes
            'test_point_to_point_action = navigation_controller.nodes.test_point_to_point_action:main',
            # 'demo_multi_point_patrol = navigation_controller.nodes.demo_multi_point_patrol:main',  # 文件缺失,已注释
            'nav_goal_pub_key_demo = navigation_controller.nodes.nav_goal_pub_key_demo2:main',
            'test_point_to_point_action_keyboard = scripts.test_point_to_point_action_keyboard:main',
            
            # Monitoring and debugging utilities (文件缺失,已注释)
            # 'localization_monitor = navigation_controller.nodes.localization_monitor:main',
            # 'monitor_ekf_fusion = navigation_controller.nodes.monitor_ekf_fusion:main',
            # 'tf_tree_monitor = navigation_controller.nodes.tf_tree_monitor:main',
            
            # Legacy ROS1 nodes (for reference/migration)
            'astar_planner_ros1 = navigation_controller.nodes.astar_planner:main',
            'g1_position_control_ros1 = navigation_controller.nodes.g1_position_control:main',
            'dwa_planner_ros1 = navigation_controller.nodes.dwa_planner_ros1_mid360:main',
            
            # Utility tools
            'pcd2pgm = navigation_controller.nodes.pcd2pgm:main',
        ],
    },
)
