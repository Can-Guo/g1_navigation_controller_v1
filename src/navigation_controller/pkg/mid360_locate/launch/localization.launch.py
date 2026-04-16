from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的共享目录
    pkg_share = FindPackageShare('mid360_locate')
    
    # 声明启动参数
    map_file_path_arg = DeclareLaunchArgument(
        'map_file_path',
        default_value='/home/york/ros2_ws_nav/src/navigation_controller/map/map.pcd',
        description='Path to the PCD map file'
    )
    
    points_type_arg = DeclareLaunchArgument(
        'points_type',
        default_value='0',
        description='0: livox custom message, 1: standard PointCloud2'
    )
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='/livox/lidar',
        description='Topic name for point cloud input'
    )
    
    map_roll_arg = DeclareLaunchArgument(
        'map_roll',
        default_value='3.14',
        description='Map roll angle'
    )
    
    map_pitch_arg = DeclareLaunchArgument(
        'map_pitch',
        default_value='0.0',
        description='Map pitch angle'
    )
    
    map_yaw_arg = DeclareLaunchArgument(
        'map_yaw',
        default_value='-0.05',
        description='Map yaw angle'
    )
    
    # 定位节点
    locate_node = Node(
        package='mid360_locate',
        executable='locate_node',
        name='locate_node',
        output='screen',
        parameters=[{
            'map_file_path': LaunchConfiguration('map_file_path'),
            'points_type': LaunchConfiguration('points_type'),
            'topic_name': LaunchConfiguration('topic_name'),
            'map_roll': LaunchConfiguration('map_roll'),
            'map_pitch': LaunchConfiguration('map_pitch'),
            'map_yaw': LaunchConfiguration('map_yaw'),
        }]
    )
    
    return LaunchDescription([
        map_file_path_arg,
        points_type_arg,
        topic_name_arg,
        map_roll_arg,
        map_pitch_arg,
        map_yaw_arg,
        locate_node,
    ])
