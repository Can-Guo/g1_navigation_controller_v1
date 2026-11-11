#!/usr/bin/env python3
"""
DWA (Dynamic Window Approach) Planner for ROS2 Humble
Migrated from ROS1 Noetic dwa_planner_ros1_mid360.py

This node implements local obstacle avoidance using point cloud data
and the DWA algorithm for trajectory planning.

Key Features:
- Point cloud processing and obstacle detectionBEST_EFFORT
- Clustering algorithm for obstacle identification  
- Dynamic Window Approach for local planning
- TF2 coordinate transformation
- Smooth velocity control with gradual acceleration/deceleration
- Visualization of obstacles and candidate trajectories

ROS2 Migration Changes:
- rospy → rclpy with Node class inheritance
- rospy.Time.now() → self.get_clock().now()
- rospy.Duration() → rclpy.duration.Duration()
- rospy.Timer() → self.create_timer() without event parameter
- TF: tf.TransformListener() → tf2_ros.Buffer() + TransformListener(node)
- Logging: rospy.loginfo() → self.get_logger().info()
- Parameters: rospy.get_param() → self.declare_parameter() + get_parameter()
- QoS profiles for sensor data (BEST_EFFORT) and control (RELIABLE)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

import math
import random
import numpy as np
from collections import defaultdict

# ROS2 message imports
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, Point, PoseStamped
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray

# TF2 imports for ROS2
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PointStamped

# Transform utilities
# ROS2 Migration Fix: Use scipy instead of tf_transformations
# tf_transformations is not available by default in ROS2
from scipy.spatial.transform import Rotation

def euler_from_quaternion(quaternion):
    """
    Convert quaternion to euler angles (roll, pitch, yaw)
    Args:
        quaternion: list/tuple [x, y, z, w]
    Returns:
        tuple: (roll, pitch, yaw) in radians
    """
    r = Rotation.from_quat(quaternion)
    return tuple(r.as_euler('xyz', degrees=False))

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert euler angles to quaternion
    Args:
        roll, pitch, yaw: angles in radians
    Returns:
        list: [x, y, z, w]
    """
    r = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=False)
    return r.as_quat().tolist()


class DWAPlanner(Node):
    """DWA local planner with obstacle avoidance"""
    
    def __init__(self):
        super().__init__('dwa_planner')
        
        self.get_logger().info("Initializing DWA Planner for ROS2 Humble...")
        
        # Callback groups for concurrent execution
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.subscription_group = ReentrantCallbackGroup()
        
        # === Parameters Declaration (ROS2 pattern) ===
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('min_speed', 0.12)
        self.declare_parameter('max_turn', 0.12)  # 🔧 降低角速度避免定位丢失 (0.5 → 0.12)
        self.declare_parameter('detection_range', 1.2)
        self.declare_parameter('time_horizon', 1.0)
        self.declare_parameter('min_obstacle_points', 40)
        self.declare_parameter('clustering_distance', 0.3)
        self.declare_parameter('smooth_factor', 0.2)
        self.declare_parameter('waypoint_distance', 0.3)
        self.declare_parameter('debug', False)
        self.declare_parameter('debug_interval', 2.0)
        self.declare_parameter('loop_rate', 10.0)
        # 目标坐标系（默认与定位发布一致：map）
        self.declare_parameter('target_frame', 'map')
        
        # Get parameters
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_turn = self.get_parameter('max_turn').value
        self.detection_range = self.get_parameter('detection_range').value
        self.time_horizon = self.get_parameter('time_horizon').value
        self.min_obstacle_points = self.get_parameter('min_obstacle_points').value
        self.clustering_distance = self.get_parameter('clustering_distance').value
        self.smooth_factor = self.get_parameter('smooth_factor').value
        self.waypoint_distance = self.get_parameter('waypoint_distance').value
        self.debug = self.get_parameter('debug').value
        debug_interval_sec = self.get_parameter('debug_interval').value
        loop_rate = self.get_parameter('loop_rate').value
        self.target_frame = self.get_parameter('target_frame').value
        
        # === QoS Profiles ===
        # For sensor data (point cloud) - BEST_EFFORT for low latency
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, 
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # For control commands and path - RELIABLE for guaranteed delivery
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # === TF2 Setup (ROS2 pattern) ===
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # === Subscribers ===
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/aligned',
            self.pointcloud_callback,
            sensor_qos,
            callback_group=self.subscription_group
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            # 与 mid360_locate 发布保持一致，使用 /odometry/map
            '/odometry/map',
            self.odom_callback,
            reliable_qos,
            callback_group=self.subscription_group
        )       
        
        self.path_sub = self.create_subscription(
            Path,
            '/astar_path',
            self.path_callback,
            reliable_qos,
            callback_group=self.subscription_group
        )
        
        # === Publishers ===
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/dwa_cmd_vel',
            reliable_qos
        )
        
        self.path_publisher = self.create_publisher(
            Marker,
            '/dwa_path_visualization',
            reliable_qos
        )
        
        self.obstacle_pub = self.create_publisher(
            MarkerArray,
            '/detected_obstacles',
            reliable_qos
        )
        
        self.filtered_points_pub = self.create_publisher(
            Marker,
            '/filtered_points',
            reliable_qos
        )
        
        self.obstacle_status_pub = self.create_publisher(
            Int32,
            '/obstacle_status',
            reliable_qos
        )
        
        # === State Variables ===
        self.pointcloud_data = None
        self.odom_data = None
        self.waypoints = []
        self.has_path = False
        self.goal_reached = False
        self.current_waypoint_index = 0
        self.goal_x = None
        self.goal_y = None
        
        self.obstacles = []
        self.obstacle_avoidance_active = False
        self.preferred_turn_direction = None
        
        self.last_cmd_vel = {'linear': 0.0, 'angular': 0.0}
        
        # ====================================================================
        # G1人形机器人改进：逃逸行为检测
        # 检测机器人是否"卡住"并自动触发逃逸机制
        # ====================================================================
        self.stuck_counter = 0
        self.last_position = None
        self.escape_mode = False
        self.escape_phase = 0  # 0: 后退, 1: 旋转, 2: 前进尝试
        self.escape_timer = 0
        
        # G1特定：位置历史用于检测步态异常
        self.position_history = []
        
        # Debug timing (ROS2 pattern)
        self.debug_interval = Duration(seconds=debug_interval_sec)
        self.last_debug_time = self.get_clock().now()
        
        # === Main Control Timer (ROS2 pattern - no event parameter) ===
        timer_period = 1.0 / loop_rate  # 10 Hz default
        self.control_timer = self.create_timer(
            timer_period,
            self.movement_loop,
            callback_group=self.timer_group
        )
        
        self.get_logger().info(f"DWA Planner initialized with:")
        self.get_logger().info(f"  max_speed={self.max_speed}, min_speed={self.min_speed}")
        self.get_logger().info(f"  detection_range={self.detection_range}, clustering_distance={self.clustering_distance}")
        self.get_logger().info(f"  loop_rate={loop_rate} Hz")
    
    # === Callback Functions ===
    
    def pointcloud_callback(self, msg):
        """Receive point cloud data"""
        self.pointcloud_data = msg
        self.process_pointcloud()
    
    def odom_callback(self, msg):
        """Receive odometry data"""
        self.odom_data = msg
    
    def path_callback(self, msg):
        """Receive path from A* planner"""
        if msg.poses:
            self.waypoints = msg.poses
            self.has_path = True
            self.goal_reached = False
            self.current_waypoint_index = 0
            
            # Store goal position
            if len(self.waypoints) > 0:
                self.goal_x = self.waypoints[-1].pose.position.x
                self.goal_y = self.waypoints[-1].pose.position.y
            
            self.get_logger().info(f"Received new path with {len(self.waypoints)} waypoints")
        else:
            self.waypoints = []
            self.has_path = False
            self.goal_x = None
            self.goal_y = None
    
    def publish_obstacle_status(self, status):
        """Publish obstacle detection status (0: clear, 1: obstacle detected)"""
        msg = Int32()
        msg.data = status
        self.obstacle_status_pub.publish(msg)
    
    # === Point Cloud Processing ===
    
    def process_pointcloud(self):
        """Process point cloud and detect obstacles"""
        if self.pointcloud_data is None or self.odom_data is None:
            return
        
        # Read point cloud data
        points_list = []
        for point in point_cloud2.read_points(self.pointcloud_data, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        
        if not points_list:
            self.obstacles = []
            return
        
        # Get robot pose
        robot_x = self.odom_data.pose.pose.position.x
        robot_y = self.odom_data.pose.pose.position.y
        orient = self.odom_data.pose.pose.orientation
        _, _, robot_yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        # Transform points to target frame (default: map)
        try:
            # ROS2: lookup_transform requires Time object
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.pointcloud_data.header.frame_id,
                rclpy.time.Time(),  # Use latest available transform
                timeout=Duration(seconds=0.1)
            )
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform point cloud: {ex}", throttle_duration_sec=2.0)
            return
        
        transformed_points = []
        for point in points_list:
            # Create PointStamped for transformation
            point_stamped = PointStamped()
            point_stamped.header.frame_id = self.pointcloud_data.header.frame_id
            point_stamped.header.stamp = self.pointcloud_data.header.stamp
            # 🔧 紧急修复: 将 numpy 类型转换为 Python float
            # ROS2 消息要求严格的 Python float 类型
            point_stamped.point.x = float(point[0])
            point_stamped.point.y = float(point[1])
            point_stamped.point.z = float(point[2])
            
            # Transform point
            try:
                transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
                transformed_points.append([
                    transformed_point.point.x,
                    transformed_point.point.y,
                    transformed_point.point.z
                ])
            except Exception as e:
                continue
        
        # Filter points based on distance, angle, and height
        filtered_points = []
        for point in transformed_points:
            px, py, pz = point
            
            # Calculate distance and angle relative to robot
            dx = px - robot_x
            dy = py - robot_y
            distance = math.hypot(dx, dy)
            angle = math.atan2(dy, dx) - robot_yaw
            angle = self.normalize_angle(angle)
            
            # Filter criteria
            if (distance < self.detection_range and 
                abs(angle) < math.pi / 2 and  # ±90 degrees
                pz > -0.3 and pz < 0.6):  # Height threshold
                filtered_points.append([px, py, pz])
        
        # Publish filtered points visualization
        if filtered_points:
            self.publish_filtered_points(filtered_points)
        
        # Cluster filtered points to identify obstacles
        if len(filtered_points) >= self.min_obstacle_points:
            clustered_obstacles = self.cluster_points(filtered_points)
            self.obstacles = clustered_obstacles
            
            # Publish obstacle visualization
            if clustered_obstacles:
                self.publish_obstacles_visualization(clustered_obstacles)
        else:
            self.obstacles = []
    
    def cluster_points(self, points):
        """Cluster nearby points to identify separate obstacles (DBSCAN-like algorithm)"""
        if not points:
            return []
        
        clusters = []
        visited = [False] * len(points)
        
        for i in range(len(points)):
            if visited[i]:
                continue
            
            # Start a new cluster
            cluster = [points[i]]
            visited[i] = True
            queue = [i]
            
            while queue:
                current_idx = queue.pop(0)
                current_point = points[current_idx]
                
                # Find neighbors within clustering distance
                for j in range(len(points)):
                    if visited[j]:
                        continue
                    
                    neighbor_point = points[j]
                    distance = math.hypot(
                        current_point[0] - neighbor_point[0],
                        current_point[1] - neighbor_point[1]
                    )
                    
                    if distance < self.clustering_distance:
                        cluster.append(neighbor_point)
                        visited[j] = True
                        queue.append(j)
            
            # Only keep clusters with enough points
            if len(cluster) >= self.min_obstacle_points:
                # Calculate cluster centroid
                avg_x = sum(p[0] for p in cluster) / len(cluster)
                avg_y = sum(p[1] for p in cluster) / len(cluster)
                avg_z = sum(p[2] for p in cluster) / len(cluster)
                
                clusters.append({
                    'x': avg_x,
                    'y': avg_y,
                    'z': avg_z,
                    'points': len(cluster)
                })
        
        return clusters
    
    def publish_obstacles_visualization(self, obstacles):
        """Publish obstacle markers for RViz visualization"""
        marker_array = MarkerArray()
        
        for i, obs in enumerate(obstacles):
            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = obs['x']
            marker.pose.position.y = obs['y']
            marker.pose.position.z = obs['z']
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        self.obstacle_pub.publish(marker_array)
    
    # === Obstacle Detection ===
    
    def detect_stuck(self):
        """
        检测G1人形机器人是否卡住
        
        对于G1机器人，"卡住"的判定需要考虑：
        1. 位置几乎不变（< 10cm in 5秒）
        2. 持续有控制命令输出
        3. 不是处于正常停止状态
        
        Returns:
            bool: True表示机器人卡住
        """
        if self.odom_data is None or not self.has_path:
            return False
        
        current_pos = np.array([
            self.odom_data.pose.pose.position.x,
            self.odom_data.pose.pose.position.y
        ])
        
        # 添加到位置历史
        self.position_history.append(current_pos)
        if len(self.position_history) > 50:  # 保持5秒历史 @ 10Hz
            self.position_history.pop(0)
        
        if len(self.position_history) < 50:
            return False  # 数据不足
        
        # 计算5秒内的总位移
        total_displacement = 0.0
        for i in range(1, len(self.position_history)):
            displacement = np.linalg.norm(
                self.position_history[i] - self.position_history[i-1]
            )
            total_displacement += displacement
        
        # G1人形机器人特定阈值：
        # - 正常步行速度约0.3-1.0 m/s
        # - 5秒应至少移动1.5米（保守估计）
        # 🔧 绕圈修复: 降低卡住阈值，避免将"原地转向"误判为卡住
        STUCK_THRESHOLD = 0.15  # 从0.5降至0.15米，更严格的判定
        
        # 检查是否有线速度命令但位移很小（排除纯旋转情况）
        has_linear_command = abs(self.last_cmd_vel['linear']) > 0.05
        angular_velocity = abs(self.odom_data.twist.twist.angular.z) if self.odom_data else 0.0
        is_rotating_in_place = angular_velocity > 0.1 and abs(self.last_cmd_vel['linear']) < 0.05
        
        # 🔧 关键修复: 只有在有前进命令且不在原地转向时才判定为卡住
        is_stuck = (total_displacement < STUCK_THRESHOLD and 
                   has_linear_command and 
                   not is_rotating_in_place)
        
        if is_stuck:
            self.stuck_counter += 1
            if self.stuck_counter > 10:  # 连续1秒检测到卡住
                return True
        else:
            self.stuck_counter = 0
        
        return False
    
    def analyze_obstacle_distribution(self):
        """
        DWA改进: 分析当前障碍物分布,智能选择逃逸方向
        
        将周围360度空间分为8个扇区,计算每个扇区的:
        - 障碍物数量
        - 平均距离
        - 综合评分
        
        Returns:
            best_escape_direction: float, 最佳逃逸方向(弧度)
            should_reverse: bool, 是否需要后退
        """
        if not self.obstacles or self.odom_data is None:
            return None, False
        
        robot_x = self.odom_data.pose.pose.position.x
        robot_y = self.odom_data.pose.pose.position.y
        orient = self.odom_data.pose.pose.orientation
        _, _, robot_yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        # 将障碍物分配到8个扇区 (每个45度)
        sector_counts = [0] * 8
        sector_distances = [[] for _ in range(8)]
        
        for obs in self.obstacles:
            dx = obs['x'] - robot_x
            dy = obs['y'] - robot_y
            distance = math.hypot(dx, dy)
            
            if distance < 2.0:  # 只考虑2米内障碍物
                # 计算相对角度
                angle_to_obs = math.atan2(dy, dx)
                relative_angle = self.normalize_angle(angle_to_obs - robot_yaw)
                
                # 确定扇区 (0=前方, 2=左侧, 4=后方, 6=右侧)
                sector_idx = int((relative_angle + math.pi) / (math.pi / 4)) % 8
                sector_counts[sector_idx] += 1
                sector_distances[sector_idx].append(distance)
        
        # 查找最空旷的扇区
        sector_scores = []
        for i in range(8):
            if sector_counts[i] == 0:
                avg_distance = 2.0  # 无障碍物,满分
            else:
                avg_distance = sum(sector_distances[i]) / sector_counts[i]
            
            # 评分 = 平均距离 / (障碍物数量 + 1)
            score = avg_distance / (sector_counts[i] + 1)
            sector_scores.append(score)
        
        # 找到最佳扇区
        best_sector = sector_scores.index(max(sector_scores))
        
        # 计算逃逸方向
        best_escape_angle = (best_sector * math.pi / 4) - math.pi
        best_escape_direction = self.normalize_angle(robot_yaw + best_escape_angle)
        
        # 判断是否需要后退 (前方扇区0,1,7障碍物密集)
        front_obstacle_count = sector_counts[0] + sector_counts[1] + sector_counts[7]
        should_reverse = front_obstacle_count > 5
        
        self.get_logger().info(
            f"障碍物分析: 扇区计数={sector_counts}, "
            f"最佳扇区={best_sector}, 是否后退={should_reverse}"
        )
        
        return best_escape_direction, should_reverse
    
    def generate_escape_paths(self):
        """
        DWA改进: 为G1人形机器人生成智能逃逸轨迹
        
        使用障碍物分布分析智能决策:
        - 分析8个方向的障碍物密度
        - 选择最空旷方向逃逸
        - 根据前方障碍密度决定是否后退
        
        Returns:
            list: 候选逃逸轨迹列表
        """
        possible_paths = []
        
        # DWA改进: 分析障碍物分布,智能选择逃逸方向
        escape_direction, should_reverse = self.analyze_obstacle_distribution()
        
        if escape_direction is None:
            # 无障碍物信息,使用默认逃逸(回退到正常路径生成)
            return self.generate_paths()
        
        # 当前朝向
        orient = self.odom_data.pose.pose.orientation
        _, _, current_yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        # 计算需要转向的角度
        turn_angle = self.normalize_angle(escape_direction - current_yaw)
        
        # 根据阶段生成逃逸轨迹
        if self.escape_phase == 0 and should_reverse:
            # ================================================================
            # 阶段0: 后退 (仅当前方有密集障碍时)
            # ================================================================
            speeds = [-0.2, -0.15, -0.1]  # 后退速度
            turns = [-0.2, 0.0, 0.2]      # 微调方向
            
            self.get_logger().info("智能逃逸 - 阶段0: 后退避障")
            
        elif should_reverse and self.escape_phase == 1:
            # ================================================================
            # 阶段1: 转向最佳方向
            # ================================================================
            turn_rate = 0.5 if turn_angle > 0 else -0.5
            speeds = [0.0, 0.0]
            turns = [turn_rate, turn_rate * 0.8]
            
            self.get_logger().info(f"智能逃逸 - 阶段1: 转向最佳方向 (目标角度={math.degrees(turn_angle):.1f}°)")
            
        else:
            # ================================================================
            # 阶段2 (或跳过后退时): 朝最佳方向前进
            # ================================================================
            speeds = [0.2, 0.25, 0.3]
            # 逐渐对准最佳方向
            turn_rate = max(-0.4, min(0.4, turn_angle * 2.0))
            turns = [turn_rate, turn_rate * 0.5, 0.0]
            
            self.get_logger().info(f"智能逃逸 - 阶段2: 朝最佳方向前进 (方向={math.degrees(escape_direction):.1f}°)")
        
        # 生成路径
        for speed in speeds:
            for turn in turns:
                path = self.predict_motion(speed, turn)
                possible_paths.append((speed, turn, path))
        
        return possible_paths
    
    def update_escape_phase(self):
        """
        更新逃逸阶段
        
        每个阶段持续一定时间后自动切换到下一阶段
        """
        self.escape_timer += 1
        
        # G1人形机器人的逃逸阶段持续时间（以控制周期计，10Hz）
        PHASE_DURATIONS = {
            0: 20,  # 后退阶段：2秒
            1: 30,  # 旋转阶段：3秒
            2: 20   # 前进阶段：2秒
        }
        
        if self.escape_timer >= PHASE_DURATIONS.get(self.escape_phase, 20):
            self.escape_timer = 0
            self.escape_phase += 1
            
            if self.escape_phase > 2:
                # 完成所有逃逸阶段，退出逃逸模式
                self.get_logger().info("逃逸序列完成，恢复正常导航")
                self.escape_mode = False
                self.escape_phase = 0
                return True  # 表示逃逸完成
        
        return False  # 表示仍在逃逸中
    
    def check_obstacles(self):
        """Check if obstacles are present in the detection range"""
        if not self.obstacles or self.odom_data is None:
            return False
        
        robot_x = self.odom_data.pose.pose.position.x
        robot_y = self.odom_data.pose.pose.position.y
        orient = self.odom_data.pose.pose.orientation
        _, _, robot_yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        obstacle_detected = False
        left_obstacle = False
        right_obstacle = False
        
        for obs in self.obstacles:
            dx = obs['x'] - robot_x
            dy = obs['y'] - robot_y
            distance = math.hypot(dx, dy)
            
            if distance < self.detection_range:
                obstacle_detected = True
                
                # Determine obstacle side relative to robot heading
                angle_to_obstacle = math.atan2(dy, dx)
                relative_angle = self.normalize_angle(angle_to_obstacle - robot_yaw)
                
                if relative_angle > 0:
                    left_obstacle = True
                else:
                    right_obstacle = True
        
        # Set preferred turn direction to avoid obstacles
        if obstacle_detected:
            if left_obstacle and not right_obstacle:
                self.preferred_turn_direction = -1  # Turn right
            elif right_obstacle and not left_obstacle:
                self.preferred_turn_direction = 1  # Turn left
            else:
                # Both sides have obstacles, keep previous direction or choose randomly
                if self.preferred_turn_direction is None:
                    self.preferred_turn_direction = random.choice([-1, 1])
        
        return obstacle_detected
    
    # === DWA Algorithm ===
    
    def predict_motion(self, speed, turn_rate):
        """Predict robot trajectory given speed and turn rate"""
        dt = 0.1  # Time step
        path = []
        
        x = self.odom_data.pose.pose.position.x
        y = self.odom_data.pose.pose.position.y
        orient = self.odom_data.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        for _ in range(int(self.time_horizon / dt)):
            x += speed * math.cos(yaw) * dt
            y += speed * math.sin(yaw) * dt
            yaw += turn_rate * dt
            path.append((x, y))
        
        return path
    
    def check_for_collisions(self, path):
        """
        DWA改进: 改进的碰撞检测 - 连续惩罚函数 + 障碍物大小考虑
        
        相比原版改进:
        1. 考虑障碍物大小(点云聚类半径估计)
        2. 使用连续惩罚函数替代阶梯函数,避免评分突变
        3. 危险区(<0.4m)使用指数惩罚,强力避障
        4. 警告区(0.4-1.0m)平滑过渡
        """
        if not self.obstacles:
            return 0.0
        
        collision_risk = 0.0
        min_clearance = float('inf')
        
        # G1机器人安全半径 (机器人宽度 + 安全边距)
        ROBOT_SAFETY_RADIUS = 0.4  # 米
        
        for x, y in path:
            for obs in self.obstacles:
                # 考虑障碍物大小(点云聚类的半径估计)
                obs_radius = 0.15  # 保守估计每个障碍物半径
                
                distance = math.hypot(x - obs['x'], y - obs['y'])
                effective_distance = distance - obs_radius  # 实际间隙
                min_clearance = min(min_clearance, effective_distance)
                
                # 使用连续惩罚函数替代阶梯函数
                # 公式: risk = -k / (d + epsilon)^2
                # d越小,惩罚越大;连续平滑无突变
                
                if effective_distance < ROBOT_SAFETY_RADIUS:
                    # 危险区域:指数级惩罚
                    collision_risk -= 100 * math.exp(-effective_distance / 0.2)
                elif effective_distance < 1.0:
                    # 警告区域:反比例惩罚
                    collision_risk -= 30.0 / (effective_distance + 0.1)
        
        return collision_risk
    
    def choose_best_path(self, possible_paths):
        """Evaluate all candidate paths and select the best one"""
        if not self.odom_data or not self.has_path:
            return self.min_speed, 0.0
        
        best_score = -float('inf')
        best_speed = self.min_speed
        best_turn = 0.0
        
        # Get current state
        robot_x = self.odom_data.pose.pose.position.x
        robot_y = self.odom_data.pose.pose.position.y
        orient = self.odom_data.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        current_speed = self.last_cmd_vel['linear']
        current_turn = self.last_cmd_vel['angular']
        
        # Goal position (last waypoint)
        if self.goal_x is not None and self.goal_y is not None:
            goal_x = self.goal_x
            goal_y = self.goal_y
        else:
            return self.min_speed, 0.0
        
        # Evaluate each candidate path
        for speed, turn, path in possible_paths:
            if not path:
                continue
            
            # Distance to goal from end of path
            end_x, end_y = path[-1]
            goal_distance = math.hypot(goal_x - end_x, goal_y - end_y)
            # 🔧 绕圈修复: 大幅提升目标吸引力，让机器人积极朝目标前进
            goal_distance_score = -goal_distance * 15  # 从8提升到15，增强目标导向
            
            # Heading alignment with goal
            angle_to_goal = math.atan2(goal_y - robot_y, goal_x - robot_x)
            heading_diff = abs(self.normalize_angle(angle_to_goal - yaw))
            # 🔧 绕圈修复: 提升方向对准权重
            heading_score = -heading_diff * 10  # 从6提升到10，强化方向对准
            
            # Collision risk
            # DWA改进: 双倍碰撞惩罚,确保安全优先
            collision_risk = self.check_for_collisions(path) * 2.0  # 新增×2.0
            
            # Smoothness score (penalize large changes from current velocity)
            speed_change = abs(speed - current_speed)
            turn_change = abs(turn - current_turn)
            # 🔧 绕圈修复: 降低平滑权重，允许快速响应
            smoothness_score = -3.0 * speed_change - 5.0 * turn_change  # 恢复原始值，避免过度平滑
            
            # Speed reward (encourage higher speeds)
            # 🔧 绕圈修复: 提升速度奖励，鼓励快速直线前进
            speed_reward = speed * 3.0  # 从1.5提升到3.0，鼓励高速运动
            
            # Direction bonus (if preferred turn direction exists)
            direction_bonus = 0
            if self.preferred_turn_direction is not None:
                if (self.preferred_turn_direction > 0 and turn > 0) or \
                   (self.preferred_turn_direction < 0 and turn < 0):
                    direction_match = 1.0 - abs(abs(turn) - 0.3) / 0.3
                    # DWA改进: 减少方向强制从15降至8
                    direction_bonus = 8 * direction_match  # 原15
            
            # Total score
            total_score = (goal_distance_score + heading_score + collision_risk +
                          smoothness_score + speed_reward + direction_bonus)
            
            if self.debug:
                self.get_logger().info(
                    f"Path score: speed={speed:.2f}, turn={turn:.2f}, total={total_score:.2f}, "
                    f"goal_dist={goal_distance_score:.2f}, heading={heading_score:.2f}, "
                    f"collision={collision_risk:.2f}, smoothness={smoothness_score:.2f}, "
                    f"speed_reward={speed_reward:.2f}, direction_bonus={direction_bonus:.2f}",
                    throttle_duration_sec=2.0
                )
            
            # Update best path
            if total_score > best_score:
                best_score = total_score
                best_speed, best_turn = speed, turn
        
        # Ensure minimum speed
        if best_speed < self.min_speed:
            best_speed = self.min_speed
        
        current_time = self.get_clock().now()
        if self.debug and (current_time - self.last_debug_time) > self.debug_interval:
            self.get_logger().info(f"Best path selected: vx={best_speed:.2f}, vyaw={best_turn:.2f}, score={best_score:.2f}")
            self.last_debug_time = current_time
        
        return best_speed, best_turn
    
    def generate_paths(self):
        """Generate candidate trajectories considering preferred turn direction"""
        possible_paths = []
        
        if self.preferred_turn_direction is not None:
            # 60% paths in preferred direction
            min_speed = max(self.min_speed, 0.12)
            
            for i in range(60):
                speed = min_speed + (self.max_speed - min_speed) * (i / 60.0)
                if self.preferred_turn_direction > 0:  # Left turn preferred
                    turn = random.uniform(0.05, self.max_turn)
                else:  # Right turn preferred
                    turn = random.uniform(-self.max_turn, -0.05)
                path = self.predict_motion(speed, turn)
                possible_paths.append((speed, turn, path))
            
            # 30% straight paths
            for i in range(30):
                speed = min_speed + (self.max_speed - min_speed) * (i / 30.0)
                turn = random.uniform(-0.05, 0.05)
                path = self.predict_motion(speed, turn)
                possible_paths.append((speed, turn, path))
            
            # 10% random paths
            for i in range(10):
                speed = random.uniform(min_speed, self.max_speed)
                turn = random.uniform(-self.max_turn, self.max_turn)
                path = self.predict_motion(speed, turn)
                possible_paths.append((speed, turn, path))
        else:
            # No preferred direction - generate uniform distribution
            min_speed = max(self.min_speed, 0.12)
            
            for i in range(100):
                speed = min_speed + (self.max_speed - min_speed) * (i / 100.0)
                turn = random.uniform(-self.max_turn, self.max_turn)
                path = self.predict_motion(speed, turn)
                possible_paths.append((speed, turn, path))
        
        return possible_paths
    
    def find_closest_waypoint(self):
        """Find closest safe waypoint considering obstacle distances"""
        if not self.has_path or not self.odom_data:
            return None
        
        robot_x = self.odom_data.pose.pose.position.x
        robot_y = self.odom_data.pose.pose.position.y
        
        min_dist = float('inf')
        closest_idx = self.current_waypoint_index
        found_safe_waypoint = False
        min_safe_distance = 1.0  # 1 meter safety margin
        
        # Search from current index forward
        for i in range(self.current_waypoint_index, len(self.waypoints)):
            waypoint = self.waypoints[i]
            dist = math.hypot(
                waypoint.pose.position.x - robot_x,
                waypoint.pose.position.y - robot_y
            )
            
            # Check if waypoint is safe (>1m from all obstacles)
            is_safe = True
            if self.obstacles:
                for obstacle in self.obstacles:
                    obstacle_dist = math.hypot(
                        waypoint.pose.position.x - obstacle['x'],
                        waypoint.pose.position.y - obstacle['y']
                    )
                    if obstacle_dist < min_safe_distance:
                        is_safe = False
                        break
            
            if is_safe:
                found_safe_waypoint = True
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i
        
        # If no safe waypoint found, use last waypoint
        if self.obstacles and not found_safe_waypoint:
            self.get_logger().info("No safe waypoint found, switching to last waypoint")
            return self.waypoints[-1]
        
        # If closest is current, try to advance
        if closest_idx == self.current_waypoint_index and closest_idx + 3 < len(self.waypoints):
            for i in range(closest_idx + 3, len(self.waypoints)):
                waypoint = self.waypoints[i]
                is_safe = True
                
                if self.obstacles:
                    for obstacle in self.obstacles:
                        obstacle_dist = math.hypot(
                            waypoint.pose.position.x - obstacle['x'],
                            waypoint.pose.position.y - obstacle['y']
                        )
                        if obstacle_dist < min_safe_distance:
                            is_safe = False
                            break
                
                if is_safe:
                    self.get_logger().info(f"Using forward safe waypoint: index {i}, {i - closest_idx} points ahead")
                    return self.waypoints[i]
            
            self.get_logger().info("No forward safe waypoint, switching to last waypoint")
            return self.waypoints[-1]
        
        return self.waypoints[closest_idx]
    
    # === Main Control Loop ===
    
    def movement_loop(self):
        """Main control loop for DWA planner (ROS2: no event parameter)"""
        if self.odom_data is None:
            self.get_logger().warn("Waiting for odometry data...", throttle_duration_sec=1.0)
            return
        
        if self.pointcloud_data is None:
            self.get_logger().warn("Waiting for point cloud data...", throttle_duration_sec=1.0)
            return
        
        # ====================================================================
        # G1改进：检测机器人是否卡住
        # ====================================================================
        if self.detect_stuck() and not self.escape_mode:
            self.get_logger().warn("检测到G1机器人卡住！启动逃逸行为...")
            self.escape_mode = True
            self.escape_phase = 0
            self.escape_timer = 0
        
        # ====================================================================
        # 逃逸模式处理（优先级最高）
        # ====================================================================
        if self.escape_mode:
            # 生成逃逸轨迹
            escape_paths = self.generate_escape_paths()
            
            # 选择最佳逃逸路径
            speed, turn = self.choose_best_path(escape_paths)
            
            # 应用速度平滑
            smooth_speed = self.last_cmd_vel['linear'] + self.smooth_factor * (speed - self.last_cmd_vel['linear'])
            smooth_turn = self.last_cmd_vel['angular'] + self.smooth_factor * (turn - self.last_cmd_vel['angular'])
            
            # 发布逃逸控制命令
            move_cmd = Twist()
            move_cmd.linear.x = smooth_speed
            move_cmd.angular.z = smooth_turn
            self.cmd_publisher.publish(move_cmd)
            
            # 更新速度记录
            self.last_cmd_vel['linear'] = smooth_speed
            self.last_cmd_vel['angular'] = smooth_turn
            
            # 更新逃逸阶段
            escape_complete = self.update_escape_phase()
            
            if escape_complete:
                self.get_logger().info("逃逸完成，清空位置历史")
                self.position_history.clear()
                self.stuck_counter = 0
            
            self.get_logger().info(
                f"[逃逸模式] 阶段={self.escape_phase}, 速度={smooth_speed:.2f}, 转向={smooth_turn:.2f}",
                throttle_duration_sec=0.5
            )
            
            return
        
        # ====================================================================
        # 正常导航流程（非逃逸模式）
        # ====================================================================
        
        # Check if path is valid
        if not self.has_path or self.goal_reached:
            # Clear obstacle avoidance state if active
            if self.obstacle_avoidance_active:
                self.get_logger().info("Exiting DWA obstacle avoidance mode")
                self.obstacle_avoidance_active = False
                self.preferred_turn_direction = None
                self.publish_obstacle_status(0)
                
                # Send stop command
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.angular.z = 0.0
                self.cmd_publisher.publish(stop_cmd)
                
                # Reset velocity record
                self.last_cmd_vel['linear'] = 0.0
                self.last_cmd_vel['angular'] = 0.0
            
            return
        
        # Check if path is completed
        if self.current_waypoint_index >= len(self.waypoints):
            if not self.goal_reached:
                self.get_logger().info("DWA path tracking completed")
                self.goal_reached = True
                
                # Send stop command
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.angular.z = 0.0
                self.cmd_publisher.publish(stop_cmd)
                
                # Reset velocity record
                self.last_cmd_vel['linear'] = 0.0
                self.last_cmd_vel['angular'] = 0.0
                
                # Clear obstacle avoidance state
                if self.obstacle_avoidance_active:
                    self.obstacle_avoidance_active = False
                    self.preferred_turn_direction = None
                    self.publish_obstacle_status(0)
            
            return
        
        # Check for obstacles
        obstacle_present = self.check_obstacles()
        
        # Generate candidate paths
        possible_paths = self.generate_paths()
        
        # Control mode based on obstacle presence
        if obstacle_present or self.obstacle_avoidance_active:
            # Obstacle avoidance mode
            if not self.obstacle_avoidance_active:
                self.get_logger().info("Activating DWA obstacle avoidance mode")
                self.obstacle_avoidance_active = True
                self.publish_obstacle_status(1)
            
            # Choose best path using DWA
            speed, turn = self.choose_best_path(possible_paths)
            
            # Ensure minimum speed
            if speed < self.min_speed:
                speed = self.min_speed
            
            # Reduce speed for sharp turns
            if abs(turn) > 0.4:
                speed = max(self.min_speed, speed * (1.0 - (abs(turn) - 0.4) / 0.6))
            
            # Smooth velocity changes
            smooth_speed = self.last_cmd_vel['linear'] + self.smooth_factor * (speed - self.last_cmd_vel['linear'])
            smooth_turn = self.last_cmd_vel['angular'] + self.smooth_factor * (turn - self.last_cmd_vel['angular'])
            
            # Publish control command
            move_cmd = Twist()
            move_cmd.linear.x = smooth_speed
            move_cmd.angular.z = smooth_turn
            
            # Debug logging
            current_x = self.odom_data.pose.pose.position.x
            current_y = self.odom_data.pose.pose.position.y
            orient = self.odom_data.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
            
            if self.goal_x is not None and self.goal_y is not None:
                dx = self.goal_x - current_x
                dy = self.goal_y - current_y
                distance = math.hypot(dx, dy)
                theta = math.atan2(dy, dx)
                angle_diff = abs(self.normalize_angle(theta - yaw))
                self.get_logger().info(
                    f"[DWA CMD] vx={smooth_speed:.2f}, vyaw={smooth_turn:.2f}, "
                    f"angle_diff={angle_diff:.2f}, distance={distance:.2f}",
                    throttle_duration_sec=1.0
                )
            
            self.cmd_publisher.publish(move_cmd)
            
            # Update velocity record
            self.last_cmd_vel['linear'] = smooth_speed
            self.last_cmd_vel['angular'] = smooth_turn
            
            current_time = self.get_clock().now()
            if self.debug and (current_time - self.last_debug_time) > self.debug_interval:
                direction_str = 'Right' if self.preferred_turn_direction > 0 else 'Left' if self.preferred_turn_direction < 0 else 'None'
                self.get_logger().info(f"DWA control: vx={smooth_speed:.2f}, vyaw={smooth_turn:.2f}, direction={direction_str}")
                self.last_debug_time = current_time
            
            # Publish trajectory visualization
            self.publish_path_visualization(possible_paths)
        
        else:
            # No obstacles - transition back to A* path following
            if self.obstacle_avoidance_active:
                self.get_logger().info("Obstacles cleared")
                self.get_logger().info("Exiting DWA obstacle avoidance, resuming A* path tracking")
                self.obstacle_avoidance_active = False
                self.preferred_turn_direction = None
                self.publish_obstacle_status(0)
                
                # Find closest waypoint to resume path
                if self.has_path and len(self.waypoints) > 0:
                    closest_waypoint = self.find_closest_waypoint()
                    if closest_waypoint:
                        new_index = self.waypoints.index(closest_waypoint)
                        old_index = self.current_waypoint_index
                        if new_index > self.current_waypoint_index:
                            self.get_logger().info(
                                f"Repositioned to waypoint {new_index} "
                                f"(previous: {old_index}, total: {len(self.waypoints)})"
                            )
                            self.current_waypoint_index = new_index
            
            # Gradual deceleration instead of abrupt stop
            if self.last_cmd_vel['linear'] > 0.01 or abs(self.last_cmd_vel['angular']) > 0.01:
                new_linear = self.last_cmd_vel['linear'] * 0.5
                new_angular = self.last_cmd_vel['angular'] * 0.5
                
                if new_linear < 0.01:
                    new_linear = 0.0
                if abs(new_angular) < 0.01:
                    new_angular = 0.0
                
                move_cmd = Twist()
                move_cmd.linear.x = new_linear
                move_cmd.angular.z = new_angular
                self.cmd_publisher.publish(move_cmd)
                
                self.last_cmd_vel['linear'] = new_linear
                self.last_cmd_vel['angular'] = new_angular
            else:
                # Complete stop
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.angular.z = 0.0
                self.cmd_publisher.publish(stop_cmd)
                
                self.last_cmd_vel['linear'] = 0.0
                self.last_cmd_vel['angular'] = 0.0
    
    # === Visualization ===
    
    def publish_path_visualization(self, possible_paths):
        """Publish trajectory visualization markers"""
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.color.r = 1.0
        marker.color.a = 1.0
        
        for _, _, path in possible_paths:
            for x, y in path:
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.0
                marker.points.append(point)
        
        self.path_publisher.publish(marker)
    
    def publish_filtered_points(self, points):
        """Publish filtered point cloud visualization"""
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        for x, y, _ in points:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            marker.points.append(point)
        
        self.filtered_points_pub.publish(marker)
    
    # === Utility Functions ===
    
    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        planner = DWAPlanner()
        
        # Use MultiThreadedExecutor for concurrent callback execution
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(planner)
        
        try:
            planner.get_logger().info("DWA Planner running... Press Ctrl+C to stop")
            executor.spin()
        except KeyboardInterrupt:
            planner.get_logger().info("DWA Planner stopped by user")
        finally:
            executor.shutdown()
            planner.destroy_node()
    
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
