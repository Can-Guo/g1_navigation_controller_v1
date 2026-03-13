#!/usr/bin/env python3
"""
G1 Manhattan Position Control Node for ROS2 Humble
Author: shenlanxueyuan (Modified for Manhattan Motion)
Email: 1347893477@qq.com
Organization: Shen Lan xue yuan
Date: 2026-03-05

🔧 MANHATTAN MOTION CONTROLLER
================================
This specialized controller implements Manhattan-style motion for the G1 robot:

MOTION CONSTRAINTS:
------------------
1. **Single-Axis Movement**: Robot can only move along ONE axis at a time
   - X-axis mode: vx ≠ 0, vy = 0 (Forward/Backward)
   - Y-axis mode: vx = 0, vy ≠ 0 (Left/Right Strafe)
   - No diagonal movement (vx and vy cannot both be non-zero)

2. **Orientation Control**: Robot maintains heading toward goal while moving
   - Continuous orientation adjustment using angular velocity (vyaw)
   - Decoupled from linear motion (non-holonomic-like behavior)

3. **State Machine**: Manages axis switching with stability control
   - Greedy strategy: Always move along axis with larger error
   - Hysteresis to prevent rapid oscillation
   - Smooth transitions between axes

PROFESSIONAL FEATURES:
---------------------
- Manhattan distance heuristic for path evaluation
- Axis-aware waypoint tolerance (separate thresholds for X/Y)
- Direction switching penalty to encourage straight segments
- Orientation-aware control (faces target during movement)
- Compatible with Manhattan A* planner output

USE CASES:
----------
- Structured environments (hallways, warehouses)
- Testing path planning algorithms (Manhattan vs Euclidean)  
- Compliance with grid-based navigation constraints
- Studying non-holonomic robot behavior

Migration Notes (ROS1 → ROS2):
- Changed from rospy to rclpy
- Updated to use Node class (object-oriented)
- Timer callback signature changed (no event parameter)
- Parameter API changed (declare_parameter required)
- Time API changed (get_clock())
- Logging changed (get_logger())

Hardware Integration:
- Integrated Unitree SDK for G1 robot direct control
- Automatic mode detection (simulation vs real hardware)
- Network interface configuration for SDK communication
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration as RCLDuration
from rclpy.time import Time

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist, Point
from std_msgs.msg import Bool
import math
import numpy as np
import time
import os
import sys
from enum import Enum
from collections import deque
import datetime

# 🔧 紧急修复: 动态添加 unitree_sdk2py 路径到 sys.path
unitree_sdk_path_added = False
try:
    from ament_index_python.packages import get_package_prefix
    try:
        unitree_sdk_prefix = get_package_prefix('unitree_sdk2py')
        unitree_sdk_path = os.path.join(unitree_sdk_prefix, 'lib', 'python3.10', 'site-packages')
        if os.path.exists(unitree_sdk_path):
            if unitree_sdk_path not in sys.path:
                sys.path.insert(0, unitree_sdk_path)
            unitree_sdk_path_added = True
            print(f"[SDK路径] 方法1成功: {unitree_sdk_path}", file=sys.stderr)
    except Exception as e1:
        print(f"[SDK路径] 方法1失败: {e1}", file=sys.stderr)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        workspace_root = os.path.realpath(os.path.join(script_dir, '..', '..', '..', '..'))
        unitree_sdk_path = os.path.join(workspace_root, 'install', 'unitree_sdk2py', 'lib', 'python3.10', 'site-packages')
        if os.path.exists(unitree_sdk_path):
            if unitree_sdk_path not in sys.path:
                sys.path.insert(0, unitree_sdk_path)
            unitree_sdk_path_added = True
            print(f"[SDK路径] 方法2成功: {unitree_sdk_path}", file=sys.stderr)
        else:
            print(f"[SDK路径] 方法2失败: 路径不存在 {unitree_sdk_path}", file=sys.stderr)
except Exception as e:
    print(f"[SDK路径] 致命错误: {e}", file=sys.stderr)

# Unitree SDK 导入 (可选,仿真模式不需要)
UNITREE_SDK_AVAILABLE = False
LocoClient = None
ChannelFactoryInitialize = None

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    UNITREE_SDK_AVAILABLE = True
    print(f"[SDK导入] ✅ unitree_sdk2py 导入成功", file=sys.stderr)
except ImportError as e:
    print(f"[SDK导入] ❌ 导入失败: {e}", file=sys.stderr)
    print(f"[SDK导入] sys.path: {[p for p in sys.path if 'unitree' in p]}", file=sys.stderr)


class MovementAxis(Enum):
    """Movement axis enumeration for Manhattan motion"""
    X_AXIS = "x"  # Forward/Backward movement
    Y_AXIS = "y"  # Left/Right strafing
    STOPPED = "stop"  # No movement


class ManhattanState(Enum):
    """State machine for Manhattan motion controller"""
    IDLE = "idle"
    ORIENTING = "orienting"  # Rotating to face target
    MOVING_X = "moving_x"    # Moving along X-axis
    MOVING_Y = "moving_y"    # Moving along Y-axis
    REACHED = "reached"      # Goal reached


class G1ManhattanPositionControl(Node):
    """
    Manhattan Motion Controller for G1 Robot
    
    Implements single-axis movement with professional SLAM considerations:
    - Manhattan distance metrics
    - Greedy axis selection with hysteresis
    - Orientation control toward goal
    - State machine for robust switching
    """
    
    def __init__(self):
        # 初始化ROS2节点
        super().__init__('g1_manhattan_position_control')
        
        # =====================================================================
        # MANHATTAN MOTION PARAMETERS
        # =====================================================================
        
        # 声明参数
        self.declare_parameter('waypoint_distance', 0.5)
        self.declare_parameter('goal_tolerance', 0.15)  # 单轴运动可以更精确
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 0.45)
        self.declare_parameter('use_hardware', False)
        
        # Manhattan-specific parameters
        self.declare_parameter('axis_switch_hysteresis', 0.10)  # 防止抖动的滞环
        self.declare_parameter('axis_tolerance', 0.08)  # 轴对齐容差
        self.declare_parameter('orientation_tolerance', 0.15)  # 朝向容差 (rad, ~8.5度)
        self.declare_parameter('min_move_speed', 0.08)  # 最小移动速度
        
        # 读取参数
        self.waypoint_distance = self.get_parameter('waypoint_distance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.use_hardware = self.get_parameter('use_hardware').value
        
        # Manhattan特定参数
        self.axis_switch_hysteresis = self.get_parameter('axis_switch_hysteresis').value
        self.axis_tolerance = self.get_parameter('axis_tolerance').value
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').value
        self.min_move_speed = self.get_parameter('min_move_speed').value
        
        # =====================================================================
        # STATE MACHINE VARIABLES
        # =====================================================================
        
        self.state = ManhattanState.IDLE
        self.current_axis = MovementAxis.STOPPED
        self.last_axis_switch_time = self.get_clock().now()
        self.axis_switch_delay = 0.3  # 轴切换最小间隔 (秒)
        
        # Axis error tracking for hysteresis
        self.error_x = 0.0
        self.error_y = 0.0
        self.prev_axis = MovementAxis.STOPPED
        
        # Movement history for state transitions
        self.movement_history = deque(maxlen=10)  # 记录最近10次的运动状态
        
        # =====================================================================
        # STANDARD CONTROLLER VARIABLES (from original)
        # =====================================================================
        
        self.current_pose = None
        self.goal_pose = None
        self.has_goal = False
        self.has_path = False
        self.waypoints = []
        self.current_waypoint_index = 0
        self.last_waypoint_index = -1
        self.count = 0
        
        # 障碍物和避障相关
        self.has_obstacle = False
        self.obstacle_avoidance_active = False
        self.dwa_cmd = None
        self.last_dwa_cmd_time = None
        
        # 超时检测
        self.target_start_time = None
        self.target_distance = None
        self.timeout_triggered = False
        self.last_robot_position = None
        self.position_history = []
        
        # 硬件控制相关
        self.hardware_mode = False
        self.loco_client = None
        self.last_hardware_cmd_time = 0
        self.network_interface = "ens37"  # G1默认网口
        
        # 导航激活状态
        self.navigation_active = False
        self.last_active_time = self.get_clock().now()
        
        # 初始化cmd_vel日志文件
        _script_dir = os.path.dirname(os.path.abspath(__file__))
        _log_dir = os.path.join(_script_dir, 'logs', 'cmd_logs')
        os.makedirs(_log_dir, exist_ok=True)
        _log_name = datetime.datetime.now().strftime('cmd_vel_%Y%m%d-%H%M%S.log')
        self._cmd_log_file = open(os.path.join(_log_dir, _log_name), 'w', buffering=1)
        self._cmd_log_file.write('datetime_ms,vx,vy,vyaw\n')
        
        # 初始化硬件控制 (如果需要)
        if self.use_hardware:
            self._init_hardware_control()
        
        # =====================================================================
        # ROS2 COMMUNICATION
        # =====================================================================
        
        # QoS配置
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/map',
            self.odom_callback,
            best_effort_qos
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.goal_callback,
            reliable_qos
        )
        
        self.path_sub = self.create_subscription(
            Path,
            '/astar_path',
            self.path_callback,
            reliable_qos
        )
        
        self.dwa_cmd_sub = self.create_subscription(
            Twist,
            '/dwa_cmd_vel',
            self.dwa_cmd_callback,
            reliable_qos
        )
        
        self.obstacle_status_sub = self.create_subscription(
            Point,
            '/obstacle_status',
            self.obstacle_status_callback,
            reliable_qos
        )
        
        self.nav_active_sub = self.create_subscription(
            Bool,
            '/navigation_active',
            self.navigation_active_callback,
            reliable_qos
        )
        
        # 发布者
        self.vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            reliable_qos
        )
        
        # 定时器
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        mode_str = "🤖 硬件模式" if self.hardware_mode else "🎮 仿真模式"
        self.get_logger().info(
            f"🔲 Manhattan导航控制节点已启动 [{mode_str}]\n"
            f"   运动约束: 单轴移动 (X或Y，不能同时)\n"
            f"   轴切换滞环: {self.axis_switch_hysteresis:.3f}m\n"
            f"   轴对齐容差: {self.axis_tolerance:.3f}m\n"
            f"   朝向容差: {self.orientation_tolerance:.2f} rad"
        )
    
    # =========================================================================
    # HARDWARE CONTROL (unchanged from original)
    # =========================================================================
    
    def _init_hardware_control(self):
        """初始化Unitree SDK硬件控制 (与原版相同)"""
        if not UNITREE_SDK_AVAILABLE:
            self.get_logger().error("❌ Unitree SDK 未安装,无法使用硬件模式")
            self.get_logger().warn("   降级到仿真模式,将发布 /cmd_vel 消息")
            self.hardware_mode = False
            return
        
        try:
            self.get_logger().info("🔧 初始化 Unitree SDK...")
            
            try:
                ChannelFactoryInitialize(0, self.network_interface)
                self.get_logger().info(f"✓ DDS通道初始化成功 (接口: {self.network_interface})")
            except Exception as e:
                self.get_logger().warn(f"接口 {self.network_interface} 初始化失败: {e}")
                self.get_logger().info("尝试使用默认接口...")
                ChannelFactoryInitialize(0, "")
                self.get_logger().info("✓ 使用默认接口初始化成功")
            
            self.loco_client = LocoClient()
            self.loco_client.SetTimeout(10.0)
            self.loco_client.Init()
            self.get_logger().info("✓ G1 运动控制客户端创建成功")
            
            time.sleep(0.5)
            
            try:
                self.loco_client.SwitchMoveMode(True)
                time.sleep(1.0)
                self.get_logger().info("✓ 已切换到运动模式")
            except Exception as e:
                self.get_logger().warn(f"运动模式切换警告: {e}")
            
            self.hardware_mode = True
            self.get_logger().info("🤖 硬件控制模式已启动")
            
        except Exception as e:
            self.get_logger().error(f"❌ SDK初始化失败: {e}")
            self.get_logger().warn("   降级到仿真模式")
            self.hardware_mode = False
            self.loco_client = None
    
    def _send_velocity_command(self, vx: float, vy: float, vyaw: float):
        """
        统一的速度命令发送接口
        
        🔲 Manhattan Motion: 确保 vx 和 vy 不会同时非零
        """
        # Manhattan constraint enforcement
        if abs(vx) > 1e-3 and abs(vy) > 1e-3:
            self.get_logger().warn(
                f"⚠️  违反Manhattan约束: vx={vx:.3f}, vy={vy:.3f} 同时非零! "
                "强制归零较小值"
            )
            if abs(vx) > abs(vy):
                vy = 0.0
            else:
                vx = 0.0
        
        # 创建ROS2速度消息
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.angular.z = vyaw
        
        # 写入cmd_vel日志
        _ts = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        self._cmd_log_file.write(f'{_ts},{vx:.4f},{vy:.4f},{vyaw:.4f}\n')
        
        # 发布 /cmd_vel
        self.vel_pub.publish(cmd_vel)
        
        if self.hardware_mode and self.loco_client is not None:
            try:
                self.loco_client.Move(vx, vy, vyaw)
                self.last_hardware_cmd_time = time.time()
                
                if self.count % 50 == 0:
                    self.get_logger().debug(
                        f"[硬件] SDK命令: vx={vx:.2f}, vy={vy:.2f}, vyaw={vyaw:.2f}"
                    )
            except Exception as e:
                self.get_logger().error(f"硬件SDK命令失败: {e}")
                self.hardware_mode = False
    
    # =========================================================================
    # ROS2 CALLBACKS (mostly unchanged)
    # =========================================================================
    
    def navigation_active_callback(self, msg):
        """处理导航激活状态"""
        was_active = self.navigation_active
        self.navigation_active = msg.data
        self.last_active_time = self.get_clock().now()
        
        if was_active != self.navigation_active:
            if self.navigation_active:
                self.get_logger().info("🚀 Manhattan导航已激活")
                self.current_waypoint_index = 0
                self.last_waypoint_index = -1
                self.last_robot_position = None
                self.target_start_time = None
                self.position_history.clear()
                self.state = ManhattanState.IDLE
                self.current_axis = MovementAxis.STOPPED
            else:
                self.get_logger().info("🛑 Manhattan导航已停止")
                self._send_velocity_command(0.0, 0.0, 0.0)
                self.has_goal = False
                self.has_path = False
                self.waypoints.clear()
                self.state = ManhattanState.IDLE
                self.current_axis = MovementAxis.STOPPED
    
    def odom_callback(self, msg):
        """处理里程计消息"""
        self.current_pose = msg
        
        if self.count % 50 == 0:
            self.get_logger().info(
                f"🔲 当前位置: x={msg.pose.pose.position.x:.2f}, "
                f"y={msg.pose.pose.position.y:.2f}, "
                f"状态={self.state.value}, 轴={self.current_axis.value}"
            )
        self.count += 1
    
    def goal_callback(self, msg):
        """处理目标位置消息"""
        self.goal_pose = msg
        self.has_goal = True
        
        self.get_logger().info(
            f"🎯 收到新目标: x={msg.pose.position.x:.2f}, "
            f"y={msg.pose.position.y:.2f}"
        )
    
    def path_callback(self, msg):
        """处理路径消息"""
        if not msg.poses:
            self.get_logger().warn("收到空路径，忽略")
            return
        
        old_count = len(self.waypoints)
        self.waypoints.clear()
        
        for pose in msg.poses:
            self.waypoints.append(pose)
        
        self.has_path = True
        self.current_waypoint_index = 0
        self.last_waypoint_index = -1
        self.last_robot_position = None
        self.target_start_time = None
        self.position_history.clear()
        self.timeout_triggered = False
        
        # Reset Manhattan state
        self.state = ManhattanState.IDLE
        self.current_axis = MovementAxis.STOPPED
        
        self.get_logger().info(
            f"📍 收到Manhattan路径: {len(self.waypoints)} 个路点, "
            f"目标=({self.waypoints[-1].pose.position.x:.2f}, "
            f"{self.waypoints[-1].pose.position.y:.2f})"
        )
        
        if self.waypoints:
            self.goal_pose = self.waypoints[-1]
            self.has_goal = True
        
        if self.obstacle_avoidance_active:
            self.get_logger().info("收到新路径，重置避障状态")
            self.obstacle_avoidance_active = False
            self.has_obstacle = False
    
    def dwa_cmd_callback(self, msg):
        """处理DWA控制命令"""
        self.dwa_cmd = msg
        self.last_dwa_cmd_time = self.get_clock().now()
    
    def obstacle_status_callback(self, msg):
        """处理障碍物状态消息"""
        new_has_obstacle = (msg.x > 0.5)
        new_avoidance_active = (msg.y > 0.5)
        
        if new_has_obstacle != self.has_obstacle:
            if new_has_obstacle:
                self.get_logger().info("🚧 检测到障碍物")
            else:
                self.get_logger().info("✓ 障碍物已清除")
        
        if new_avoidance_active != self.obstacle_avoidance_active:
            if new_avoidance_active:
                self.get_logger().info("启动DWA避障模式 (Manhattan控制暂停)")
            else:
                self.get_logger().info("退出DWA避障，恢复Manhattan控制")
                self.find_closest_waypoint()
                self.state = ManhattanState.IDLE  # 重置状态机
        
        self.has_obstacle = new_has_obstacle
        self.obstacle_avoidance_active = new_avoidance_active
    
    # =========================================================================
    # WAYPOINT MANAGEMENT (unchanged)
    # =========================================================================
    
    def get_current_waypoint(self):
        """获取当前应该前往的路径点"""
        if self.has_path and self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        elif self.has_goal:
            return self.goal_pose
        
        current = PoseStamped()
        current.header = self.current_pose.header
        current.pose = self.current_pose.pose.pose
        return current
    
    def update_waypoint_if_needed(self):
        """更新当前目标路径点 (使用Manhattan距离判断)"""
        if not self.has_path or self.current_waypoint_index >= len(self.waypoints):
            return
        
        robot_x = self.current_pose.pose.pose.position.x
        robot_y = self.current_pose.pose.pose.position.y
        
        waypoint = self.waypoints[self.current_waypoint_index]
        waypoint_x = waypoint.pose.position.x
        waypoint_y = waypoint.pose.position.y
        
        # 🔲 Manhattan距离
        manhattan_dist = abs(waypoint_x - robot_x) + abs(waypoint_y - robot_y)
        
        # 每次切换目标点时记录时间
        if self.current_waypoint_index != self.last_waypoint_index:
            self.target_start_time = self.get_clock().now()
            self.target_distance = manhattan_dist
            self.timeout_triggered = False
            self.last_waypoint_index = self.current_waypoint_index
            self.last_robot_position = (robot_x, robot_y)
            self.get_logger().info(
                f"切换到路点{self.current_waypoint_index}，"
                f"Manhattan距离={manhattan_dist:.2f}m"
            )
        
        # 检查机器人是否真的在移动
        if hasattr(self, 'last_robot_position'):
            dx = robot_x - self.last_robot_position[0]
            dy = robot_y - self.last_robot_position[1]
            movement = math.sqrt(dx*dx + dy*dy)
            
            if movement < 0.01:
                if self.target_start_time:
                    elapsed = (self.get_clock().now() - self.target_start_time).nanoseconds / 1e9
                    if elapsed < 2.0:
                        return
        
        # 🔲 使用更宽松的到达判断 (Manhattan允许轴向偏差)
        if manhattan_dist < self.waypoint_distance:
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.get_logger().info(
                    f"✓ 到达路点 {self.current_waypoint_index-1}，"
                    f"前往下一路点 {self.current_waypoint_index}"
                )
                self.state = ManhattanState.IDLE  # 重置状态机
    
    def check_timeout(self, distance):
        """根据距离和已用时间判断是否超时"""
        if self.target_start_time is None or self.timeout_triggered:
            return False
        
        elapsed = (self.get_clock().now() - self.target_start_time).nanoseconds / 1e9
        d = self.target_distance if self.target_distance is not None else distance
        
        # Manhattan运动通常需要更长时间 (因为不走直线)
        timeout_multiplier = 1.5  # 增加50%超时时间
        
        if 0 <= d <= 3 and elapsed > 30 * timeout_multiplier:
            self.get_logger().warn(f"Manhattan超时: {d:.2f}m, {elapsed:.2f}s")
            return True
        elif 3 < d <= 10 and elapsed > 60 * timeout_multiplier:
            self.get_logger().warn(f"Manhattan超时: {d:.2f}m, {elapsed:.2f}s")
            return True
        elif 10 < d <= 20 and elapsed > 120 * timeout_multiplier:
            self.get_logger().warn(f"Manhattan超时: {d:.2f}m, {elapsed:.2f}s")
            return True
        elif d > 20 and elapsed > 180 * timeout_multiplier:
            self.get_logger().warn(f"Manhattan超时: {d:.2f}m, {elapsed:.2f}s")
            return True
        return False
    
    def find_closest_waypoint(self):
        """查找最近的路径点 (使用Manhattan距离)"""
        if not self.has_path or not self.current_pose or not self.waypoints:
            return
        
        robot_x = self.current_pose.pose.pose.position.x
        robot_y = self.current_pose.pose.pose.position.y
        
        closest_idx = self.current_waypoint_index
        min_dist = float('inf')
        
        for i in range(self.current_waypoint_index, len(self.waypoints)):
            waypoint = self.waypoints[i]
            # 🔲 Manhattan距离
            dist = (abs(waypoint.pose.position.x - robot_x) + 
                   abs(waypoint.pose.position.y - robot_y))
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        if closest_idx == self.current_waypoint_index and closest_idx + 3 < len(self.waypoints):
            closest_idx += 3
        
        if closest_idx != self.current_waypoint_index:
            self.get_logger().info(
                f"避障后重新定位到路径点 {closest_idx} "
                f"(Manhattan距离: {min_dist:.2f}m)"
            )
            self.current_waypoint_index = closest_idx
    
    # =========================================================================
    # MANHATTAN MOTION CONTROL CORE
    # =========================================================================
    
    def select_movement_axis(self, dx: float, dy: float) -> MovementAxis:
        """
        Select movement axis using greedy strategy with hysteresis
        
        Strategy:
        1. Calculate absolute errors for both axes
        2. Apply hysteresis to prevent rapid switching
        3. Select axis with larger error (greedy)
        4. Enforce minimum switch interval for stability
        
        Args:
            dx: Error in X direction (meters)
            dy: Error in Y direction (meters)
        
        Returns:
            MovementAxis: Selected axis for movement
        """
        abs_dx = abs(dx)
        abs_dy = abs(dy)
        
        # Store errors for logging
        self.error_x = abs_dx
        self.error_y = abs_dy
        
        # Check if both errors are within tolerance (reached)
        if abs_dx < self.axis_tolerance and abs_dy < self.axis_tolerance:
            return MovementAxis.STOPPED
        
        # Check minimum switch interval
        time_since_switch = (self.get_clock().now() - self.last_axis_switch_time).nanoseconds / 1e9
        if time_since_switch < self.axis_switch_delay and self.prev_axis != MovementAxis.STOPPED:
            # Too soon to switch, maintain current axis
            return self.prev_axis
        
        # Hysteresis logic: require significant difference to switch
        # If currently moving on X-axis
        if self.prev_axis == MovementAxis.X_AXIS:
            # Only switch to Y if Y error is significantly larger
            if abs_dy > abs_dx + self.axis_switch_hysteresis:
                selected = MovementAxis.Y_AXIS
            else:
                selected = MovementAxis.X_AXIS
        # If currently moving on Y-axis
        elif self.prev_axis == MovementAxis.Y_AXIS:
            # Only switch to X if X error is significantly larger
            if abs_dx > abs_dy + self.axis_switch_hysteresis:
                selected = MovementAxis.X_AXIS
            else:
                selected = MovementAxis.Y_AXIS
        # If stopped or first movement, use greedy selection
        else:
            if abs_dx >= abs_dy:
                selected = MovementAxis.X_AXIS
            else:
                selected = MovementAxis.Y_AXIS
        
        # Log axis switch
        if selected != self.prev_axis and self.prev_axis != MovementAxis.STOPPED:
            self.get_logger().info(
                f"🔄 轴切换: {self.prev_axis.value} → {selected.value} "
                f"(错误: X={abs_dx:.3f}m, Y={abs_dy:.3f}m)"
            )
            self.last_axis_switch_time = self.get_clock().now()
        
        self.prev_axis = selected
        return selected
    
    def calculate_target_orientation(self, dx: float, dy: float) -> float:
        """
        Calculate target orientation toward goal
        
        Professional SLAM consideration:
        - Robot should face the general direction of goal
        - Helps with sensor alignment (if applicable)
        - Provides more natural motion behavior
        
        Args:
            dx: X component to goal
            dy: Y component to goal
        
        Returns:
            float: Target yaw angle (radians)
        """
        return math.atan2(dy, dx)
    
    def get_current_yaw(self) -> float:
        """Extract yaw from quaternion orientation"""
        if self.current_pose is None:
            return 0.0
        
        q = self.current_pose.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        return yaw
    
    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def manhattan_control_step(self, target: PoseStamped):
        """
        Execute one step of Manhattan motion control
        
        Professional control strategy:
        1. Calculate errors in both axes
        2. Select movement axis (greedy with hysteresis)
        3. Calculate target orientation
        4. Generate velocity commands with axis constraint
        5. Apply speed modulation based on error
        
        Args:
            target: Target waypoint pose
        """
        # Extract positions
        robot_x = self.current_pose.pose.pose.position.x
        robot_y = self.current_pose.pose.pose.position.y
        target_x = target.pose.position.x
        target_y = target.pose.position.y
        
        # Calculate errors
        dx = target_x - robot_x
        dy = target_y - robot_y
        
        # Calculate distances
        manhattan_dist = abs(dx) + abs(dy)
        euclidean_dist = math.sqrt(dx*dx + dy*dy)
        
        # Check timeout
        if self.check_timeout(manhattan_dist):
            self.get_logger().warn(
                f"Manhattan控制超时，已停止机器人！Manhattan距离={manhattan_dist:.2f}m"
            )
            self._send_velocity_command(0.0, 0.0, 0.0)
            self.timeout_triggered = True
            return
        
        # Check if reached goal
        if not self.has_path and euclidean_dist <= self.goal_tolerance:
            self._send_velocity_command(0.0, 0.0, 0.0)
            self.has_goal = False
            self.state = ManhattanState.REACHED
            self.get_logger().info("✓ 到达最终目标位置 (Manhattan控制)")
            return
        
        # Select movement axis
        selected_axis = self.select_movement_axis(dx, dy)
        self.current_axis = selected_axis
        
        # Calculate target orientation
        target_yaw = self.calculate_target_orientation(dx, dy)
        current_yaw = self.get_current_yaw()
        yaw_error = self.normalize_angle(target_yaw - current_yaw)
        
        # Generate velocity command based on selected axis
        vx = 0.0
        vy = 0.0
        vyaw = 0.0
        
        if selected_axis == MovementAxis.STOPPED:
            # Reached current waypoint
            self.state = ManhattanState.REACHED
            vx = vy = vyaw = 0.0
        
        elif selected_axis == MovementAxis.X_AXIS:
            # Move along X-axis
            self.state = ManhattanState.MOVING_X
            
            # Calculate speed based on error (proportional control)
            speed = min(self.max_linear_speed, abs(dx) * 2.0)
            speed = max(self.min_move_speed, speed)
            
            # Apply direction
            vx = speed if dx > 0 else -speed
            vy = 0.0  # Manhattan constraint
            
            # Orientation control (continuous adjustment)
            vyaw = self._calculate_angular_velocity(yaw_error)
        
        elif selected_axis == MovementAxis.Y_AXIS:
            # Move along Y-axis
            self.state = ManhattanState.MOVING_Y
            
            # Calculate speed based on error
            speed = min(self.max_linear_speed, abs(dy) * 2.0)
            speed = max(self.min_move_speed, speed)
            
            # Apply direction
            vx = 0.0  # Manhattan constraint
            vy = speed if dy > 0 else -speed
            
            # Orientation control
            vyaw = self._calculate_angular_velocity(yaw_error)
        
        # Round to 2 decimal places
        vx = round(vx, 2)
        vy = round(vy, 2)
        vyaw = round(vyaw, 2)
        
        # Send command
        self._send_velocity_command(vx, vy, vyaw)
        
        # Logging
        if self.count % 50 == 0:
            mode_tag = "[硬件]" if self.hardware_mode else "[仿真]"
            self.get_logger().info(
                f"{mode_tag} 🔲 Manhattan控制: "
                f"轴={selected_axis.value}, vx={vx:.2f}, vy={vy:.2f}, vyaw={vyaw:.2f}, "
                f"错误(X={self.error_x:.3f}m, Y={self.error_y:.3f}m), "
                f"Manhattan距离={manhattan_dist:.2f}m, "
                f"朝向误差={math.degrees(yaw_error):.1f}°, "
                f"路点={self.current_waypoint_index}/{len(self.waypoints) if self.has_path else 0}"
            )
    
    def _calculate_angular_velocity(self, yaw_error: float) -> float:
        """
        Calculate angular velocity for orientation control
        
        Professional considerations:
        - Proportional control with saturation
        - Minimum speed threshold for hardware motor startup
        - Smooth control without oscillation
        
        Args:
            yaw_error: Yaw error in radians
        
        Returns:
            float: Angular velocity (rad/s)
        """
        # If orientation is acceptable, no rotation needed
        if abs(yaw_error) < self.orientation_tolerance:
            return 0.0
        
        # Proportional control
        k_angular = 1.0  # Angular gain
        vyaw = k_angular * yaw_error
        
        # Saturation
        vyaw = max(-self.max_angular_speed, min(self.max_angular_speed, vyaw))
        
        # Minimum threshold (for hardware motor startup)
        min_angular_speed = 0.10
        if abs(vyaw) > 1e-3 and abs(vyaw) < min_angular_speed:
            vyaw = min_angular_speed * (1 if vyaw > 0 else -1)
        
        return vyaw
    
    # =========================================================================
    # MAIN CONTROL LOOP
    # =========================================================================
    
    def control_loop(self):
        """
        Main control loop with Manhattan motion control
        
        Professional control flow:
        1. Check navigation activation
        2. Verify prerequisites (pose, path)
        3. Handle obstacle avoidance (DWA fallback)
        4. Update waypoint progress
        5. Execute Manhattan control step
        """
        # Check navigation activation
        if not self.navigation_active:
            self._send_velocity_command(0.0, 0.0, 0.0)
            return
        
        # Check activation timeout
        elapsed = (self.get_clock().now() - self.last_active_time).nanoseconds / 1e9
        if elapsed > 5.0:
            self.get_logger().warn(
                f"导航激活信号超时 ({elapsed:.1f}s) - 停止机器人",
                throttle_duration_sec=2.0
            )
            self._send_velocity_command(0.0, 0.0, 0.0)
            return
        
        # Check prerequisites
        if not self.current_pose:
            self.get_logger().warn("没有位置信息，无法执行Manhattan控制", throttle_duration_sec=1.0)
            return
        
        # DWA obstacle avoidance (fallback)
        dwa_cmd_valid = False
        if self.dwa_cmd and self.last_dwa_cmd_time:
            elapsed = (self.get_clock().now() - self.last_dwa_cmd_time).nanoseconds / 1e9
            dwa_cmd_valid = (elapsed < 0.5)
        
        if self.obstacle_avoidance_active and dwa_cmd_valid and self.has_path:
            # Use DWA command (temporarily suspend Manhattan control)
            vx = self.dwa_cmd.linear.x
            vyaw = self.dwa_cmd.angular.z
            
            self._send_velocity_command(vx, 0.0, vyaw)
            
            if self.count % 50 == 0:
                self.get_logger().info(
                    f"DWA避障控制 (Manhattan暂停): vx={vx:.2f}, vyaw={vyaw:.2f}"
                )
            return
        
        elif self.obstacle_avoidance_active and (not self.has_path or 
                self.current_waypoint_index >= len(self.waypoints)):
            self.get_logger().info("没有有效路径，退出DWA避障模式")
            self.obstacle_avoidance_active = False
            self._send_velocity_command(0.0, 0.0, 0.0)
            return
        
        # Update waypoint progress
        if self.has_path and not self.obstacle_avoidance_active:
            self.update_waypoint_if_needed()
        
        # Check if path is complete
        if (not self.has_goal and not self.has_path) or \
           (self.has_path and self.current_waypoint_index >= len(self.waypoints)):
            self._send_velocity_command(0.0, 0.0, 0.0)
            
            if self.has_path and self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info("✓ Manhattan路径导航完成")
                self.has_path = False
                self.state = ManhattanState.REACHED
            return
        
        # Skip Manhattan control during DWA avoidance
        if self.obstacle_avoidance_active:
            return
        
        # Get current target waypoint
        target = self.get_current_waypoint()
        
        # Execute Manhattan control step
        self.manhattan_control_step(target)
    
    # =========================================================================
    # SHUTDOWN
    # =========================================================================
    
    def shutdown(self):
        """安全关闭节点"""
        self.get_logger().info("正在关闭Manhattan导航控制节点...")
        
        try:
            self._send_velocity_command(0.0, 0.0, 0.0)
            self.get_logger().info("✓ 已发送停止命令")
        except Exception as e:
            self.get_logger().error(f"发送停止命令失败: {e}")
        
        if self.hardware_mode and self.loco_client is not None:
            try:
                self.loco_client.StopMove()
                self.get_logger().info("✓ 硬件控制已安全停止")
            except Exception as e:
                self.get_logger().error(f"硬件停止失败: {e}")
        
        # 关闭cmd_vel日志文件
        try:
            self._cmd_log_file.close()
            self.get_logger().info("✓ cmd_vel日志文件已关闭")
        except Exception as e:
            self.get_logger().error(f"关闭日志文件失败: {e}")


def main(args=None):
    """Main function with proper shutdown handling"""
    rclpy.init(args=args)
    node = None
    
    try:
        node = G1ManhattanPositionControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f'Unexpected error: {e}')
    finally:
        if node is not None:
            try:
                node.shutdown()
            except Exception as e:
                print(f"[警告] 关闭失败: {e}")
            
            try:
                node.destroy_node()
            except Exception:
                pass
        
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
