#!/usr/bin/env python3
"""
G1 Position Control Node for ROS2 Humble
Author: shenlanxueyuan
Email: 1347893477@qq.com
Organization: Shen Lan xue yuan
Date: 2025-10-14

This node implements position control for the G1 robot, including:
- Path following using A* path
- Obstacle avoidance using DWA
- Position control with dynamic speed adjustment
- Direct hardware control via Unitree SDK (实体机器人模式)
- ROS2 /cmd_vel fallback (仿真模式)

Migration Notes (ROS1 → ROS2):
- Changed from rospy to rclpy
- Updated to use Node class (object-oriented)
- Timer callback signature changed (no event parameter)
- Parameter API changed (declare_parameter required)
- Time API changed (get_clock())
- Logging changed (get_logger())

Hardware Integration (2025-11-26):
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
import datetime

# 🔧 紧急修复: 动态添加 unitree_sdk2py 路径到 sys.path
# 解决 launch 文件启动时 PYTHONPATH 未正确继承的问题
unitree_sdk_path_added = False
try:
    # 方法1: 从 ROS2 安装目录搜索
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
        # 方法2: 从工作空间推断路径
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


class G1PositionControl(Node):
    def __init__(self):
        # 初始化ROS2节点
        super().__init__('g1_position_control')
        
        # 声明参数 (ROS2必须先声明参数)
        self.declare_parameter('waypoint_distance', 0.5)
        self.declare_parameter('goal_tolerance', 0.20) # 🔧 减小目标容忍距离 (0.4 → 0.1), 20260203, York
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 0.45)  # 🔧 降低角速度避免定位丢失 (0.3 → 0.12)
        self.declare_parameter('use_hardware', False)  # 🆕 是否使用实体硬件
        self.declare_parameter('network_interface', 'ens37')  # 🆕 网络接口
        self.declare_parameter('cmd_timeout', 0.5)  # 🆕 命令超时时间
        
        # 获取参数值
        self.waypoint_distance = self.get_parameter('waypoint_distance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.use_hardware = self.get_parameter('use_hardware').value
        self.network_interface = self.get_parameter('network_interface').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        
        # 初始化目标位置标志
        self.has_goal = False
        self.has_path = False
        
        # 障碍物避障模式标志
        self.has_obstacle = False
        self.obstacle_avoidance_active = False
        
        # 存储当前位置、目标位置和路径
        self.current_pose = None
        self.goal_pose = None
        self.waypoints = []
        
        # 存储DWA控制命令
        self.dwa_cmd = None
        self.last_dwa_cmd_time = None
        
        # 路径导航相关变量
        self.current_waypoint_index = 0
        
        # 消息计数器，用于减少日志频率
        self.count = 0

        # 超时检测相关变量
        self.target_start_time = None
        self.target_distance = None
        self.timeout_triggered = False
        self.last_waypoint_index = -1
        self.last_robot_position = None  # 记录上次机器人位置,用于检测是否真的在移动
        self.position_history = []  # 🔧 新增: 位置历史记录
        
        # 🆕 Unitree SDK 硬件控制相关
        self.loco_client = None
        self.hardware_mode = False  # 实际运行模式标志
        self.last_hardware_cmd_time = time.time()
        
        # 🆕 初始化硬件连接 (如果启用)
        if self.use_hardware:
            self._init_hardware_control()
        else:
            self.get_logger().info("🎮 仿真模式: 使用 /cmd_vel 发布速度命令")
        
        # QoS配置
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅ROS2定位话题 - 获取机器人当前位置
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/map',
            self.odom_callback,
            sensor_qos
        )
        
        # 订阅2D Nav Goal设置的目标位置话题
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',  # 或使用 /goal_pose
            self.goal_callback,
            reliable_qos
        )
        
        # 订阅导航路径话题
        self.path_sub = self.create_subscription(
            Path,
            '/astar_path',
            self.path_callback,
            reliable_qos
        )
        
        # 订阅DWA避障控制命令
        self.dwa_cmd_sub = self.create_subscription(
            Twist,
            '/dwa_cmd_vel',
            self.dwa_cmd_callback,
            reliable_qos
        )
        
        # 订阅障碍物状态
        self.obstacle_status_sub = self.create_subscription(
            Point,
            '/obstacle_status',
            self.obstacle_status_callback,
            reliable_qos
        )
        
        # 创建速度指令发布者
        self.vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            reliable_qos
        )
        
        # 🆕 订阅导航激活状态 (来自 Action Server 的控制信号)
        self.nav_active_sub = self.create_subscription(
            Bool,
            '/navigation_active',
            self.navigation_active_callback,
            reliable_qos
        )
        
        # 导航激活标志 (由 Action Server 控制)
        self.navigation_active = False
        self.last_active_time = self.get_clock().now()
        
        # 初始化cmd_vel日志文件
        _script_dir = os.path.dirname(os.path.abspath(__file__))
        _log_dir = os.path.join(_script_dir, 'logs', 'cmd_logs')
        os.makedirs(_log_dir, exist_ok=True)
        _log_name = datetime.datetime.now().strftime('cmd_vel_%Y%m%d-%H%M%S.log')
        self._cmd_log_file = open(os.path.join(_log_dir, _log_name), 'w', buffering=1)
        self._cmd_log_file.write('datetime_ms,vx,vy,vyaw\n')
        
        # 创建定时器，每100ms检查是否需要移动
        # ROS2: Timer callback doesn't receive event parameter
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        mode_str = "🤖 硬件模式" if self.hardware_mode else "🎮 仿真模式"
        self.get_logger().info(f"G1导航控制节点已启动 [{mode_str}]，等待路径信息...")
    
    def _init_hardware_control(self):
        """
        初始化Unitree SDK硬件控制
        
        功能:
            - 初始化DDS通道
            - 创建LocoClient运动控制客户端
            - 切换到运动模式
            - 异常处理和fallback
        """
        if not UNITREE_SDK_AVAILABLE:
            self.get_logger().error("❌ Unitree SDK 未安装,无法使用硬件模式")
            self.get_logger().warn("   降级到仿真模式,将发布 /cmd_vel 消息")
            self.hardware_mode = False
            return
        
        try:
            self.get_logger().info("🔧 初始化 Unitree SDK...")
            
            # 初始化DDS通道
            try:
                ChannelFactoryInitialize(0, self.network_interface)
                self.get_logger().info(f"✓ DDS通道初始化成功 (接口: {self.network_interface})")
            except Exception as e:
                self.get_logger().warn(f"接口 {self.network_interface} 初始化失败: {e}")
                self.get_logger().info("尝试使用默认接口...")
                ChannelFactoryInitialize(0, "")
                self.get_logger().info("✓ 使用默认接口初始化成功")
            
            # 创建运动控制客户端
            self.loco_client = LocoClient()
            self.loco_client.SetTimeout(10.0)
            self.loco_client.Init()
            self.get_logger().info("✓ G1 运动控制客户端创建成功")
            
            # 等待连接稳定
            time.sleep(0.5)
            
            # 切换到运动模式
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
        
        参数:
            vx: 前进速度 (m/s)
            vy: 横向速度 (m/s)  
            vyaw: 旋转角速度 (rad/s)
        
        根据运行模式自动选择:
            - 硬件模式: 发布 /cmd_vel 消息 AND 调用 loco_client.Move()
            - 仿真模式: 仅发布 /cmd_vel 消息
        
        说明:
            硬件模式下同时发布 /cmd_vel 是为了:
            1. 让ROS2可视化工具(如RViz)显示速度命令
            2. 允许其他监控节点订阅速度信息
            3. 保持完整的ROS2生态兼容性
            4. 便于调试和数据记录
        """
        # 创建ROS2速度消息 (两种模式都需要)
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.angular.z = vyaw
        
        # 写入cmd_vel日志
        _ts = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        self._cmd_log_file.write(f'{_ts},{vx:.4f},{vy:.4f},{vyaw:.4f}\n')
        
        # 始终发布 /cmd_vel 消息 (硬件模式和仿真模式都发布)
        self.vel_pub.publish(cmd_vel)
        
        if self.hardware_mode and self.loco_client is not None:
            # 硬件模式: 额外调用SDK直接控制G1
            try:
                self.loco_client.Move(vx, vy, vyaw)
                self.last_hardware_cmd_time = time.time()
                
                # 降低日志频率
                if self.count % 50 == 0:
                    self.get_logger().debug(
                        f"[硬件] SDK命令: vx={vx:.2f}, vy={vy:.2f}, vyaw={vyaw:.2f} "
                        f"| /cmd_vel已同步发布"
                    )
            except Exception as e:
                self.get_logger().error(f"硬件SDK命令失败: {e}")
                # 尝试降级到仿真模式
                self.get_logger().warn("SDK控制失败,降级到仅/cmd_vel模式...")
                self.hardware_mode = False
                # 注意: /cmd_vel已经在上面发布过了,这里不需要递归调用
    
    def navigation_active_callback(self, msg):
        """
        处理导航激活状态 (来自 Action Server)
        
        功能:
            - True: Action Server 正在执行任务,允许发布 /cmd_vel
            - False: Action Server 无任务或已停止,禁止发布 /cmd_vel
        
        说明:
            这是 Action Server 控制 Position Control 的关键机制
        """
        was_active = self.navigation_active
        self.navigation_active = msg.data
        self.last_active_time = self.get_clock().now()
        
        # 状态变化时记录日志
        if was_active != self.navigation_active:
            if self.navigation_active:
                self.get_logger().info("🚀 导航已激活 - Action Server 开始新任务")
                # 🔧 完整重置所有状态变量（避免任务间污染）
                self.current_waypoint_index = 0
                self.last_waypoint_index = -1
                self.last_robot_position = None
                self.target_start_time = None
                self.position_history.clear()
            else:
                self.get_logger().info("🛑 导航已停止 - Action Server 结束任务")
                # 立即发送零速度停止机器人
                self._send_velocity_command(0.0, 0.0, 0.0)
                # 🔧 停止时也清理状态
                self.has_goal = False
                self.has_path = False
                self.waypoints.clear()
                self.current_waypoint_index = 0
                self.last_waypoint_index = -1
                self.last_robot_position = None
                self.current_goal = None
                self.current_path = None
    
    def odom_callback(self, msg):
        """处理里程计消息"""
        self.current_pose = msg
        
        # 输出当前位置，便于调试 (使用throttle减少日志)
        if self.count % 50 == 0:
            self.get_logger().info(
                f"当前位置: x={msg.pose.pose.position.x:.2f}, "
                f"y={msg.pose.pose.position.y:.2f}, "
                f"frame_id={msg.header.frame_id}"
            )
        self.count += 1
        
    def goal_callback(self, msg):
        """处理目标位置消息"""
        self.goal_pose = msg
        self.has_goal = True
        
        self.get_logger().info(
            f"收到新目标: x={msg.pose.position.x:.2f}, "
            f"y={msg.pose.position.y:.2f}, "
            f"frame_id={msg.header.frame_id}"
        )
        
    def path_callback(self, msg):
        """处理路径消息"""
        if not msg.poses:
            self.get_logger().warn("收到空路径，忽略")
            return
        
        # 🔧 记录旧状态用于调试
        old_index = self.current_waypoint_index
        old_count = len(self.waypoints)
        
        # 清空当前路径点
        self.waypoints.clear()
        
        # 提取路径中的路点
        self.get_logger().info("全部路径点:")
        for pose in msg.poses:
            self.waypoints.append(pose)
        
        self.has_path = True
        
        # 🔧 强制重置，无论 navigation_active 状态
        self.current_waypoint_index = 0
        self.last_waypoint_index = -1
        self.last_robot_position = None
        self.target_start_time = None
        self.position_history.clear()  # ← 新增：清空位置历史
        self.timeout_triggered = False  # ← 新增：重置超时标志
        
        self.get_logger().info(
            f"📍 收到新路径: {len(self.waypoints)} 个路点 (旧路径{old_count}个), "
            f"目标=({self.waypoints[-1].pose.position.x:.2f}, {self.waypoints[-1].pose.position.y:.2f}), "
            f"frame_id={msg.header.frame_id}, "
            f"waypoint_index重置: {old_index}→0"
        )
        
        # 设置最终目标
        if self.waypoints:
            self.goal_pose = self.waypoints[-1]
            self.has_goal = True
        
        # 重置避障状态
        if self.obstacle_avoidance_active:
            self.get_logger().info("收到新路径，重置避障状态")
            self.obstacle_avoidance_active = False
            self.has_obstacle = False
            
    def dwa_cmd_callback(self, msg):
        """处理DWA控制命令"""
        self.dwa_cmd = msg
        self.last_dwa_cmd_time = self.get_clock().now()
        
        # 添加调试信息
        if self.count % 50 == 0:
            self.get_logger().info(
                f"收到DWA控制命令: vx={msg.linear.x:.2f}, "
                f"vyaw={msg.angular.z:.2f}"
            )
            
    def obstacle_status_callback(self, msg):
        """处理障碍物状态消息"""
        # msg.x: 0表示无障碍物，1表示有障碍物
        # msg.y: 0表示未激活避障，1表示激活避障
        new_has_obstacle = (msg.x > 0.5)
        new_avoidance_active = (msg.y > 0.5)
        
        # 状态变化时记录日志
        if new_has_obstacle != self.has_obstacle:
            if new_has_obstacle:
                self.get_logger().info("检测到障碍物")
            else:
                self.get_logger().info("障碍物已清除")
        
        if new_avoidance_active != self.obstacle_avoidance_active:
            if new_avoidance_active:
                self.get_logger().info("启动DWA避障模式")
            else:
                self.get_logger().info("退出DWA避障模式，恢复A*路径跟踪")
                # 避障模式关闭时，查找新的路径点
                self.find_closest_waypoint()
        
        self.has_obstacle = new_has_obstacle
        self.obstacle_avoidance_active = new_avoidance_active
        
    def get_current_waypoint(self):
        """获取当前应该前往的路径点"""
        if self.has_path and self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        elif self.has_goal:
            return self.goal_pose
        
        # 默认返回当前位置（不移动）
        current = PoseStamped()
        current.header = self.current_pose.header
        current.pose = self.current_pose.pose.pose
        return current
        
    def update_waypoint_if_needed(self):
        """更新当前目标路径点"""
        if not self.has_path or self.current_waypoint_index >= len(self.waypoints):
            return
        
        robot_x = self.current_pose.pose.pose.position.x
        robot_y = self.current_pose.pose.pose.position.y
        
        waypoint = self.waypoints[self.current_waypoint_index]
        waypoint_x = waypoint.pose.position.x
        waypoint_y = waypoint.pose.position.y
        
        distance = math.sqrt((waypoint_x - robot_x)**2 + (waypoint_y - robot_y)**2)
        
        # 每次切换目标点时，记录时间和距离
        if self.current_waypoint_index != self.last_waypoint_index:
            self.target_start_time = self.get_clock().now()
            self.target_distance = distance
            self.timeout_triggered = False
            self.last_waypoint_index = self.current_waypoint_index
            self.last_robot_position = (robot_x, robot_y)  # 记录上次机器人位置
            self.get_logger().info(
                f"切换到新目标点{self.current_waypoint_index}，"
                f"距离={distance:.2f}m，计时开始"
            )
        
        # 检查机器人是否真的在移动(验证位置变化)
        if hasattr(self, 'last_robot_position'):
            dx = robot_x - self.last_robot_position[0]
            dy = robot_y - self.last_robot_position[1]
            movement = math.sqrt(dx*dx + dy*dy)
            
            # 如果机器人位置几乎没变化(小于1cm),认为没有运动
            if movement < 0.01:
                # 位置未变化,可能是仿真或机器人未连接,限制路点跳跃速度
                # 只有每2秒才允许检查一次是否到达路点
                if self.target_start_time:
                    elapsed = (self.get_clock().now() - self.target_start_time).nanoseconds / 1e9
                    if elapsed < 2.0:
                        return  # 还未到检查时间,直接返回
        
        # 如果距离当前路点足够近，更新到下一个路点
        if distance < self.waypoint_distance:
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.get_logger().info(
                    f"到达路点 {self.current_waypoint_index-1}，"
                    f"前往下一路点 {self.current_waypoint_index}"
                )
    
    def check_timeout(self, distance):
        """根据距离和已用时间判断是否超时"""
        if self.target_start_time is None or self.timeout_triggered:
            return False
        
        # ROS2: Time difference calculation
        elapsed = (self.get_clock().now() - self.target_start_time).nanoseconds / 1e9
        d = self.target_distance if self.target_distance is not None else distance
        
        # 根据距离设置不同的超时阈值
        if 0 <= d <= 3 and elapsed > 30:
            self.get_logger().warn(
                f"距离 {d:.2f}m，已用时间 {elapsed:.2f}秒，触发超时"
            )
            return True
        elif 3 < d <= 10 and elapsed > 60:
            self.get_logger().warn(
                f"距离 {d:.2f}m，已用时间 {elapsed:.2f}秒，触发超时"
            )
            return True
        elif 10 < d <= 20 and elapsed > 120:
            self.get_logger().warn(
                f"距离 {d:.2f}m，已用时间 {elapsed:.2f}秒，触发超时"
            )
            return True
        elif d > 20 and elapsed > 180:
            self.get_logger().warn(
                f"距离 {d:.2f}m，已用时间 {elapsed:.2f}秒，触发超时"
            )
            return True
        return False
    
    def find_closest_waypoint(self):
        """查找最近的路径点"""
        if not self.has_path or not self.current_pose or not self.waypoints:
            return
        
        robot_x = self.current_pose.pose.pose.position.x
        robot_y = self.current_pose.pose.pose.position.y
        
        closest_idx = self.current_waypoint_index
        min_dist = float('inf')
        
        # 从当前索引开始查找最近的路径点
        for i in range(self.current_waypoint_index, len(self.waypoints)):
            waypoint = self.waypoints[i]
            dist = math.sqrt(
                (waypoint.pose.position.x - robot_x)**2 + 
                (waypoint.pose.position.y - robot_y)**2
            )
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # 如果找到的点就是当前点，尝试前进2-3个点以避开障碍物
        if closest_idx == self.current_waypoint_index and closest_idx + 3 < len(self.waypoints):
            closest_idx += 3
        
        if closest_idx != self.current_waypoint_index:
            self.get_logger().info(
                f"避障后重新定位到路径点 {closest_idx} "
                f"(原路径点: {self.current_waypoint_index}，"
                f"总点数: {len(self.waypoints)})"
            )
            self.current_waypoint_index = closest_idx
    
    def control_loop(self):
        """主控制循环 (ROS2: No event parameter)"""
        # 🆕 检查导航是否激活 (Action Server 控制)
        if not self.navigation_active:
            # 导航未激活时,确保机器人停止
            self._send_velocity_command(0.0, 0.0, 0.0)
            return
        
        # 检查激活信号超时 (5秒无更新则视为 Action Server 异常)
        elapsed = (self.get_clock().now() - self.last_active_time).nanoseconds / 1e9
        if elapsed > 5.0:
            self.get_logger().warn(
                f"导航激活信号超时 ({elapsed:.1f}s 无更新) - 停止机器人",
                throttle_duration_sec=2.0
            )
            self._send_velocity_command(0.0, 0.0, 0.0)
            return
        
        # 如果没有当前位置，则返回
        if not self.current_pose:
            self.get_logger().warn("没有收到位置信息，无法执行控制", throttle_duration_sec=1.0)
            return
        
        # 添加调试信息
        self.get_logger().info(
            f"控制状态: has_goal={self.has_goal}, has_path={self.has_path}, "
            f"obstacle_avoidance_active={self.obstacle_avoidance_active}, "
            f"current_waypoint_index={self.current_waypoint_index}, "
            f"total_waypoints={len(self.waypoints) if self.has_path else 0}",
            throttle_duration_sec=2.0
        )
        
        # 检查DWA控制命令是否过期（超过0.5秒）
        dwa_cmd_valid = False
        if self.dwa_cmd and self.last_dwa_cmd_time:
            # ROS2: Time difference calculation
            elapsed = (self.get_clock().now() - self.last_dwa_cmd_time).nanoseconds / 1e9
            dwa_cmd_valid = (elapsed < 0.5)
        
        # 如果有障碍物并且DWA命令有效，并且有路径，则使用DWA控制
        if self.obstacle_avoidance_active and dwa_cmd_valid and self.has_path:
            # 使用DWA避障控制命令
            vx = self.dwa_cmd.linear.x
            vyaw = self.dwa_cmd.angular.z
            
            self._send_velocity_command(vx, 0.0, vyaw)
            
            if self.count % 50 == 0:
                self.get_logger().info(
                    f"DWA避障控制: vx={vx:.2f}, vyaw={vyaw:.2f}"
                )
            return
        elif self.obstacle_avoidance_active and (not self.has_path or self.current_waypoint_index >= len(self.waypoints)):
            # 如果处于避障模式但没有路径或路径已完成，则退出避障模式
            self.get_logger().info("没有有效路径或路径已完成，退出DWA避障模式")
            self.obstacle_avoidance_active = False
            
            # 发送停止命令
            self._send_velocity_command(0.0, 0.0, 0.0)
            return
        
        # 如果有路径，根据当前位置更新要前往的路点
        if self.has_path and not self.obstacle_avoidance_active:
            self.update_waypoint_if_needed()
        
        # 如果没有目标或路径已完成，则停止
        if (not self.has_goal and not self.has_path) or \
           (self.has_path and self.current_waypoint_index >= len(self.waypoints)):
            self._send_velocity_command(0.0, 0.0, 0.0)
            
            if self.has_path and self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info("路径导航完成，停止移动")
                self.has_path = False
            return
        
        # 如果处于避障模式，不执行A*路径跟踪
        if self.obstacle_avoidance_active:
            return
        
        # 获取当前导航目标点
        target = self.get_current_waypoint()
        
        # 提取当前位置和目标位置
        robot_x = self.current_pose.pose.pose.position.x
        robot_y = self.current_pose.pose.pose.position.y
        target_x = target.pose.position.x
        target_y = target.pose.position.y
        
        # 计算位置差异
        dx = target_x - robot_x
        dy = target_y - robot_y
        
        # 计算移动方向和速度
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 超时检测
        if self.check_timeout(distance):
            self.get_logger().warn(
                f"到达目标点超时，已停止机器人！距离={distance:.2f}m"
            )
            self._send_velocity_command(0.0, 0.0, 0.0)
            self.timeout_triggered = True
            return
        
        # 检查是否到达最终目标
        if not self.has_path and distance <= self.goal_tolerance:
            # 接近最终目标，停止
            self._send_velocity_command(0.0, 0.0, 0.0)
            
            # 清除目标，避免重复控制
            self.has_goal = False
            self.get_logger().info("到达最终目标位置，停止移动")
            return
        
        # 计算移动方向
        theta = math.atan2(dy, dx)
        
        # 从四元数中提取朝向角
        q = self.current_pose.pose.pose.orientation
        # 计算yaw角 (z轴旋转)
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        angle_diff = theta - yaw
        
        # 🔧 修复: 归一化角度到[-PI, PI]（使用 atan2 保证最短路径）
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        # 创建控制命令
        cmd_vel = Twist()
        
        # 🔧 纯转向模式策略: 角度偏差大时停止前进,专注转向
        # 最小角速度设为 0.15 rad/s 以确保硬件电机能够启动
        vx = self.max_linear_speed
        if abs(angle_diff) > 0.5:  # 角度偏差 > 28.6度时进入纯转向模式
            vx = 0.0
            # 确保角速度不低于 0.15 rad/s (硬件电机启动阈值)
            vyaw_magnitude = max(0.15, min(self.max_angular_speed, abs(angle_diff) * 1.0))
            vyaw = vyaw_magnitude * (1 if angle_diff > 0 else -1)
        else:
            # 角度偏差较小时,允许前进并微调方向
            if abs(angle_diff) > 0.3:
                # 角度偏差中等时减速
                vx = self.max_linear_speed * (1.0 - abs(angle_diff) / math.pi)
            
            # York, 20260203: 移除接近目标时减速逻辑，避免过早减速导致无法到达目标
            if distance < 0.5:
                # 接近目标时减速
                vx = vx * (distance / 0.5)

            # 角速度与偏差成比例
            vyaw = min(self.max_angular_speed, abs(angle_diff) * 1.0) * (1 if angle_diff > 0 else -1)
        
        # Round to 2 decimal places
        vx = round(vx, 2)
        vyaw = round(vyaw, 2)
        
        # 发送速度命令 (自动选择硬件/仿真模式)
        self._send_velocity_command(vx, 0.0, vyaw)
        
        if self.count % 50 == 0:
            mode_tag = "[硬件]" if self.hardware_mode else "[仿真]"
            self.get_logger().info(
                f"{mode_tag} 导航控制: vx={vx:.2f}, vyaw={vyaw:.2f}, "
                f"距离={distance:.2f}, 朝向={yaw:.2f}→{theta:.2f}, "
                f"路点={self.current_waypoint_index}/{len(self.waypoints) if self.has_path else 0}"
            )
    
    def shutdown(self):
        """
        安全关闭节点
        
        功能:
            - 发送停止命令
            - 关闭硬件连接
            - 清理资源
        """
        self.get_logger().info("正在关闭G1导航控制节点...")
        
        # 发送停止命令
        try:
            self._send_velocity_command(0.0, 0.0, 0.0)
            self.get_logger().info("✓ 已发送停止命令")
        except Exception as e:
            self.get_logger().error(f"发送停止命令失败: {e}")
        
        # 关闭硬件连接
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
        node = G1PositionControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # User pressed Ctrl+C
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f'Unexpected error: {e}')
    finally:
        # Safe cleanup: check if context is still valid before shutdown
        if node is not None:
            try:
                node.shutdown()  # 🆕 调用安全关闭方法
            except Exception as e:
                print(f"[警告] 关闭失败: {e}")
            
            try:
                node.destroy_node()
            except Exception:
                pass
        
        # Only shutdown if not already done
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            # Already shutdown, ignore
            pass


if __name__ == '__main__':
    main()
