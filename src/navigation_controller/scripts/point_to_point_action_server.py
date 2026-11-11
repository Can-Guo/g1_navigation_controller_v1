#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
================================================================================
PointToPoint Navigation Action Server - G1 Unitree Robot
================================================================================

功能描述:
    ROS2 Action 服务器,实现基于协议 v1.0 的点对点导航控制
    整合 A* 全局规划器、DWA 局部规划器和位置控制器
    提供实时反馈、状态监控和紧急停止机制

核心职责:
    1. 接收 MCB 发送的导航目标 (target_pose)
    2. 监控导航子系统状态 (A*, DWA, localization)
    3. 提供实时反馈 (位姿、速度、障碍物、进度)
    4. 处理异常情况 (定位丢失、路径阻塞、超时)
    5. 支持任务抢占和紧急停止

系统架构:
    MCB (客户端) <--Action--> PointToPoint Server <--> Navigation Stack
                                    |
                        +-----------+-----------+
                        |           |           |
                      A* Planner  DWA Planner  Position Control
                        |           |           |
                        +------> /cmd_vel <-----+

话题订阅:
    - /odometry/map (nav_msgs/Odometry)
        来源: mid360_locate 定位节点
        用途: 获取当前位姿和速度
    
    - /astar_path (nav_msgs/Path)
        来源: astar_planner_ros2.py
        用途: 获取全局路径,计算剩余距离
    
    - /dwa_obstacles (sensor_msgs/PointCloud2)
        来源: dwa_planner_ros2_mid360.py
        用途: 检测最近障碍物距离
    
    - /localization_confidence (std_msgs/Float32)
        来源: mid360_locate (需添加发布)
        用途: 监控定位质量
    
    - /emergency_stop (std_msgs/Bool)
        来源: 安全监控节点或 MCB
        用途: 紧急停止信号

话题发布:
    - /move_base_simple/goal (geometry_msgs/PoseStamped)
        目标: A* 规划器
        用途: 触发全局路径规划

服务提供:
    - ~/point_to_point (robot_interfaces/action/navigation/PointToPoint.action)
        Action 服务器主接口

安全机制:
    1. 实时定位监控 (置信度阈值: 0.3)
    2. 卡住检测 (5秒内移动 < 0.1m)
    3. 超时保护 (动态计算,最大 300 秒)
    4. 紧急停止响应时间 < 100ms
    5. 任务抢占支持

作者: G1 Navigation Team
日期: 2025-11-05
版本: 1.0.0
ROS2: Humble
Python: 3.10+
================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import math
import time
from threading import Lock
from collections import deque

# ROS2 Messages
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32
from sensor_msgs_py import point_cloud2

# 自定义 Action 接口  
from robot_interfaces.action import PointToPoint


class PointToPointActionServer(Node):
    """
    点对点导航 Action 服务器
    
    功能:
        - 管理导航任务生命周期
        - 整合多个导航节点的状态
        - 提供实时反馈和异常处理
        - 支持紧急停止和任务抢占
    
    状态机:
        IDLE → PLANNING → FOLLOWING → (AVOIDING/RECOVERING) → GOAL_REACHED/FAILED
    """
    
    def __init__(self):
        super().__init__('action_nav_point_to_point')
        
        # ====================================================================
        # 参数声明与配置
        # ====================================================================
        self.declare_parameter('goal_tolerance',  0.20)  # 目标到达阈值 [m] #  减小目标容忍距,York,20250203, 0.3 -> 0.1
        self.declare_parameter('localization_confidence_threshold', 0.20)  # 定位置信度阈值
        self.declare_parameter('stuck_timeout', 5.0)  # 卡住检测时间 [s]
        self.declare_parameter('stuck_distance_threshold', 0.1)  # 卡住距离阈值 [m]
        self.declare_parameter('base_timeout', 150.0)  # 基础超时时间 [s] (增加到150秒)
        self.declare_parameter('timeout_per_meter', 5.0)  # 每米额外超时 [s/m] (增加到5秒/米)
        self.declare_parameter('max_timeout', 180.0)  # 最大超时时间 [s] (增加到180秒)
        self.declare_parameter('feedback_rate', 5.0)  # 反馈发布频率 [Hz]
        self.declare_parameter('obstacle_critical_distance', 0.5)  # 危险障碍物距离 [m]
        self.declare_parameter('obstacle_near_distance', 1.5)  # 接近障碍物距离 [m]
        self.declare_parameter('recovery_timeout', 10.0)  # 恢复行为超时 [s]
        
        # 读取参数
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.localization_confidence_threshold = self.get_parameter('localization_confidence_threshold').value
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        self.stuck_distance_threshold = self.get_parameter('stuck_distance_threshold').value
        self.base_timeout = self.get_parameter('base_timeout').value
        self.timeout_per_meter = self.get_parameter('timeout_per_meter').value
        self.max_timeout = self.get_parameter('max_timeout').value
        self.feedback_rate = self.get_parameter('feedback_rate').value
        self.obstacle_critical_distance = self.get_parameter('obstacle_critical_distance').value
        self.obstacle_near_distance = self.get_parameter('obstacle_near_distance').value
        self.recovery_timeout = self.get_parameter('recovery_timeout').value
        
        # ====================================================================
        # 状态变量初始化
        # ====================================================================
        self.state_lock = Lock()  # 线程安全锁
        
        # 当前任务状态
        self.current_task_id = None
        self.current_goal = None
        self.task_start_time = None
        self.task_timeout = None
        
        # 机器人状态
        self.current_pose = None
        self.current_velocity = Twist()
        self.localization_confidence = 1.0  # 初始假设定位正常
        
        # 导航状态
        self.navigation_state = PointToPoint.Feedback.STATE_PLANNING
        self.obstacle_proximity = PointToPoint.Feedback.OBSTACLE_CLEAR
        self.current_path = None
        self.path_length_remaining = 0.0
        
        # 卡住检测
        self.position_history = deque(maxlen=int(self.stuck_timeout * 10))  # 10Hz 采样
        self.last_progress_time = None
        
        # 紧急停止标志
        self.emergency_stop_flag = False
        
        # 恢复行为状态
        self.recovery_start_time = None
        
        # 警告限流
        self.last_confidence_warning_time = 0.0
        self.confidence_warning_interval = 2.0  # 每 2 秒最多警告一次
        
        # ====================================================================
        # QoS 配置
        # ====================================================================
        # 传感器数据 QoS (Best Effort, 适用于高频数据)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 导航命令 QoS (Reliable, 确保关键命令送达)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ====================================================================
        # ROS2 订阅器
        # ====================================================================
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/map',
            self.odometry_callback,
            sensor_qos
        )
        
        self.path_sub = self.create_subscription(
            Path,
            '/astar_path',
            self.path_callback,
            reliable_qos
        )
        
        self.obstacles_sub = self.create_subscription(
            PointCloud2,
            '/dwa_obstacles',
            self.obstacles_callback,
            sensor_qos
        )
        
        self.confidence_sub = self.create_subscription(
            Float32,
            '/localization_confidence',
            self.confidence_callback,
            sensor_qos
        )
        
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_stop_callback,
            reliable_qos
        )
        
        # ====================================================================
        # ROS2 发布器
        # ====================================================================
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/move_base_simple/goal',
            reliable_qos
        )
        
        # 🆕 导航激活状态发布器 (控制 Position Control 执行)
        self.nav_active_pub = self.create_publisher(
            Bool,
            '/navigation_active',
            reliable_qos
        )
        
        # 🆕 创建定时器持续发布激活信号 (心跳机制,2Hz)
        self.heartbeat_timer = None
        
        # ====================================================================
        # Action Server
        # ====================================================================
        self._action_server = ActionServer(
            self,
            PointToPoint,
            'action_nav_point_to_point',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        
        self.get_logger().info('='*80)
        self.get_logger().info('PointToPoint Action Server Initialized')
        self.get_logger().info('='*80)
        self.get_logger().info(f'Goal Tolerance: {self.goal_tolerance} m')
        self.get_logger().info(f'Localization Confidence Threshold: {self.localization_confidence_threshold}')
        self.get_logger().info(f'Feedback Rate: {self.feedback_rate} Hz')
        self.get_logger().info(f'Emergency Stop Topic: /emergency_stop')
        self.get_logger().info('='*80)
    
    # ========================================================================
    # Action Server 回调函数
    # ========================================================================
    
    def goal_callback(self, goal_request):
        """
        处理新的导航目标请求
        
        验证:
            - 目标位姿是否在地图坐标系 (frame_id = 'map')
            - 定位系统是否正常工作
            - 是否处于紧急停止状态
        
        返回:
            GoalResponse.ACCEPT 或 GoalResponse.REJECT
        """
        self.get_logger().info(f'收到新导航目标请求: Task ID = {goal_request.task_id}')
        
        # 检查 frame_id
        if goal_request.target_pose.header.frame_id != 'map':
            self.get_logger().error(
                f'目标坐标系错误: {goal_request.target_pose.header.frame_id} '
                f'(应为 "map")'
            )
            return GoalResponse.REJECT
        
        # 检查定位状态
        if self.current_pose is None:
            self.get_logger().error('拒绝目标: 未接收到定位数据 (/odometry/map)')
            return GoalResponse.REJECT
        
        # 检查紧急停止
        if self.emergency_stop_flag:
            self.get_logger().error('拒绝目标: 系统处于紧急停止状态')
            return GoalResponse.REJECT
        
        # 计算目标距离
        distance = self.calculate_distance(
            self.current_pose.pose.position,
            goal_request.target_pose.pose.position
        )
        
        self.get_logger().info(
            f'接受导航目标: ({goal_request.target_pose.pose.position.x:.2f}, '
            f'{goal_request.target_pose.pose.position.y:.2f}), '
            f'距离: {distance:.2f} m'
        )
        
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """
        处理任务取消请求
        
        场景:
            - MCB 主动取消任务
            - 新任务抢占当前任务
            - 用户手动停止
        
        返回:
            CancelResponse.ACCEPT
        """
        self.get_logger().info(f'收到任务取消请求: Task ID = {self.current_task_id}')
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """
        执行导航任务主逻辑
        
        流程:
            1. 初始化任务状态
            2. 发布目标到 A* 规划器
            3. 周期性监控导航状态
            4. 发送实时反馈
            5. 检测完成/失败条件
            6. 返回最终结果
        
        返回:
            PointToPoint.Result
        """
        self.get_logger().info('='*80)
        self.get_logger().info(f'开始执行导航任务: {goal_handle.request.task_id}')
        self.get_logger().info('='*80)
        
        # 🔧 保存目标引用 (用于退出时计算误差,避免被 deactivate_navigation 清空)
        goal_pose_for_error_calc = goal_handle.request.target_pose
        
        # ====================================================================
        # 1. 初始化任务状态
        # ====================================================================
        with self.state_lock:
            self.current_task_id = goal_handle.request.task_id
            self.current_goal = goal_handle.request.target_pose
            self.task_start_time = time.time()
            
            # 计算动态超时时间
            distance_to_goal = self.calculate_distance(
                self.current_pose.pose.position,
                self.current_goal.pose.position
            )
            self.task_timeout = min(
                self.base_timeout + distance_to_goal * self.timeout_per_meter,
                self.max_timeout
            )
            
            # 重置状态
            self.navigation_state = PointToPoint.Feedback.STATE_PLANNING
            self.current_path = None  # 🔧 清空旧路径
            self.path_length_remaining = 0.0
            self.position_history.clear()
            self.last_progress_time = time.time()
            self.recovery_start_time = None
        
        self.get_logger().info(f'目标距离: {distance_to_goal:.2f} m')
        self.get_logger().info(f'任务超时: {self.task_timeout:.1f} s')
        
        # ====================================================================
        # 2. 激活导航系统 (Master 控制 Slave)
        # ====================================================================
        self.activate_navigation()  # 🆕 开启主控制,启动心跳
        
        # ====================================================================
        # 3. 发布目标到 A* 规划器
        # ====================================================================
        goal_pose = goal_handle.request.target_pose
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(goal_pose)
        self.get_logger().info(
            f'已发布目标到 A* 规划器 - '
            f'frame: {goal_pose.header.frame_id}, '
            f'pos: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})'
        )
        
        # 等待 A* 规划器响应 (异步检查)
        self.get_logger().info('等待 A* 路径规划...')
        
        # ====================================================================
        # 3. 主监控循环
        # ====================================================================
        feedback_msg = PointToPoint.Feedback()
        result = PointToPoint.Result()
        
        rate = self.create_rate(self.feedback_rate)  # 5 Hz
        debug_logged = False  # 标志位,确保调试日志只显示一次
        planning_timeout = 10.0  # A* 规划超时 10 秒
        
        while rclpy.ok():
            # ----------------------------------------------------------------
            # 检查任务状态
            # ----------------------------------------------------------------
            
            # 检查取消请求
            if goal_handle.is_cancel_requested:
                self.deactivate_navigation()  # 🆕 停止主控制
                goal_handle.canceled()
                result.result = "failed"
                result.status_code = PointToPoint.Result.STATUS_PREEMPTED
                result.final_pose = self.current_pose
                result.position_error = self.calculate_distance(
                    self.current_pose.pose.position,
                    goal_pose_for_error_calc.pose.position
                )
                self.get_logger().info('任务已被取消')
                return result
            
            # 检查紧急停止
            if self.emergency_stop_flag:
                self.deactivate_navigation()  # 🆕 停止主控制
                goal_handle.abort()
                result.result = "failed"
                result.status_code = PointToPoint.Result.STATUS_EMERGENCY_STOP
                result.final_pose = self.current_pose
                result.position_error = self.calculate_distance(
                    self.current_pose.pose.position,
                    goal_pose_for_error_calc.pose.position
                )
                self.get_logger().error('紧急停止触发,任务中止')
                return result
            
            # 检查超时
            elapsed_time = time.time() - self.task_start_time
            if elapsed_time > self.task_timeout:
                # 🔧 超时前检查是否真的在移动
                if hasattr(self, 'position_history') and len(self.position_history) >= 2:
                    first_pos = self.position_history[0]
                    last_pos = self.position_history[-1]
                    total_moved = math.sqrt(
                        (last_pos['x'] - first_pos['x'])**2 +
                        (last_pos['y'] - first_pos['y'])**2
                    )
                    self.get_logger().error(
                        f'任务超时 ({elapsed_time:.1f}s > {self.task_timeout:.1f}s), '
                        f'总移动距离: {total_moved:.3f}m, '
                        f'当前距离目标: {self.calculate_distance(self.current_pose.pose.position, goal_pose_for_error_calc.pose.position):.3f}m'
                    )
                else:
                    self.get_logger().error(f'任务超时 ({elapsed_time:.1f}s > {self.task_timeout:.1f}s)')
                
                self.deactivate_navigation()  # 🆕 停止主控制
                goal_handle.abort()
                result.result = "failed"
                result.status_code = PointToPoint.Result.STATUS_TIMEOUT
                result.final_pose = self.current_pose
                result.position_error = self.calculate_distance(
                    self.current_pose.pose.position,
                    goal_pose_for_error_calc.pose.position
                )
                return result
            
            # 检查定位丢失
            if self.localization_confidence < self.localization_confidence_threshold:
                # 允许短暂丢失 (5 秒缓冲)
                if not hasattr(self, 'low_confidence_start_time'):
                    self.low_confidence_start_time = time.time()
                elif time.time() - self.low_confidence_start_time > 5.0:
                    self.deactivate_navigation()  # 🆕 停止主控制
                    goal_handle.abort()
                    result.result = "failed"
                    result.status_code = PointToPoint.Result.STATUS_LOCALIZATION_LOST
                    result.final_pose = self.current_pose
                    result.position_error = self.calculate_distance(
                        self.current_pose.pose.position,
                        goal_pose_for_error_calc.pose.position
                    )
                    self.get_logger().error(
                        f'定位丢失: 置信度 = {self.localization_confidence:.2f} '
                        f'< {self.localization_confidence_threshold:.2f}'
                    )
                    return result
            else:
                # 重置低置信度计时器
                if hasattr(self, 'low_confidence_start_time'):
                    delattr(self, 'low_confidence_start_time')
            
            # ----------------------------------------------------------------
            # 检查目标到达
            # ----------------------------------------------------------------
            current_distance = self.calculate_distance(
                self.current_pose.pose.position,
                self.current_goal.pose.position
            )
            
            # 🔧 改进: 每10次循环输出一次距离变化 (5Hz * 10 = 每2秒)
            if not hasattr(self, 'debug_counter'):
                self.debug_counter = 0
                self.last_distance = current_distance
            
            self.debug_counter += 1
            if self.debug_counter == 1 or self.debug_counter % 10 == 0:
                distance_change = self.last_distance - current_distance
                self.get_logger().info(
                    f'[进度] 当前位置: ({self.current_pose.pose.position.x:.3f}, '
                    f'{self.current_pose.pose.position.y:.3f}), '
                    f'目标: ({self.current_goal.pose.position.x:.3f}, '
                    f'{self.current_goal.pose.position.y:.3f}), '
                    f'剩余距离: {current_distance:.3f}m, '
                    f'变化: {distance_change:+.3f}m, '
                    f'速度: ({self.current_velocity.linear.x:.2f}, '
                    f'{self.current_velocity.angular.z:.2f})'
                )
                self.last_distance = current_distance
            
            if current_distance < self.goal_tolerance:
                # 额外检查: 速度是否足够小 (< 0.05 m/s)
                current_speed = math.sqrt(
                    self.current_velocity.linear.x**2 +
                    self.current_velocity.linear.y**2
                )
                
                if current_speed < 0.05:
                    self.deactivate_navigation()  # 🆕 停止主控制
                    goal_handle.succeed()
                    result.result = "success"
                    result.status_code = PointToPoint.Result.STATUS_SUCCESS
                    result.final_pose = self.current_pose
                    result.position_error = current_distance
                    self.get_logger().info('='*80)
                    self.get_logger().info('🎯 成功到达目标!')
                    self.get_logger().info(f'位置误差: {current_distance:.3f} m')
                    self.get_logger().info(f'耗时: {elapsed_time:.1f} s')
                    self.get_logger().info('='*80)
                    return result
            
            # ----------------------------------------------------------------
            # 检查路径规划超时
            # ----------------------------------------------------------------
            if self.navigation_state == PointToPoint.Feedback.STATE_PLANNING:
                if elapsed_time > planning_timeout:
                    self.deactivate_navigation()  # 🆕 停止主控制
                    goal_handle.abort()
                    result.result = "failed"
                    result.status_code = PointToPoint.Result.STATUS_GOAL_UNREACHABLE
                    result.final_pose = self.current_pose
                    result.position_error = current_distance
                    self.get_logger().error(
                        f'路径规划超时 ({elapsed_time:.1f}s > {planning_timeout:.1f}s), '
                        f'可能原因: A* 规划器未运行或目标不可达'
                    )
                    return result
            
            # ----------------------------------------------------------------
            # 更新导航状态
            # ----------------------------------------------------------------
            self.update_navigation_state(current_distance)
            
            # ----------------------------------------------------------------
            # 检测卡住状态
            # ----------------------------------------------------------------
            if self.is_stuck():
                # 尝试恢复行为
                if self.recovery_start_time is None:
                    self.recovery_start_time = time.time()
                    self.navigation_state = PointToPoint.Feedback.STATE_RECOVERING
                    self.get_logger().warn('检测到卡住,启动恢复行为')
                elif time.time() - self.recovery_start_time > self.recovery_timeout:
                    # 恢复失败
                    self.deactivate_navigation()  # 🆕 停止主控制
                    goal_handle.abort()
                    result.result = "failed"
                    result.status_code = PointToPoint.Result.STATUS_PATH_BLOCKED
                    result.final_pose = self.current_pose
                    result.position_error = current_distance
                    self.get_logger().error('恢复行为超时,判定为路径阻塞')
                    return result
            else:
                # 重置恢复计时器
                if self.recovery_start_time is not None:
                    self.recovery_start_time = None
                    self.get_logger().info('恢复成功,继续导航')
            
            # ----------------------------------------------------------------
            # 构建并发送反馈
            # ----------------------------------------------------------------
            feedback_msg = self.build_feedback(current_distance, elapsed_time)
            goal_handle.publish_feedback(feedback_msg)
            
            # ----------------------------------------------------------------
            # 休眠到下一周期
            # ----------------------------------------------------------------
            rate.sleep()
        
        # 不应到达这里 (循环被 rclpy.ok() 中断)
        self.deactivate_navigation()  # 🆕 停止主控制 (包含状态清理)
        
        goal_handle.abort()
        result.result = "failed"
        result.status_code = PointToPoint.Result.STATUS_EMERGENCY_STOP
        result.final_pose = self.current_pose
        result.position_error = self.calculate_distance(
            self.current_pose.pose.position,
            goal_pose_for_error_calc.pose.position
        )
        return result
    
    # ========================================================================
    # 话题订阅回调函数
    # ========================================================================
    
    def odometry_callback(self, msg):
        """更新机器人当前位姿和速度"""
        with self.state_lock:
            # 将 Odometry 转换为 PoseStamped 格式
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header  # 保留时间戳和坐标系
            pose_stamped.pose = msg.pose.pose  # 提取 Pose (不含协方差)
            
            self.current_pose = pose_stamped
            self.current_velocity = msg.twist.twist
            
            # 记录位置历史 (用于卡住检测)
            if self.current_task_id is not None:
                self.position_history.append({
                    'time': time.time(),
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y
                })
    
    def path_callback(self, msg):
        """更新 A* 全局路径"""
        with self.state_lock:
            # 🔧 调试: 记录当前状态径回调之间存在竞态,可能导致状态不更新
            state_names = {
                PointToPoint.Feedback.STATE_PLANNING: 'STATE_PLANNING',
                PointToPoint.Feedback.STATE_FOLLOWING: 'STATE_FOLLOWING',
                PointToPoint.Feedback.STATE_AVOIDING: 'STATE_AVOIDING',
                PointToPoint.Feedback.STATE_RECOVERING: 'STATE_RECOVERING',
                PointToPoint.Feedback.STATE_STUCK: 'STATE_STUCK'
            }
            current_state_name = state_names.get(self.navigation_state, 'UNKNOWN')
            
            self.get_logger().info(
                f'📍 收到 A* 路径,包含 {len(msg.poses)} 个路点 '
                f'(当前状态: {current_state_name}, 任务ID: {self.current_task_id})'
            )
            
            self.current_path = msg
            
            # 计算路径总长度
            if len(msg.poses) > 1:
                path_length = 0.0
                for i in range(len(msg.poses) - 1):
                    path_length += self.calculate_distance(
                        msg.poses[i].pose.position,
                        msg.poses[i+1].pose.position
                    )
                self.path_length_remaining = path_length
            else:
                self.path_length_remaining = 0.0
            
            # 🔧 修复: 无论当前状态如何,收到路径就切换到跟踪状态
            # (避免时序问题导致状态不更新)
            if self.current_task_id is not None:  # 确保有活动任务
                old_state = self.navigation_state
                self.navigation_state = PointToPoint.Feedback.STATE_FOLLOWING
                self.get_logger().info(
                    f'✅ 路径规划完成,路径长度: {self.path_length_remaining:.2f} m, '
                    f'路点数: {len(msg.poses)} (状态切换: {state_names.get(old_state, "UNKNOWN")} -> STATE_FOLLOWING)'
                )
    
    def obstacles_callback(self, msg):
        """更新障碍物接近度"""
        with self.state_lock:
            # 从点云中提取最近障碍物距离
            min_distance = float('inf')
            
            if self.current_pose is not None:
                robot_x = self.current_pose.pose.position.x
                robot_y = self.current_pose.pose.position.y
                
                # 解析点云数据
                for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                    dist = math.sqrt((point[0] - robot_x)**2 + (point[1] - robot_y)**2)
                    min_distance = min(min_distance, dist)
            
            # 更新接近度等级
            if min_distance < self.obstacle_critical_distance:
                self.obstacle_proximity = PointToPoint.Feedback.OBSTACLE_CRITICAL
                if self.navigation_state == PointToPoint.Feedback.STATE_FOLLOWING:
                    self.navigation_state = PointToPoint.Feedback.STATE_AVOIDING
            elif min_distance < self.obstacle_near_distance:
                self.obstacle_proximity = PointToPoint.Feedback.OBSTACLE_NEAR
                if self.navigation_state == PointToPoint.Feedback.STATE_FOLLOWING:
                    self.navigation_state = PointToPoint.Feedback.STATE_AVOIDING
            else:
                self.obstacle_proximity = PointToPoint.Feedback.OBSTACLE_CLEAR
                # 如果之前在避障,现在可以恢复跟踪
                if self.navigation_state == PointToPoint.Feedback.STATE_AVOIDING:
                    self.navigation_state = PointToPoint.Feedback.STATE_FOLLOWING
    
    def confidence_callback(self, msg):
        """更新定位置信度"""
        with self.state_lock:
            self.localization_confidence = msg.data
            
            # 限流警告: 每 2 秒最多警告一次
            if msg.data < self.localization_confidence_threshold:
                current_time = time.time()
                if current_time - self.last_confidence_warning_time > self.confidence_warning_interval:
                    self.get_logger().warn(
                        f'定位置信度低: {msg.data:.2f} < {self.localization_confidence_threshold:.2f}'
                    )
                    self.last_confidence_warning_time = current_time
    
    def emergency_stop_callback(self, msg):
        """处理紧急停止信号"""
        if msg.data and not self.emergency_stop_flag:
            self.emergency_stop_flag = True
            self.get_logger().error('🚨 紧急停止触发!')
        elif not msg.data and self.emergency_stop_flag:
            self.emergency_stop_flag = False
            self.get_logger().info('✅ 紧急停止解除')
    
    # ========================================================================
    # 辅助函数
    # ========================================================================
    
    def activate_navigation(self):
        """
        激活导航系统 (通知 Position Control 开始执行)
        
        功能:
            - 发布 active=True 信号
            - 启动心跳定时器 (持续发布,确保 Position Control 知道任务进行中)
        """
        msg = Bool()
        msg.data = True
        self.nav_active_pub.publish(msg)
        
        # 启动心跳定时器 (每 0.5 秒发布一次)
        if self.heartbeat_timer is None:
            self.heartbeat_timer = self.create_timer(
                0.5,  # 2Hz
                self.publish_active_heartbeat
            )
        
        self.get_logger().info("✅ 导航系统已激活")
    
    def deactivate_navigation(self):
        """
        停止导航系统 (通知 Position Control 停止执行)
        
        功能:
            - 发布 active=False 信号
            - 停止心跳定时器
            - Position Control 将立即停止机器人并清除目标
            - 清理任务状态 (避免影响下一个任务)
        """
        msg = Bool()
        msg.data = False
        self.nav_active_pub.publish(msg)
        
        # 停止心跳定时器
        if self.heartbeat_timer is not None:
            self.heartbeat_timer.cancel()
            self.heartbeat_timer = None
        
        # 🔧 清理任务状态 (确保下一个任务不会被旧状态影响)
        with self.state_lock:
            self.current_task_id = None
            self.current_goal = None
            self.current_path = None
            self.path_length_remaining = 0.0
            self.navigation_state = PointToPoint.Feedback.STATE_PLANNING
            self.position_history.clear()
        
        self.get_logger().info("🛑 导航系统已停止")
    
    def publish_active_heartbeat(self):
        """心跳: 持续发布激活信号 (任务执行期间)"""
        msg = Bool()
        msg.data = True
        self.nav_active_pub.publish(msg)
    
    def calculate_distance(self, pos1, pos2):
        """计算两点间的欧几里得距离 (2D)"""
        return math.sqrt(
            (pos1.x - pos2.x)**2 +
            (pos1.y - pos2.y)**2
        )
    
    def update_navigation_state(self, distance_to_goal):
        """
        根据当前情况更新导航状态
        
        逻辑:
            - 如果有障碍物接近 → AVOIDING
            - 如果检测到卡住 → RECOVERING/STUCK
            - 否则 → FOLLOWING
        """
        with self.state_lock:
            # 障碍物优先级最高
            if self.obstacle_proximity != PointToPoint.Feedback.OBSTACLE_CLEAR:
                if self.navigation_state != PointToPoint.Feedback.STATE_RECOVERING:
                    self.navigation_state = PointToPoint.Feedback.STATE_AVOIDING
            # 检查是否卡住
            elif self.is_stuck():
                if self.recovery_start_time is not None:
                    self.navigation_state = PointToPoint.Feedback.STATE_RECOVERING
                else:
                    self.navigation_state = PointToPoint.Feedback.STATE_STUCK
            # 正常跟踪
            else:
                if self.current_path is not None:
                    self.navigation_state = PointToPoint.Feedback.STATE_FOLLOWING
    
    def is_stuck(self):
        """
        检测机器人是否卡住
        
        判断依据:
            - 最近 5 秒内移动距离 < 0.1m
            - 且速度指令非零 (排除正常停止)
        
        返回:
            bool: True 表示卡住
        """
        if len(self.position_history) < 10:  # 至少 1 秒数据
            return False
        
        # 计算 stuck_timeout 秒内的移动距离
        current_time = time.time()
        old_positions = [
            pos for pos in self.position_history
            if current_time - pos['time'] >= self.stuck_timeout
        ]
        
        if not old_positions:
            return False
        
        old_pos = old_positions[-1]
        current_pos = self.position_history[-1]
        
        distance_moved = math.sqrt(
            (current_pos['x'] - old_pos['x'])**2 +
            (current_pos['y'] - old_pos['y'])**2
        )
        
        # 检查是否有速度指令 (排除正常停止)
        has_velocity_command = (
            abs(self.current_velocity.linear.x) > 0.01 or
            abs(self.current_velocity.angular.z) > 0.01
        )
        
        return distance_moved < self.stuck_distance_threshold and has_velocity_command
    
    def build_feedback(self, distance_to_goal, elapsed_time):
        """
        构建反馈消息
        
        参数:
            distance_to_goal: 到目标的直线距离
            elapsed_time: 任务已执行时间
        
        返回:
            PointToPoint.Feedback
        """
        feedback = PointToPoint.Feedback()
        
        with self.state_lock:
            # 当前位姿和速度
            if self.current_pose is not None:
                feedback.current_pose = self.current_pose
            else:
                # 如果没有位姿数据,创建一个空的 PoseStamped
                feedback.current_pose = PoseStamped()
                feedback.current_pose.header.frame_id = "map"
                
            feedback.current_velocity = self.current_velocity
            
            # 导航状态
            feedback.navigation_state = self.navigation_state
            feedback.obstacle_proximity = self.obstacle_proximity
            
            # 进度信息
            feedback.distance_to_goal = distance_to_goal
            feedback.path_length_remaining = self.path_length_remaining if self.current_path else distance_to_goal
            
            # 估算剩余时间 (平均速度法)
            avg_speed = 0.2  # 保守估计 0.2 m/s
            if abs(self.current_velocity.linear.x) > 0.05:
                avg_speed = abs(self.current_velocity.linear.x)
            feedback.estimated_time_remaining = feedback.path_length_remaining / avg_speed
            
            # 定位置信度
            feedback.localization_confidence = self.localization_confidence
            
            # 调试信息
            state_names = {
                PointToPoint.Feedback.STATE_PLANNING: '规划中',
                PointToPoint.Feedback.STATE_FOLLOWING: '路径跟踪',
                PointToPoint.Feedback.STATE_AVOIDING: '避障中',
                PointToPoint.Feedback.STATE_RECOVERING: '恢复中',
                PointToPoint.Feedback.STATE_STUCK: '可能卡住'
            }
            
            feedback.debug_message = (
                f'[{state_names.get(self.navigation_state, "未知")}] '
                f'剩余: {distance_to_goal:.2f}m, '
                f'置信度: {self.localization_confidence:.2f}, '
                f'耗时: {elapsed_time:.1f}s'
            )
        
        return feedback


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    action_server = None
    executor = None
    
    try:
        # 创建节点
        action_server = PointToPointActionServer()
        
        # 使用多线程执行器 (支持 Action 并发)
        executor = MultiThreadedExecutor()
        executor.add_node(action_server)
        
        action_server.get_logger().info('PointToPoint Action Server 运行中...')
        executor.spin()
        
    except KeyboardInterrupt:
        if action_server:
            action_server.get_logger().info('收到 Ctrl+C, 正在关闭...')
    except Exception as e:
        if action_server:
            action_server.get_logger().error(f'执行过程中发生错误: {e}')
    finally:
        # 安全关闭 - 不抛出异常
        if executor:
            try:
                executor.shutdown()
            except Exception:
                pass
        
        if action_server:
            try:
                action_server.destroy_node()
            except Exception:
                pass
        
        # 最后关闭 rclpy (检查 context 是否仍然有效)
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # 忽略已经关闭的情况


if __name__ == '__main__':
    main()
