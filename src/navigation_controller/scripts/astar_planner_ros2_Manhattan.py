#!/usr/bin/env python3
"""
A* Path Planner for ROS2
Author: shenlanxueyuan
Email: 1347893477@qq.com
Organization: Shen Lan xue yuan
Date: 2025-10-14
你这结婚的都买上了，怕个啥？
This node implements an A* path planning algorithm with:
- Dynamic obstacle avoidance using inflated map
- Path smoothing and optimization
- Path visualization and saving
- Integration with ROS2 navigation stack

Migration Notes (ROS1 -> ROS2):
- Changed from rospy to rclpy
- Updated node initialization pattern
- Changed callback groups for multi-threaded execution
- Updated QoS profiles for reliable communication
- Modified TF2 buffer initialization
- Changed timer callback signature
- Updated logging methods
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
#from tf_transformations import euler_from_quaternion
import heapq
import math
from scipy import ndimage
import os
import time
import csv


class AStarPlanner(Node):
    def __init__(self):
        # 初始化ROS2节点
        super().__init__('astar_planner')
        
        # 创建回调组
        self.callback_group1 = MutuallyExclusiveCallbackGroup()
        self.callback_group2 = MutuallyExclusiveCallbackGroup()
        
        # 参数
        self.map_data = None
        self.inflated_map = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin_x = 0
        self.map_origin_y = 0
        # A*改进: 调整安全边距参数 - 总距离从1.0m降至0.6m
        self.robot_radius = 0.35  # 机器人半径（米）- G1实际宽度+小量余量
        # self.inflation_radius_m = 0.25  # 膨胀半径（米）- 减小障碍物膨胀
        self.inflation_radius_m = 0.15  # 膨胀半径（米）- 进一步减小障碍物膨胀
        # ========================================================================
        # G1单轴移动机器人参数：每次只能沿一个坐标轴移动
        # 运动约束：
        #   - 前/后移动: vx ≠ 0, vy = 0, angular.z = 0
        #   - 左/右移动: vx = 0, vy ≠ 0, angular.z = 0
        #   - 禁止斜向移动（vx和vy不能同时非零）
        # ========================================================================
        self.max_linear_speed = 0.5  # 最大线性速度（米/秒）
        self.waypoint_tolerance = 0.15  # 路径点到达容差（米）
        self.direction_switch_penalty = 0.3  # 方向切换惩罚（鼓励连续同方向移动）
        
        # 起点和终点
        self.start_x = 0
        self.start_y = 0
        self.goal_x = 0
        self.goal_y = 0
        self.start_set = False  # 添加标志，表示是否设置了起点
        self.is_planning = False  # 添加标志，表示是否正在规划路径  
        
        # ========================================================================
        # G1机器人改进：添加目标点缓存机制
        # 解决时序问题：如果目标点到达时起点还未设置，缓存目标点等待重试
        # ========================================================================
        self.pending_goal = None  # 缓存的目标点
        self.goal_received_time = None  # 目标点接收时间
        
        # 路径
        self.path = None
        
        # 添加TF缓存区 (ROS2 uses Buffer instead of tf2_ros.Buffer())
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # G1人形机器人改进：添加当前朝向跟踪
        self.current_heading = None
        self.current_pose = None  # 保存完整的位姿信息
        
        # QoS配置 - 为不同类型的消息使用不同的QoS配置
        # 地图使用持久性QoS，确保后加入的节点也能收到
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 传感器数据使用Best Effort QoS，提高性能
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 默认QoS
        default_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ROS订阅和发布
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.map_callback, 
            map_qos,
            callback_group=self.callback_group1
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',  # ROS2中通常改为 /goal_pose
            self.goal_callback,
            default_qos,
            callback_group=self.callback_group1
        )
        
        # 使用融合定位替代手动设置的初始位置
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/map',
            self.odom_callback,
            sensor_qos,
            callback_group=self.callback_group2
        )
        
        # 保留现有的发布者
        self.path_pub = self.create_publisher(Path, '/astar_path', default_qos)
        self.inflated_map_pub = self.create_publisher(OccupancyGrid, '/inflated_map', map_qos)
        
        # 创建定时器用于周期性发布膨胀地图
        self.map_publish_timer = None
        
        # ========================================================================
        # G1机器人改进：创建定时器用于重试待处理的目标点
        # 每0.5秒检查一次是否可以处理缓存的目标点
        # ========================================================================
        self.pending_goal_timer = self.create_timer(
            0.5,  # 500ms 检查一次
            self.check_pending_goal_callback
        )
        
        self.get_logger().info('A* Planner initialized successfully')
        self.get_logger().info('等待地图数据 (/map)...')
        self.get_logger().info('等待机器人位置 (/odometry/map)...')
        self.get_logger().info('等待目标点 (/move_base_simple/goal)...')
        
    def check_pending_goal_callback(self):
        """
        定时检查是否有待处理的目标点
        如果起点已设置且有缓存的目标点，则尝试规划路径
        """
        if self.pending_goal is not None and self.start_set and not self.is_planning:
            self.get_logger().info('检测到待处理的目标点，现在起点已就绪，开始规划路径...')
            
            # 提取缓存的目标点数据
            goal_x, goal_y = self.world_to_map(
                self.pending_goal.pose.position.x, 
                self.pending_goal.pose.position.y
            )
            self.goal_x, self.goal_y = goal_x, goal_y
            
            # 清除缓存
            self.pending_goal = None
            self.goal_received_time = None
            
            # 规划路径
            self.is_planning = True
            self.plan_path()
    
        
    def map_callback(self, data):
        """接收地图数据"""
        self.map_data = np.array(data.data).reshape((data.info.height, data.info.width))
        self.map_width = data.info.width
        self.map_height = data.info.height
        self.map_resolution = data.info.resolution
        self.map_origin_x = data.info.origin.position.x
        self.map_origin_y = data.info.origin.position.y
        
        # 地图加载成功提示
        self.get_logger().info(
            f"✓ 地图数据已加载: {self.map_width}x{self.map_height} "
            f"分辨率={self.map_resolution:.3f}m/cell"
        )
        
        # 创建膨胀地图
        self.create_inflated_map(data)
    
    def odom_callback(self, data):
        """从融合定位获取机器人当前位置作为起点"""
        # 保存完整位姿信息
        self.current_pose = data
        
        # 更新当前朝向（用于改进的启发式函数）
        self.update_current_heading()
        
        if self.map_data is None:
            self.get_logger().warn("尚未接收到地图数据，无法设置起始位置", throttle_duration_sec=1.0)
            return
            
        try:
            # 获取当前位置
            curr_x = data.pose.pose.position.x
            curr_y = data.pose.pose.position.y
            
            # 检查frame_id和child_frame_id并记录
            source_frame = data.header.frame_id
            child_frame = data.child_frame_id
            
            # 将世界坐标转换为地图坐标
            start_x, start_y = self.world_to_map(curr_x, curr_y)
            self.start_x, self.start_y = start_x, start_y
            
            # ====================================================================
            # G1机器人改进：首次设置起点时给出明确提示和详细诊断
            # ====================================================================
            if not self.start_set:
                self.get_logger().info(f"✓ 机器人位置已就绪: 坐标系 {source_frame}->{child_frame}")
                self.get_logger().info(f"✓ 当前位置: 世界({curr_x:.2f}, {curr_y:.2f}), 地图({start_x}, {start_y})")
                self.get_logger().info(f"  地图信息: 大小={self.map_width}x{self.map_height}, 原点=({self.map_origin_x:.2f}, {self.map_origin_y:.2f}), 分辨率={self.map_resolution:.3f}")
                
                # 检查起点是否有效
                if start_x < 0 or start_x >= self.map_width or start_y < 0 or start_y >= self.map_height:
                    self.get_logger().warn(f"⚠️  起点 ({start_x}, {start_y}) 超出地图范围！")
                    self.get_logger().warn(f"   地图范围: X[0, {self.map_width}), Y[0, {self.map_height})")
                elif self.inflated_map is not None and self.inflated_map[start_y][start_x] >= 50:
                    occupancy_value = self.inflated_map[start_y][start_x]
                    self.get_logger().warn(f"⚠️  起点 ({start_x}, {start_y}) 在障碍物中！占用值={occupancy_value}")
                else:
                    self.get_logger().info("✓ 起点位置有效，A* 规划器准备完毕，可以设置目标点了！")
            
            self.start_set = True
            
            # 调试日志（节流输出）
            self.get_logger().debug(
                f"当前机器人位置: 世界坐标({curr_x:.2f}, {curr_y:.2f}), "
                f"地图坐标({start_x}, {start_y})",
                throttle_duration_sec=2.0
            )
        except Exception as e:
            self.get_logger().error(f"处理里程计数据时出错: {e}")
    
    def create_inflated_map(self, map_msg):
        """创建膨胀地图"""
        # 计算膨胀半径（像素）
        inflation_pixels = int(self.inflation_radius_m / self.map_resolution)
        
        try:
            # 创建二值障碍物地图
            binary_obstacle_map = np.zeros_like(self.map_data)
            binary_obstacle_map[self.map_data >= 50] = 1  # 障碍物为1，自由空间为0
            
            # 使用距离变换来膨胀障碍物
            distance_from_obstacles = ndimage.distance_transform_edt(1 - binary_obstacle_map) * self.map_resolution
            
            # 创建膨胀地图
            self.inflated_map = np.copy(self.map_data)
            # 标记膨胀区域（距离障碍物小于膨胀半径的区域）
            inflation_mask = (distance_from_obstacles <= self.inflation_radius_m) & (self.map_data < 50)
            self.inflated_map[inflation_mask] = 99  # 将膨胀区域标记为99（接近障碍物）
            
            # 发布膨胀地图以便在RViz中可视化
            inflated_map_msg = OccupancyGrid()
            inflated_map_msg.header.stamp = self.get_clock().now().to_msg()
            inflated_map_msg.header.frame_id = "map"
            
            # 完整复制地图元数据，确保所有必要字段都设置
            inflated_map_msg.info = map_msg.info
            
            # 确保地图数据正确格式化
            flat_data = self.inflated_map.flatten()
            inflated_map_msg.data = [int(cell) for cell in flat_data]
            
            # 发布膨胀地图
            self.inflated_map_pub.publish(inflated_map_msg)
            
            # 取消之前的定时器（如果存在）
            if self.map_publish_timer:
                self.map_publish_timer.cancel()
            
            # 创建新的定时器用于周期性发布
            self.map_publish_timer = self.create_timer(
                2.0,  # 每2秒发布一次
                self.publish_inflated_map_callback
            )
            
        except Exception as e:
            self.get_logger().error(f"创建膨胀地图时出错: {e}")
    
    def publish_inflated_map_callback(self):
        """定时回调，周期性发布膨胀地图"""
        if self.inflated_map is not None and self.map_data is not None:
            try:
                inflated_map_msg = OccupancyGrid()
                inflated_map_msg.header.stamp = self.get_clock().now().to_msg()
                inflated_map_msg.header.frame_id = "map"
                
                # 正确设置地图信息
                inflated_map_msg.info.resolution = self.map_resolution
                inflated_map_msg.info.width = self.map_width
                inflated_map_msg.info.height = self.map_height
                inflated_map_msg.info.origin.position.x = self.map_origin_x
                inflated_map_msg.info.origin.position.y = self.map_origin_y
                inflated_map_msg.info.origin.orientation.w = 1.0
                
                # 确保地图数据正确格式化和转换
                flat_data = self.inflated_map.flatten()
                inflated_map_msg.data = [int(cell) for cell in flat_data]
                
                # 发布消息
                self.inflated_map_pub.publish(inflated_map_msg)
            except Exception as e:
                self.get_logger().error(f"发布膨胀地图时出错: {e}")
        
    def goal_callback(self, data):
        """接收目标点"""
        if self.map_data is None:
            self.get_logger().warn("尚未接收到地图数据，无法规划路径")
            return
        
        self.get_logger().info(f'收到目标点: ({data.pose.position.x:.2f}, {data.pose.position.y:.2f})')
            
        # 将世界坐标转换为地图坐标
        goal_x, goal_y = self.world_to_map(data.pose.position.x, data.pose.position.y)
        self.goal_x, self.goal_y = goal_x, goal_y
        
        # 如果起点已设置（通过odom_callback），则立即规划路径
        if self.start_set:
            self.get_logger().info(f'起点已就绪，开始规划从({self.start_x}, {self.start_y})到({self.goal_x}, {self.goal_y})的路径')
            self.is_planning = True 
            self.plan_path()
        else:
            # ====================================================================
            # G1机器人改进：起点未就绪时缓存目标点
            # 这解决了时序问题：用户在 locate_node 开始发布 /odometry/map 之前
            # 就设置了目标点的情况
            # ====================================================================
            self.get_logger().warn("尚未接收到有效的机器人位置 (/odometry/map)")
            self.get_logger().warn("目标点已缓存，等待定位系统就绪后自动规划...")
            self.get_logger().info("提示：请确保已在 RViz 中设置 '2D Pose Estimate' 并等待2-3秒")
            
            # 缓存目标点
            self.pending_goal = data
            self.goal_received_time = self.get_clock().now()
    
    
    def world_to_map(self, world_x, world_y):
        """将世界坐标转换为地图坐标"""
        x = int((world_x - self.map_origin_x) / self.map_resolution)
        y = int((world_y - self.map_origin_y) / self.map_resolution)
        return x, y
    
    def map_to_world(self, map_x, map_y):
        """将地图坐标转换为世界坐标"""
        world_x = map_x * self.map_resolution + self.map_origin_x
        world_y = map_y * self.map_resolution + self.map_origin_y
        return world_x, world_y
    
    def is_valid(self, x, y):
        """检查坐标是否有效，考虑膨胀地图"""
        if x < 0 or x >= self.map_width or y < 0 or y >= self.map_height:
            return False
            
        # 使用膨胀地图进行检查，避开障碍物和膨胀区域
        if self.inflated_map is not None:
            return self.inflated_map[y][x] < 50  # 小于50视为可通行
        else:
            return self.map_data[y][x] < 50
    
    def get_neighbors(self, x, y):
        """获取邻居节点（仅4方向，无对角线）"""
        neighbors = []
        # G1单轴移动：只允许上下左右4个方向，禁止对角线
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            nx, ny = x + dx, y + dy
            if self.is_valid(nx, ny):
                # 所有移动代价相同
                cost = 1.0
                neighbors.append((nx, ny, cost))
        return neighbors
    
    def heuristic(self, x, y):
        """
        G1单轴移动机器人启发式函数：使用曼哈顿距离
        
        G1机器人运动约束：
        1. 只能沿坐标轴移动（前、后、左、右）
        2. 不能斜向移动（vx和vy不能同时非零）
        3. 每次移动必须选择X轴或Y轴之一
        
        对于这种网格式移动，曼哈顿距离是最优下界估计
        （因为无法抄近路，必须分别走完X和Y方向的距离）
        """
        dx = abs(self.goal_x - x)
        dy = abs(self.goal_y - y)
        
        # 曼哈顿距离（L1范数）
        return dx + dy
    
    def update_current_heading(self):
        """
        从里程计更新当前朝向
        
        用于改进的启发式函数计算
        """
        if self.current_pose is None:
            return
        
        try:
            q = self.current_pose.pose.pose.orientation
            # 从四元数计算yaw角
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            self.current_heading = yaw
        except Exception as e:
            self.get_logger().warn(f"无法更新朝向: {e}")
    
    def heuristic_original(self, x, y):
        """原始启发式函数：使用欧几里得距离（保留作为对比）"""
        goal_x, goal_y = self.goal_x, self.goal_y
        return math.sqrt((x - goal_x) ** 2 + (y - goal_y) ** 2)
    
    def plan_path(self):
        """使用A*算法规划路径"""
        if self.map_data is None:
            self.get_logger().warn("尚未接收到地图数据，无法规划路径")
            return
        
        # ====================================================================
        # 改进的起点和终点有效性检查，提供详细的诊断信息
        # ====================================================================
        
        # 检查起点是否有效
        if not self.is_valid(self.start_x, self.start_y):
            self.get_logger().error(f"❌ 起点({self.start_x}, {self.start_y})无效或处于障碍物中！")
            
            # 详细诊断信息
            if self.start_x < 0 or self.start_x >= self.map_width or self.start_y < 0 or self.start_y >= self.map_height:
                self.get_logger().error(f"   原因：起点超出地图范围")
                self.get_logger().error(f"   地图范围: X[0, {self.map_width}), Y[0, {self.map_height})")
                self.get_logger().error(f"   当前起点: X={self.start_x}, Y={self.start_y}")
            else:
                occupancy = self.inflated_map[self.start_y][self.start_x] if self.inflated_map is not None else self.map_data[self.start_y][self.start_x]
                self.get_logger().error(f"   原因：起点在障碍物中，占用值={occupancy} (>=50为障碍)")
            
            # 提供当前位置的世界坐标（反推）
            if self.current_pose is not None:
                world_x = self.current_pose.pose.pose.position.x
                world_y = self.current_pose.pose.pose.position.y
                self.get_logger().error(f"   当前世界坐标: ({world_x:.3f}, {world_y:.3f})")
                self.get_logger().error(f"   地图原点: ({self.map_origin_x:.3f}, {self.map_origin_y:.3f})")
                self.get_logger().error(f"   地图分辨率: {self.map_resolution:.3f} m/cell")
            
            self.get_logger().error("   解决方法：")
            self.get_logger().error("   1. 检查 locate_node 是否正确发布位置到 /odometry/map")
            self.get_logger().error("   2. 确认 2D Pose Estimate 设置在地图的有效区域")
            self.get_logger().error("   3. 检查地图原点和分辨率是否正确")
            return
            
        if not self.is_valid(self.goal_x, self.goal_y):
            self.get_logger().error(f"❌ 终点({self.goal_x}, {self.goal_y})无效或处于障碍物中！")
            
            # 详细诊断信息
            if self.goal_x < 0 or self.goal_x >= self.map_width or self.goal_y < 0 or self.goal_y >= self.map_height:
                self.get_logger().error(f"   原因：终点超出地图范围")
                self.get_logger().error(f"   地图范围: X[0, {self.map_width}), Y[0, {self.map_height})")
            else:
                occupancy = self.inflated_map[self.goal_y][self.goal_x] if self.inflated_map is not None else self.map_data[self.goal_y][self.goal_x]
                self.get_logger().error(f"   原因：终点在障碍物中，占用值={occupancy}")
            
            self.get_logger().error("   请在 RViz 中选择另一个目标位置（远离障碍物）")
            return
        
        self.get_logger().info(f"开始 A* 路径规划：起点({self.start_x}, {self.start_y}) → 终点({self.goal_x}, {self.goal_y})")
        
        # A*算法
        open_set = []
        closed_set = set()
        came_from = {}
        tmp_start_x = self.start_x
        tmp_start_y = self.start_y
        g_score = {(tmp_start_x, tmp_start_y): 0}
        f_score = {(tmp_start_x, tmp_start_y): self.heuristic(tmp_start_x, tmp_start_y)}
        
        heapq.heappush(open_set, (f_score[(tmp_start_x, tmp_start_y)], tmp_start_x, tmp_start_y))
        
        while open_set:
            _, current_x, current_y = heapq.heappop(open_set)
            current = (current_x, current_y)
            
            if current_x == self.goal_x and current_y == self.goal_y:
                # 构建路径
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append((tmp_start_x, tmp_start_y))
                path.reverse()
                
                # ============================================================
                # G1人形机器人改进：应用路径平滑
                # ============================================================
                self.get_logger().info(f"✓ A* 找到路径！原始路径点数: {len(path)}")
                self.get_logger().info("  正在平滑处理...")
                smoothed_path = self.smooth_path_for_g1(path)
                
                # 转换为世界坐标并发布
                self.publish_path(smoothed_path)
                
                # 计算路径长度
                path_length = self.calculate_path_length(smoothed_path)
                self.get_logger().info(f"✓ 路径规划成功！")
                self.get_logger().info(f"  - 平滑后路径点数: {len(smoothed_path)}")
                self.get_logger().info(f"  - 路径总长度: {path_length:.2f} 米")
                self.get_logger().info(f"  - 已发布到 /astar_path 话题")
                
                # 重置规划标志
                self.is_planning = False
                return
            
            closed_set.add(current)
            
            for neighbor_x, neighbor_y, cost in self.get_neighbors(current_x, current_y):
                # 处理邻居节点
                neighbor = (neighbor_x, neighbor_y)
                if neighbor in closed_set:
                    continue  # 已访问，跳过

                tentative_g_score = g_score[current] + cost  # 计算到邻居的g值

                # 如果邻居未被访问过，或找到更优路径
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current  # 记录父节点
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor_x, neighbor_y)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor_x, neighbor_y))  # 加入open_set
        
        # 如果没有找到路径
        self.get_logger().error("❌ 无法找到从起点到终点的路径！")
        self.get_logger().error("   可能原因：")
        self.get_logger().error("   1. 起点或终点被障碍物包围")
        self.get_logger().error("   2. 两点之间被障碍物完全阻隔")
        self.get_logger().error("   3. 膨胀半径设置过大")
        self.get_logger().info("   建议：在 RViz 中选择另一个目标位置")
        self.is_planning = False
    
    def calculate_path_length(self, path):
        """计算路径总长度（米）"""
        if len(path) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            dx = x2 - x1
            dy = y2 - y1
            total_length += math.sqrt(dx*dx + dy*dy) * self.map_resolution
        
        return total_length
    
    def smooth_path_for_g1(self, path):
        """
        为G1人形机器人平滑路径
        
        使用三次样条插值方法，确保：
        1. 路径曲率连续，避免急转弯
        2. 路径点数量适中，不过密也不过疏
        3. 保持与障碍物的安全距离
        
        Args:
            path: 原始路径点列表 [(x1, y1), (x2, y2), ...]
        
        Returns:
            smoothed_path: 平滑后的路径点列表
        """
        if len(path) < 3:
            return path  # 路径太短，无需平滑
        
        # ====================================================================
        # 步骤1：路径简化 - 移除冗余点
        # ====================================================================
        simplified = [path[0]]  # 保留起点
        
        for i in range(1, len(path) - 1):
            prev = simplified[-1]
            curr = path[i]
            next_pt = path[i + 1]
            
            # 计算角度变化
            v1 = np.array([curr[0] - prev[0], curr[1] - prev[1]])
            v2 = np.array([next_pt[0] - curr[0], next_pt[1] - curr[1]])
            
            # 归一化
            if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                v1_norm = v1 / np.linalg.norm(v1)
                v2_norm = v2 / np.linalg.norm(v2)
                
                # 计算夹角（点积）
                cos_angle = np.dot(v1_norm, v2_norm)
                
                # 如果方向变化明显（夹角 < 150度），保留该点
                if cos_angle < 0.866:  # cos(30°) ≈ 0.866
                    simplified.append(curr)
            else:
                simplified.append(curr)
        
        simplified.append(path[-1])  # 保留终点
        
        # ====================================================================
        # 步骤2：使用分段线性插值进行初步平滑
        # ====================================================================
        if len(simplified) < 3:
            return simplified
        
        smoothed = []
        
        for i in range(len(simplified) - 1):
            p1 = np.array(simplified[i])
            p2 = np.array(simplified[i + 1])
            
            # 两点间距离
            distance = np.linalg.norm(p2 - p1)
            
            # A*改进: G1人形机器人的步幅约0.3-0.5米，路径点间隔从0.4米降至0.3米提升密度
            num_points = max(2, int(distance / 0.3))
            
            # 线性插值
            for j in range(num_points):
                t = j / num_points
                point = p1 + t * (p2 - p1)
                smoothed.append((int(point[0]), int(point[1])))
        
        smoothed.append(simplified[-1])  # 添加最后一个点
        
        # ====================================================================
        # 步骤3：移动平均平滑（确保曲率连续）
        # ====================================================================
        if len(smoothed) < 5:
            return smoothed
        
        final_smoothed = [smoothed[0], smoothed[1]]  # 保留前两个点
        
        # 移动平均窗口大小
        # A*改进: 减小窗口从5降至3,避免平滑后偏离障碍物
        WINDOW_SIZE = 3
        
        for i in range(2, len(smoothed) - 2):
            # 收集窗口内的点
            window_points = []
            for j in range(max(0, i - WINDOW_SIZE // 2), 
                          min(len(smoothed), i + WINDOW_SIZE // 2 + 1)):
                window_points.append(smoothed[j])
            
            # 计算平均位置
            avg_x = sum(p[0] for p in window_points) / len(window_points)
            avg_y = sum(p[1] for p in window_points) / len(window_points)
            
            smoothed_point = (int(avg_x), int(avg_y))
            
            # 碰撞检查：确保平滑后的点仍然安全
            if self.is_valid(smoothed_point[0], smoothed_point[1]):
                final_smoothed.append(smoothed_point)
            else:
                # 如果平滑点不安全，使用原始点
                final_smoothed.append(smoothed[i])
        
        final_smoothed.extend(smoothed[-2:])  # 保留最后两个点
        
        # ====================================================================
        # 步骤4：G1单轴移动优化 - 路径对齐到主轴方向
        # 确保路径段尽可能沿X轴或Y轴，避免复杂的斜向插值
        # ====================================================================
        axis_aligned = self.align_path_to_axes(final_smoothed)
        
        self.get_logger().info(
            f"路径平滑: 原始{len(path)}点 → 简化{len(simplified)}点 "
            f"→ 插值{len(smoothed)}点 → 平滑{len(final_smoothed)}点 → 轴对齐{len(axis_aligned)}点"
        )
        
        return axis_aligned
    
    def align_path_to_axes(self, path):
        """
        将路径对齐到坐标轴方向（曼哈顿路径化）
        
        对于每个路径段，如果不是完全水平或垂直，
        将其分解为水平+垂直两段
        
        Args:
            path: 输入路径点列表
        
        Returns:
            aligned_path: 轴对齐后的路径
        """
        if len(path) < 2:
            return path
        
        aligned = [path[0]]  # 保留起点
        
        for i in range(len(path) - 1):
            curr = path[i]
            next_pt = path[i + 1]
            
            dx = next_pt[0] - curr[0]
            dy = next_pt[1] - curr[1]
            
            # 如果已经是轴对齐（纯水平或纯垂直）
            if dx == 0 or dy == 0:
                if next_pt not in aligned:
                    aligned.append(next_pt)
            else:
                # 分解为两段：先走主方向，再走次方向
                # 选择距离较大的作为主方向
                if abs(dx) >= abs(dy):
                    # 先走X，再走Y
                    mid_point = (next_pt[0], curr[1])
                    if mid_point not in aligned and self.is_valid(mid_point[0], mid_point[1]):
                        aligned.append(mid_point)
                    if next_pt not in aligned:
                        aligned.append(next_pt)
                else:
                    # 先走Y，再走X
                    mid_point = (curr[0], next_pt[1])
                    if mid_point not in aligned and self.is_valid(mid_point[0], mid_point[1]):
                        aligned.append(mid_point)
                    if next_pt not in aligned:
                        aligned.append(next_pt)
        
        return aligned
    
    def publish_path(self, path):
        """发布路径消息"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        # 保存世界坐标路径点和速度命令
        world_path = []
        
        # 计算目标点的世界坐标（用于朝向计算）
        goal_world_x, goal_world_y = self.map_to_world(self.goal_x, self.goal_y)
        
        for x, y in path:
            world_x, world_y = self.map_to_world(x, y)
            world_path.append({'x': world_x, 'y': world_y})
            
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            
            # ================================================================
            # G1单轴移动机器人：朝向指向目标（供参考，实际不使用角速度）
            # 运动控制器应忽略朝向，仅使用线性速度命令
            # ================================================================
            yaw_to_goal = math.atan2(goal_world_y - world_y, goal_world_x - world_x)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw_to_goal / 2)
            pose.pose.orientation.w = math.cos(yaw_to_goal / 2)
            
            path_msg.poses.append(pose)
        
        # 如果路径为空，至少添加起点和终点
        if not path_msg.poses:
            start_world_x, start_world_y = self.map_to_world(self.start_x, self.start_y)
            goal_world_x, goal_world_y = self.map_to_world(self.goal_x, self.goal_y)
            
            world_path.append({'x': start_world_x, 'y': start_world_y})
            world_path.append({'x': goal_world_x, 'y': goal_world_y})
            
            start_pose = PoseStamped()
            start_pose.header.frame_id = "map"
            start_pose.pose.position.x = start_world_x
            start_pose.pose.position.y = start_world_y
            
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.pose.position.x = goal_world_x
            goal_pose.pose.position.y = goal_world_y
            
            path_msg.poses.append(start_pose)
            path_msg.poses.append(goal_pose)
        
        # 保存路径到文件
        self.save_path_to_file(world_path)
        
        # 发布路径消息
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"已发布包含{len(path_msg.poses)}个点的路径")
    
    def save_path_to_file(self, world_path):
        """将路径保存到CSV文件"""
        try:
            # 获取包的路径 - ROS2使用ament_index_python
            from ament_index_python.packages import get_package_share_directory
            package_path = get_package_share_directory('g1_nav')
            
            # 创建path文件夹（如果不存在）
            path_dir = os.path.join(package_path, 'path')
            if not os.path.exists(path_dir):
                os.makedirs(path_dir)
            
            # 使用时间戳创建唯一文件名
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(path_dir, f'path_{timestamp}.csv')
            
            # 保存到CSV文件
            with open(filename, 'w', newline='') as csvfile:
                # 创建CSV写入器
                writer = csv.writer(csvfile)
                
                # 写入标题行
                writer.writerow(['x', 'y'])
                
                # 写入所有路径点
                for point in world_path:
                    writer.writerow([point['x'], point['y']])
            
            self.get_logger().info(f"路径已保存到CSV文件: {filename}")
        except Exception as e:
            self.get_logger().error(f"保存路径到文件时出错: {e}")
    
    def generate_velocity_command(self, current_pose, target_waypoint):
        """
        为G1单轴移动机器人生成速度命令（单轴线性速度，无角速度）
        
        策略（贪婪法则）：
        1. 计算到目标点的dx和dy
        2. 选择绝对值较大的分量作为主方向
        3. 只在主方向上生成速度，另一方向速度为0
        4. 角速度恒为0
        
        运动约束：
        - 前/后: vx ≠ 0, vy = 0, wz = 0
        - 左/右: vx = 0, vy ≠ 0, wz = 0
        
        Args:
            current_pose: 当前机器人位姿 {'x': x, 'y': y}
            target_waypoint: 目标路径点 {'x': x, 'y': y}
        
        Returns:
            velocity_cmd: {'linear_x': vx, 'linear_y': vy, 'angular_z': 0.0,
                          'direction': 'x'/'y'/'stop'}
        """
        curr_x, curr_y = current_pose['x'], current_pose['y']
        target_x, target_y = target_waypoint['x'], target_waypoint['y']
        
        # 计算误差
        dx = target_x - curr_x
        dy = target_y - curr_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 如果已经到达路径点
        if distance < self.waypoint_tolerance:
            return {
                'linear_x': 0.0,
                'linear_y': 0.0,
                'angular_z': 0.0,
                'direction': 'stop'
            }
        
        # ====================================================================
        # 单轴选择策略：贪婪法 - 每次走误差最大的方向
        # 优点：收敛快，适合动态环境
        # ====================================================================
        abs_dx = abs(dx)
        abs_dy = abs(dy)
        
        # 计算速度大小（比例控制）
        speed = min(self.max_linear_speed, distance * 2.0)
        speed = max(0.1, speed)  # 最小速度
        
        # 选择主方向
        if abs_dx >= abs_dy:
            # X方向移动（前/后）
            vx = speed if dx > 0 else -speed
            return {
                'linear_x': vx,
                'linear_y': 0.0,
                'angular_z': 0.0,
                'direction': 'x'
            }
        else:
            # Y方向移动（左/右）
            vy = speed if dy > 0 else -speed
            return {
                'linear_x': 0.0,
                'linear_y': vy,
                'angular_z': 0.0,
                'direction': 'y'
            }
    
    def generate_path_velocities(self, path):
        """
        为整条路径生成速度命令序列（单轴移动）
        
        对于轴对齐的路径，每段路径自然是单轴的
        
        Args:
            path: 路径点列表 [(x1, y1), (x2, y2), ...] (地图坐标)
        
        Returns:
            velocities: 速度命令列表，每个元素对应一个路径段
        """
        if len(path) < 2:
            return []
        
        velocities = []
        prev_direction = None
        
        for i in range(len(path) - 1):
            # 当前点和下一个点
            curr_map = path[i]
            next_map = path[i + 1]
            
            # 转换为世界坐标
            curr_world_x, curr_world_y = self.map_to_world(curr_map[0], curr_map[1])
            next_world_x, next_world_y = self.map_to_world(next_map[0], next_map[1])
            
            # 计算移动方向
            dx = next_world_x - curr_world_x
            dy = next_world_y - curr_world_y
            
            # 对于轴对齐路径，自然是单轴移动
            if abs(dx) > abs(dy):
                direction = 'x'
                vx = self.max_linear_speed if dx > 0 else -self.max_linear_speed
                vy = 0.0
            else:
                direction = 'y'
                vx = 0.0
                vy = self.max_linear_speed if dy > 0 else -self.max_linear_speed
            
            # 方向切换时的标记
            direction_changed = (prev_direction is not None and prev_direction != direction)
            
            velocities.append({
                'waypoint': (next_world_x, next_world_y),
                'velocity': {
                    'linear_x': vx,
                    'linear_y': vy,
                    'angular_z': 0.0,
                    'direction': direction
                },
                'distance': math.sqrt(dx*dx + dy*dy),
                'direction_changed': direction_changed
            })
            
            prev_direction = direction
        
        # 统计信息
        x_segments = sum(1 for v in velocities if v['velocity']['direction'] == 'x')
        y_segments = sum(1 for v in velocities if v['velocity']['direction'] == 'y')
        switches = sum(1 for v in velocities if v['direction_changed'])
        
        self.get_logger().info(
            f"速度序列生成完成: "
            f"总段数={len(velocities)}, "
            f"X方向={x_segments}, Y方向={y_segments}, "
            f"方向切换={switches}次"
        )
        
        return velocities


def main(args=None):
    rclpy.init(args=args)
    
    try:
        planner = AStarPlanner()
        
        # 使用多线程执行器
        executor = MultiThreadedExecutor()
        executor.add_node(planner)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            planner.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
