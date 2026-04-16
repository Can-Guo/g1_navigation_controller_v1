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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
#from tf_transformations import euler_from_quaternion
import heapq
import math
from scipy import ndimage
from common_astar_utils import save_world_path_to_csv


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
        self.inflation_radius_m = 0.25  # 膨胀半径（米）- 减小障碍物膨胀
        
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
        # 在进入新任务前先清空上一任务的可视化路径，避免 RViz 中残留旧轨迹干扰评审。
        # 这里发布一个空 Path（同一 topic），RViz Path display 会立即刷新为“无历史路径”状态。
        self.clear_astar_path_visualization()
            
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
    
    def clear_astar_path_visualization(self):
        """
        清空 A* 路径可视化。
        
        设计说明：
        - ROS2 RViz2 的 Path Display 不支持“清除按钮”语义消息；
        - 最稳妥方式是向同一 topic 发布空 Path，触发 display 立即覆盖旧数据；
        - 对于“每次只看一个目标任务”的运维需求，建议在新 goal 到来时先清空，再重算新路径。
        """
        empty_path_msg = Path()
        empty_path_msg.header.stamp = self.get_clock().now().to_msg()
        empty_path_msg.header.frame_id = "map"
        self.path_pub.publish(empty_path_msg)
        self.get_logger().info("已清空上一任务的 A* 路径可视化")
    
    
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
        """获取邻居节点"""
        neighbors = []
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            nx, ny = x + dx, y + dy
            if self.is_valid(nx, ny):
                # 对角线移动的代价为1.414
                cost = 1.414 if dx != 0 and dy != 0 else 1
                neighbors.append((nx, ny, cost))
        return neighbors
    
    def heuristic(self, x, y):
        """
        改进的启发式函数：考虑G1人形机器人的非完整约束
        
        G1机器人特点：
        1. 不能原地平移，只能前进+转向
        2. 转向需要时间和空间
        3. 转向半径约0.5米（保守估计）
        
        使用Dubins路径的简化版本作为下界估计
        """
        dx = self.goal_x - x
        dy = self.goal_y - y
        
        # 基础欧几里得距离
        euclidean_dist = math.sqrt(dx*dx + dy*dy)
        
        # 如果有当前朝向信息，计算转向代价
        if hasattr(self, 'current_heading') and self.current_heading is not None:
            # 计算目标方向
            target_heading = math.atan2(dy, dx)
            
            # 计算角度差异（归一化到[-pi, pi]）
            heading_diff = target_heading - self.current_heading
            while heading_diff > math.pi:
                heading_diff -= 2 * math.pi
            while heading_diff < -math.pi:
                heading_diff += 2 * math.pi
            
            # G1人形机器人的最小转向半径（保守估计）
            # 考虑到身高和步幅，转向半径约为0.5-0.8米
            MIN_TURN_RADIUS = 0.6
            
            # 估计转向代价：基于Dubins路径的弧长
            # 简化模型：转向代价 = 转向角度 × 转向半径
            turn_cost = abs(heading_diff) * MIN_TURN_RADIUS
            
            # 如果角度差异很大（需要掉头），额外增加代价
            # 🔧 绕圈修复: 降低大角度惩罚，允许机器人必要时掉头
            if abs(heading_diff) > math.pi * 0.75:  # 超过135度
                turn_cost += MIN_TURN_RADIUS * 1.2  # 从3.5降至1.2，避免过度绕路
            
            # 组合代价：直线距离 + 转向代价的权重
            # 🔧 绕圈修复: 降低转向权重，鼓励更直接的路径
            return euclidean_dist + 0.25 * turn_cost  # 从0.6降至0.25，减少转向惩罚
        
        # 如果没有朝向信息，使用基础欧几里得距离
        return euclidean_dist
    
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
        # 步骤4：G1特定优化 - 检查最小转向半径
        # ====================================================================
        MIN_TURN_RADIUS = 0.6  # G1的最小转向半径（米）
        
        optimized = [final_smoothed[0]]
        
        for i in range(1, len(final_smoothed) - 1):
            prev = optimized[-1]
            curr = final_smoothed[i]
            next_pt = final_smoothed[i + 1]
            
            # 计算转向半径
            # 使用三点计算曲率半径
            try:
                # 转换为世界坐标计算
                p1_world = np.array(self.map_to_world(prev[0], prev[1]))
                p2_world = np.array(self.map_to_world(curr[0], curr[1]))
                p3_world = np.array(self.map_to_world(next_pt[0], next_pt[1]))
                
                # 使用Menger曲率公式
                a = np.linalg.norm(p2_world - p1_world)
                b = np.linalg.norm(p3_world - p2_world)
                c = np.linalg.norm(p3_world - p1_world)
                
                # 半周长
                s = (a + b + c) / 2
                
                # 面积（海伦公式）
                area = math.sqrt(max(0, s * (s - a) * (s - b) * (s - c)))
                
                # 曲率半径
                if area > 0.001:  # 避免除零
                    radius = (a * b * c) / (4 * area)
                else:
                    radius = float('inf')
                
                # 如果转向半径小于最小值，跳过该点使路径更平直
                if radius >= MIN_TURN_RADIUS or radius == float('inf'):
                    optimized.append(curr)
            except:
                optimized.append(curr)  # 计算失败，保留原点
        
        optimized.append(final_smoothed[-1])
        
        self.get_logger().info(
            f"路径平滑: 原始{len(path)}点 → 简化{len(simplified)}点 "
            f"→ 插值{len(smoothed)}点 → 最终{len(optimized)}点"
        )
        
        return optimized
    
    def publish_path(self, path):
        """发布路径消息"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        # 保存世界坐标路径点
        world_path = []
        
        for x, y in path:
            world_x, world_y = self.map_to_world(x, y)
            world_path.append({'x': world_x, 'y': world_y})
            
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            
            # 简单的朝向计算
            if len(path_msg.poses) > 0:
                prev_x = path_msg.poses[-1].pose.position.x
                prev_y = path_msg.poses[-1].pose.position.y
                yaw = math.atan2(world_y - prev_y, world_x - prev_x) 
                
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = math.sin(yaw / 2)
                pose.pose.orientation.w = math.cos(yaw / 2)
            else:
                pose.pose.orientation.w = 1.0
            
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
            # 公共工具函数统一维护路径CSV落盘逻辑，避免双版本脚本重复实现。
            filename = save_world_path_to_csv(world_path)
            self.get_logger().info(f"路径已保存到CSV文件: {filename}")
        except Exception as e:
            self.get_logger().error(f"保存路径到文件时出错: {e}")


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
