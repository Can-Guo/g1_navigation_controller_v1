#!/usr/bin/env python3
"""
DWA (Dynamic Window Approach) Planner for ROS2 Humble
=====================================================
Phase 1 重构: 引入 Nav2 风格 LocalCostmap 替代自定义点云聚类障碍物检测

原始版本: 基于 DBSCAN 点云聚类 (process_pointcloud + cluster_points)
重构版本: 基于 2D 局部代价地图 (LocalCostmap + inflation + decay)

核心改进:
1. 障碍物持久化: 不再逐帧丢失, 具有时间衰减 (temporal decay)
2. 膨胀层梯度: Nav2 标准 inflation_layer, 提供平滑连续代价场
3. 形状保留: 栅格化保留障碍物真实形状, 不再简化为质心
4. 代价查询 O(1): 轨迹碰撞检测从 O(n*m) 降为 O(n) (n=轨迹点, m=障碍物数)
5. 滚动窗口: 以机器人为中心的 rolling window, 自动跟随

保留不变:
- DWA 轨迹生成与多目标评分逻辑
- 卡住检测与逃逸行为状态机
- 路径跟踪与航点管理
- 速度平滑与可视化发布

代价值定义 (遵循 Nav2 costmap_2d 标准):
  0     = FREE_SPACE (自由空间)
  1-252 = 膨胀层代价 (距障碍物越近代价越高, 指数衰减)
  253   = INSCRIBED_INFLATED_OBSTACLE (在机器人内切圆内, 必定碰撞)
  254   = LETHAL_OBSTACLE (障碍物实体占据的栅格)
  255   = NO_INFORMATION (未知区域 / 超出地图范围)
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
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, Point, PoseStamped
# 注意: obstacle_status 使用 geometry_msgs/Point 而非 Int32,
# 因为 g1_position_control_ros2.py 订阅时期望 Point 类型:
#   msg.x = 1.0 表示检测到障碍物, 0.0 表示无障碍
#   msg.y = 1.0 表示避障模式激活, 0.0 表示未激活
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
from scipy.ndimage import distance_transform_edt

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


# ============================================================================
# Nav2 风格局部代价地图 (Local Costmap)
# ============================================================================
# 设计原则:
#   - 遵循 Nav2 costmap_2d 的代价值语义 (0/253/254/255)
#   - 使用 scipy distance_transform_edt 实现高效膨胀 (与 Nav2 C++ 算法等价)
#   - 滚动窗口 (Rolling Window) 跟随机器人, 自动管理可见区域
#   - 障碍物时间衰减 (Temporal Decay) 解决视角盲区残影问题
# ============================================================================

class LocalCostmap:
    """
    Nav2 风格的 2D 局部代价地图

    核心功能:
    1. 从 PointCloud2 标记障碍物栅格 (mark_obstacle)
    2. 膨胀层: 距离变换 + 指数代价衰减 (apply_inflation)
    3. 滚动窗口: 以机器人为中心, 自动平移 (update_origin)
    4. 时间衰减: 未刷新的障碍物逐渐清除 (decay_obstacles)
    5. O(1) 代价查询: get_cost(world_x, world_y)
    """

    # Nav2 标准代价值常量
    FREE_SPACE = 0
    INSCRIBED_INFLATED = 253
    LETHAL_OBSTACLE = 254
    NO_INFORMATION = 255

    def __init__(self, width_m=4.0, height_m=4.0, resolution=0.05,
                 inflation_radius=0.45, robot_radius=0.30, decay_factor=0.90,
                 cost_scaling_factor=8.0):
        """
        初始化局部代价地图

        Args:
            width_m: 地图宽度 (米), 应 >= 2 × detection_range
            height_m: 地图高度 (米)
            resolution: 栅格分辨率 (米/格), 越小越精细但内存越大
            inflation_radius: 膨胀半径 (米) — 对应 Nav2 inflation_layer::inflation_radius
                              超过此距离的栅格代价为 0
                              设计原则: 应 < corridor_width / 2, 确保窄通道中心存在零代价通道
                              1.0m 走廊 → 需 < 0.50m; 推荐 0.45m (留 10cm 零代价通道)
            robot_radius: 机器人碰撞半径 (米) — 对应 Nav2 inflation_layer::robot_radius
                          此距离内的栅格代价 = 253 (INSCRIBED, 必定碰撞)
                          对于非圆形机器人, 应取 max(width, depth) / 2 + safety_margin
                          以确保最宽方向上的碰撞安全:
                            G1 人形机器人 宽度=0.55m → 半宽=0.275m + 0.025m安全余量 = 0.30m
                          (注: Nav2 中对应 circumscribed_radius 更精确,
                           但圆形近似下取半宽是保守且安全的选择)
            decay_factor: 每控制周期 (10Hz) 的障碍物衰减系数 (0~1)
                          0.90 表示约 2 秒内衰减到接近 0
            cost_scaling_factor: Nav2 膨胀代价衰减速率 (α)
                          代价公式: cost = 252 × exp(-α × (d - robot_radius))
                          α 越大, 高代价区域越窄, 走廊中心越"干净"
                          推荐: 3.0 (开阔场地), 5.0 (中等通道), 8.0 (窄走廊 ≤1.0m)
        """
        self.width_m = width_m
        self.height_m = height_m
        self.resolution = resolution
        self.inflation_radius = inflation_radius
        self.robot_radius = robot_radius
        self.decay_factor = decay_factor
        self.cost_scaling_factor = cost_scaling_factor

        # 栅格尺寸 (像素)
        self.width_cells = int(width_m / resolution)
        self.height_cells = int(height_m / resolution)

        # =====================================================================
        # 三层地图数据结构 (与 Nav2 costmap_2d 的 layered costmap 类似)
        # =====================================================================
        # 第 1 层: 原始障碍物标记 (仅含 0 和 LETHAL_OBSTACLE)
        self.raw_grid = np.zeros((self.height_cells, self.width_cells), dtype=np.uint8)
        # 第 2 层: 带衰减的障碍物强度 (浮点, 用于时间衰减计算)
        self.intensity_grid = np.zeros((self.height_cells, self.width_cells), dtype=np.float32)
        # 第 3 层: 膨胀后的最终代价地图 (DWA 实际查询的层)
        self.inflated_grid = np.zeros((self.height_cells, self.width_cells), dtype=np.uint8)

        # 地图左下角在世界坐标系 (odom) 中的位置
        self.origin_x = 0.0
        self.origin_y = 0.0

        # 障碍物计数 (用于日志和状态判断)
        self.obstacle_cell_count = 0

    # -----------------------------------------------------------------
    # 滚动窗口管理
    # -----------------------------------------------------------------
    def update_origin(self, robot_x, robot_y):
        """
        更新地图原点, 使机器人始终处于地图中心 (Rolling Window 核心)

        原理: 每次机器人移动超过一个栅格距离时, 将地图内容反向平移,
              使地图中心跟随机器人。新露出的区域初始化为自由空间。

        Args:
            robot_x, robot_y: 机器人当前位置 (odom 坐标系)
        """
        new_origin_x = robot_x - self.width_m / 2.0
        new_origin_y = robot_y - self.height_m / 2.0

        # 计算需要滚动的栅格数
        dx_cells = int((new_origin_x - self.origin_x) / self.resolution)
        dy_cells = int((new_origin_y - self.origin_y) / self.resolution)

        if dx_cells == 0 and dy_cells == 0:
            return  # 没有移动足够距离, 无需滚动

        # 如果位移过大 (>地图尺寸的一半), 直接重置地图
        if abs(dx_cells) > self.width_cells // 2 or abs(dy_cells) > self.height_cells // 2:
            self.raw_grid.fill(0)
            self.intensity_grid.fill(0.0)
            self.inflated_grid.fill(0)
            self.origin_x = new_origin_x
            self.origin_y = new_origin_y
            return

        # 使用 np.roll 滚动地图内容 (反向平移, 使新区域在边缘)
        for grid in [self.raw_grid, self.intensity_grid]:
            shifted = np.roll(grid, (-dy_cells, -dx_cells), axis=(0, 1))
            np.copyto(grid, shifted)

        # 清除新露出的边缘区域 (设为自由空间)
        if dx_cells > 0:
            self.raw_grid[:, -dx_cells:] = 0
            self.intensity_grid[:, -dx_cells:] = 0.0
        elif dx_cells < 0:
            self.raw_grid[:, :abs(dx_cells)] = 0
            self.intensity_grid[:, :abs(dx_cells)] = 0.0
        if dy_cells > 0:
            self.raw_grid[-dy_cells:, :] = 0
            self.intensity_grid[-dy_cells:, :] = 0.0
        elif dy_cells < 0:
            self.raw_grid[:abs(dy_cells), :] = 0
            self.intensity_grid[:abs(dy_cells), :] = 0.0

        self.origin_x = new_origin_x
        self.origin_y = new_origin_y

    # -----------------------------------------------------------------
    # 坐标转换
    # -----------------------------------------------------------------
    def world_to_grid(self, wx, wy):
        """世界坐标 (odom) → 栅格坐标 (col, row)"""
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        """栅格坐标 → 世界坐标 (栅格中心)"""
        wx = self.origin_x + (gx + 0.5) * self.resolution
        wy = self.origin_y + (gy + 0.5) * self.resolution
        return wx, wy

    def is_in_bounds(self, gx, gy):
        """检查栅格坐标是否在地图范围内"""
        return 0 <= gx < self.width_cells and 0 <= gy < self.height_cells

    # -----------------------------------------------------------------
    # 障碍物标记与衰减
    # -----------------------------------------------------------------
    def mark_obstacle(self, wx, wy):
        """
        在世界坐标处标记障碍物 (LETHAL_OBSTACLE)

        Args:
            wx, wy: 障碍物世界坐标 (odom frame)
        """
        gx, gy = self.world_to_grid(wx, wy)
        if self.is_in_bounds(gx, gy):
            self.raw_grid[gy, gx] = self.LETHAL_OBSTACLE
            # 强度设为最大值, 后续由 decay 逐步降低
            self.intensity_grid[gy, gx] = float(self.LETHAL_OBSTACLE)

    def clear_current_frame(self):
        """
        清除原始障碍物层 (每帧更新前调用)

        注意: 只清除 raw_grid, intensity_grid 保留用于衰减计算。
        这确保上一帧看到但本帧未看到的障碍物不会立即消失,
        而是通过 decay 逐渐衰减 — 这正是 costmap 持久化的核心。
        """
        self.raw_grid.fill(0)

    def decay_obstacles(self):
        """
        障碍物时间衰减

        原理: 未被当前帧重新标记的障碍物, 其强度按 decay_factor 指数衰减。
              当强度降至阈值以下时彻底清除。

        解决的问题:
        - LiDAR 视角盲区导致的障碍物残留 (转头后身后障碍物应消失)
        - 动态障碍物 (行人) 移走后的残影
        - 虚假检测 (噪声点) 的自动清除
        """
        # 已被当前帧刷新的栅格 (raw_grid 中标记为 LETHAL) 不衰减
        refreshed_mask = (self.raw_grid == self.LETHAL_OBSTACLE)
        # 刷新的栅格: 强度恢复到最大值
        self.intensity_grid[refreshed_mask] = float(self.LETHAL_OBSTACLE)
        # 未刷新的栅格: 应用衰减
        not_refreshed = ~refreshed_mask & (self.intensity_grid > 0)
        self.intensity_grid[not_refreshed] *= self.decay_factor
        # 衰减到很低时彻底清零 (避免无限趋近 0 的浮点残余)
        self.intensity_grid[self.intensity_grid < 5.0] = 0.0

    # -----------------------------------------------------------------
    # 膨胀层 (Inflation Layer)
    # -----------------------------------------------------------------
    def apply_inflation(self):
        """
        应用膨胀层, 生成最终代价地图

        算法与 Nav2 costmap_2d::InflationLayer 等价:
        1. 将强度 > 阈值的栅格视为障碍物 (二值化)
        2. 使用 scipy distance_transform_edt 计算每个自由栅格到最近障碍物的欧式距离
        3. 根据距离应用 Nav2 标准膨胀代价公式:
           - d == 0:                cost = 254 (LETHAL)
           - 0 < d <= robot_radius: cost = 253 (INSCRIBED)
           - robot_radius < d <= inflation_radius:
                                    cost = 252 × exp(-cost_scaling × (d - robot_radius))
           - d > inflation_radius:  cost = 0   (FREE)

        性能: distance_transform_edt 是 C 语言实现的向量化操作,
              在 80×80 栅格上耗时 < 0.5ms, 远快于逐点 DBSCAN 聚类。
        """
        # 将强度超过阈值的栅格视为障碍物 (阈值 = LETHAL/2 = 127)
        # 这使得衰减中的旧障碍物仍然被考虑, 直到彻底消失
        binary_obstacle = (self.intensity_grid >= 127.0).astype(np.uint8)

        self.obstacle_cell_count = int(np.sum(binary_obstacle))

        if self.obstacle_cell_count == 0:
            self.inflated_grid.fill(0)
            return

        # ---- 距离变换 (核心) ----
        # distance_transform_edt 计算每个 0-值栅格到最近 1-值栅格的欧式距离
        # 输入: 1-binary (障碍物=0, 自由=1), 输出: 距离 (单位: 栅格)
        dist_cells = distance_transform_edt(1 - binary_obstacle)
        # 转换为米
        dist_m = dist_cells * self.resolution

        # ---- 应用 Nav2 膨胀代价公式 ----
        cost = np.zeros_like(dist_m, dtype=np.float64)

        # 障碍物本体 (距离 = 0)
        cost[dist_m == 0] = self.LETHAL_OBSTACLE

        # 内切圆区域 (0 < d <= robot_radius): 必定碰撞
        inscribed_mask = (dist_m > 0) & (dist_m <= self.robot_radius)
        cost[inscribed_mask] = self.INSCRIBED_INFLATED

        # 膨胀区域 (robot_radius < d <= inflation_radius): 指数衰减
        inflation_mask = (dist_m > self.robot_radius) & (dist_m <= self.inflation_radius)
        cost[inflation_mask] = 252.0 * np.exp(
            -self.cost_scaling_factor * (dist_m[inflation_mask] - self.robot_radius)
        )

        # 截断到 [0, 254] 并转为 uint8
        self.inflated_grid = np.clip(cost, 0, 254).astype(np.uint8)

    # -----------------------------------------------------------------
    # 代价查询 API
    # -----------------------------------------------------------------
    def get_cost(self, wx, wy):
        """
        查询世界坐标处的代价值 (O(1) 栅格查找)

        Args:
            wx, wy: 世界坐标 (odom frame)
        Returns:
            int: 0=自由, 1-252=膨胀区, 253=内切障碍, 254=致命, 255=越界
        """
        gx, gy = self.world_to_grid(wx, wy)
        if not self.is_in_bounds(gx, gy):
            return self.NO_INFORMATION
        return int(self.inflated_grid[gy, gx])

    def has_obstacles(self):
        """代价地图中是否存在障碍物"""
        return self.obstacle_cell_count > 0

    # -----------------------------------------------------------------
    # RViz 可视化: 转换为 OccupancyGrid 消息
    # -----------------------------------------------------------------
    def to_occupancy_grid_msg(self, stamp, frame_id='odom'):
        """
        将膨胀后代价地图转换为 nav_msgs/OccupancyGrid, 供 RViz 显示

        映射规则 (Nav2 代价值 → OccupancyGrid 值):
          0       → 0   (自由)
          1-252   → 1-99 (按比例缩放)
          253-254 → 100  (障碍)
          255     → -1   (未知)
        """
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.info.resolution = self.resolution
        msg.info.width = self.width_cells
        msg.info.height = self.height_cells
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        flat = self.inflated_grid.flatten().astype(np.int16)
        data = np.zeros_like(flat, dtype=np.int8)
        # 自由空间
        data[flat == 0] = 0
        # 膨胀区 → 按比例映射到 1-99
        infl_mask = (flat > 0) & (flat < 253)
        data[infl_mask] = np.clip(flat[infl_mask] * 99 // 252, 1, 99).astype(np.int8)
        # 致命/内切 → 100
        data[flat >= 253] = 100
        msg.data = data.tolist()
        return msg


class DWAPlanner(Node):
    """DWA local planner with obstacle avoidance"""
    
    def __init__(self):
        super().__init__('dwa_planner')
        
        self.get_logger().info("Initializing DWA Planner for ROS2 Humble (Nav2 Costmap 版)...")
        
        # Callback groups for concurrent execution
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.subscription_group = ReentrantCallbackGroup()
        
        # === Parameters Declaration (ROS2 pattern) ===
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('min_speed', 0.12)
        self.declare_parameter('max_turn', 0.12)
        self.declare_parameter('detection_range', 1.2)
        self.declare_parameter('time_horizon', 1.0)
        # 以下两个参数保留声明以兼容 launch 文件, 但不再用于 DBSCAN 聚类
        self.declare_parameter('min_obstacle_points', 40)
        self.declare_parameter('clustering_distance', 0.3)
        self.declare_parameter('smooth_factor', 0.2)
        self.declare_parameter('waypoint_distance', 0.3)
        self.declare_parameter('debug', False)
        self.declare_parameter('debug_interval', 2.0)
        self.declare_parameter('loop_rate', 10.0)
        # ---- Nav2 Costmap 新增参数 ----
        self.declare_parameter('costmap_width', 4.0)         # 局部代价地图宽度 (米)
        self.declare_parameter('costmap_height', 4.0)        # 局部代价地图高度 (米)
        self.declare_parameter('costmap_resolution', 0.05)   # 栅格分辨率 (米/格)
        self.declare_parameter('inflation_radius', 0.45)     # 膨胀半径 (米), 应 < 走廊宽度/2
        self.declare_parameter('robot_radius', 0.30)         # G1 半宽=0.275m + 安全余量=0.30m
        self.declare_parameter('obstacle_decay_factor', 0.90) # 障碍物衰减因子
        # Nav2 膨胀代价衰减速率: α 越大高代价区越窄, 走廊中心越干净
        self.declare_parameter('cost_scaling_factor', 8.0)
        # 障碍物检测代价阈值: 仅 INSCRIBED(253) 及以上才触发 DWA 避障模式
        # 确保走廊墙壁的膨胀梯度不会误触发避障, 只有真正逼近才激活
        self.declare_parameter('obstacle_cost_threshold', 253)
        # ---- check_obstacles 窄走廊适配参数 ----
        # 扫描半锥角 (弧度): 原始 ±90° 在走廊中会扫到两侧墙壁 → 永久触发 DWA
        # 收窄为 ±30° 只关注前方路径上的真实障碍, 侧向墙壁交给 costmap 梯度处理
        self.declare_parameter('obstacle_detection_half_angle', 0.52)  # ≈30°
        # 最小高代价采样点计数: 只有超过此数量的采样点超标才触发 DWA
        # 走廊墙壁在窄锥角下仅产生少量边缘命中; 真实障碍 (行人/箱子) 产生密集命中
        self.declare_parameter('min_obstacle_samples', 8)
        # 高度过滤范围 (米): 只保留此范围内的点云作为障碍物
        self.declare_parameter('obstacle_min_height', -0.3)
        self.declare_parameter('obstacle_max_height', 0.6)
        
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
        self.obstacle_cost_threshold = self.get_parameter('obstacle_cost_threshold').value
        self.obstacle_min_height = self.get_parameter('obstacle_min_height').value
        self.obstacle_max_height = self.get_parameter('obstacle_max_height').value
        self.obstacle_detection_half_angle = self.get_parameter('obstacle_detection_half_angle').value
        self.min_obstacle_samples = self.get_parameter('min_obstacle_samples').value
        
        # =====================================================================
        # 初始化 Nav2 风格局部代价地图 (替代 DBSCAN 聚类管线)
        # =====================================================================
        self.local_costmap = LocalCostmap(
            width_m=self.get_parameter('costmap_width').value,
            height_m=self.get_parameter('costmap_height').value,
            resolution=self.get_parameter('costmap_resolution').value,
            inflation_radius=self.get_parameter('inflation_radius').value,
            robot_radius=self.get_parameter('robot_radius').value,
            decay_factor=self.get_parameter('obstacle_decay_factor').value,
            cost_scaling_factor=self.get_parameter('cost_scaling_factor').value,
        )
        self.get_logger().info(
            f"  Nav2 LocalCostmap 初始化: "
            f"{self.local_costmap.width_cells}×{self.local_costmap.height_cells} cells, "
            f"分辨率={self.local_costmap.resolution}m, "
            f"膨胀={self.local_costmap.inflation_radius}m"
        )
        
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
            '/odometry/filtered',
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
        
        # /obstacle_status 使用 Point 类型, 与 g1_position_control_ros2.py 订阅端一致
        # msg.x: 1.0=检测到障碍物, 0.0=无障碍物
        # msg.y: 1.0=避障模式激活, 0.0=未激活
        self.obstacle_status_pub = self.create_publisher(
            Point,
            '/obstacle_status',
            reliable_qos
        )
        
        # ---- /dwa_obstacles: 供 Action Server 读取的障碍物点云 ----
        # point_to_point_action_server.py 订阅此话题 (PointCloud2),
        # 通过 read_points(field_names=("x","y","z")) 计算最近障碍物距离,
        # 更新 feedback.obstacle_proximity 等级 (CLEAR / NEAR / CRITICAL)。
        # 数据源: update_costmap_from_pointcloud() 中经三重过滤后的障碍物点
        self.dwa_obstacles_pub = self.create_publisher(
            PointCloud2,
            '/dwa_obstacles',
            reliable_qos
        )
        
        # ---- Nav2 Costmap 可视化发布者 ----
        # 以 OccupancyGrid 格式发布膨胀后的代价地图, 可在 RViz 中直接显示
        costmap_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/dwa_local_costmap',
            costmap_qos
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
        
        # Phase 1 重构: self.obstacles 列表不再使用, 改用 self.local_costmap
        # 保留空列表以兼容可能的外部引用
        self.obstacles = []
        self.obstacle_avoidance_active = False
        self.preferred_turn_direction = None
        # costmap 是否已收到至少一帧有效数据
        self.costmap_ready = False
        
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
        
        self.get_logger().info(f"DWA Planner (Nav2 Costmap 版) 初始化完成:")
        self.get_logger().info(f"  速度限制: max={self.max_speed}, min={self.min_speed}")
        self.get_logger().info(f"  检测范围: {self.detection_range}m, 控制频率: {loop_rate} Hz")
        self.get_logger().info(f"  障碍物检测: Nav2 Costmap 模式 (替代 DBSCAN 聚类)")
        self.get_logger().info(f"  代价阈值: obstacle_cost_threshold={self.obstacle_cost_threshold}")
        self.get_logger().info(
            f"  前方扫描: ±{math.degrees(self.obstacle_detection_half_angle):.0f}°, "
            f"最小触发采样={self.min_obstacle_samples}"
        )
        self.get_logger().info(f"  高度过滤: [{self.obstacle_min_height}, {self.obstacle_max_height}]m")
    
    # === Callback Functions ===
    
    def pointcloud_callback(self, msg):
        """
        接收点云数据并更新局部代价地图

        Phase 1 重构:
        原始流程: pointcloud → process_pointcloud → cluster_points → self.obstacles
        新流程:   pointcloud → update_costmap_from_pointcloud → self.local_costmap
        """
        self.pointcloud_data = msg
        self.update_costmap_from_pointcloud()
    
    def odom_callback(self, msg):
        """接收里程计数据"""
        self.odom_data = msg
    
    def path_callback(self, msg):
        """接收 A* 全局路径"""
        if msg.poses:
            self.waypoints = msg.poses
            self.has_path = True
            self.goal_reached = False
            self.current_waypoint_index = 0
            
            if len(self.waypoints) > 0:
                self.goal_x = self.waypoints[-1].pose.position.x
                self.goal_y = self.waypoints[-1].pose.position.y
            
            self.get_logger().info(f"收到新路径, 共 {len(self.waypoints)} 个航点")
        else:
            self.waypoints = []
            self.has_path = False
            self.goal_x = None
            self.goal_y = None
    
    def publish_obstacle_status(self, obstacle_detected: bool, avoidance_active: bool):
        """
        发布障碍物/避障状态, 与 g1_position_control_ros2.py 的订阅端一致。

        协议 (geometry_msgs/Point):
            msg.x: 1.0=检测到障碍物, 0.0=无障碍物
            msg.y: 1.0=避障模式激活, 0.0=未激活
            msg.z: 保留 (0.0)
        """
        msg = Point()
        msg.x = 1.0 if obstacle_detected else 0.0
        msg.y = 1.0 if avoidance_active else 0.0
        msg.z = 0.0
        self.obstacle_status_pub.publish(msg)
    
    # =====================================================================
    # 点云 → 代价地图更新管线 (替代原 process_pointcloud + cluster_points)
    # =====================================================================
    
    def update_costmap_from_pointcloud(self):
        """
        从点云更新局部代价地图 (Nav2 Costmap 核心流程)

        完整管线:
        1. 获取机器人位姿 → 更新 costmap 滚动窗口原点
        2. 衰减上一帧的旧障碍物 (时间持久化)
        3. 清除原始层, 准备接收新帧数据
        4. TF 变换: 点云坐标 → odom 坐标系
        5. 高度 + 距离 + 角度三重过滤
        6. 将过滤后的点标记到 costmap 原始层
        7. 距离变换 + 膨胀层生成最终代价地图
        8. 发布 OccupancyGrid 可视化

        相比原始 DBSCAN 管线的改进:
        - 不再逐点构造 PointStamped → 使用矩阵化 TF 变换 (10x 加速)
        - 不再 O(n²) 聚类 → 直接标记栅格 (O(n))
        - 障碍物具有时间衰减, 不会因为单帧丢失而突然消失
        - 膨胀层提供平滑代价梯度, 替代离散质心距离检测
        """
        if self.pointcloud_data is None or self.odom_data is None:
            return
        
        # ---- Step 1: 获取机器人位姿, 更新滚动窗口 ----
        robot_x = self.odom_data.pose.pose.position.x
        robot_y = self.odom_data.pose.pose.position.y
        orient = self.odom_data.pose.pose.orientation
        _, _, robot_yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        self.local_costmap.update_origin(robot_x, robot_y)
        
        # ---- Step 2: 衰减旧障碍物 ----
        self.local_costmap.decay_obstacles()
        
        # ---- Step 3: 清除原始标记层 (保留 intensity 用于衰减) ----
        self.local_costmap.clear_current_frame()
        
        # ---- Step 4: TF 变换 (点云帧 → odom 帧) ----
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom',
                self.pointcloud_data.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"无法获取 TF 变换 (点云→odom): {ex}", throttle_duration_sec=2.0
            )
            return
        
        # 提取 TF 变换的旋转和平移 (矩阵化, 避免逐点 do_transform_point)
        t = transform.transform.translation
        q = transform.transform.rotation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        rot_matrix = rot.as_matrix()  # 3×3 旋转矩阵
        translation = np.array([t.x, t.y, t.z])
        
        # ---- Step 5: 批量读取点云并矩阵化变换 ----
        points_list = []
        for pt in point_cloud2.read_points(
            self.pointcloud_data, field_names=("x", "y", "z"), skip_nans=True
        ):
            points_list.append([float(pt[0]), float(pt[1]), float(pt[2])])
        
        if not points_list:
            # 无有效点, 仍然应用膨胀 (衰减中的旧障碍物仍需膨胀)
            self.local_costmap.apply_inflation()
            self._publish_costmap_visualization()
            return
        
        # numpy 矩阵化变换: [N, 3] @ R^T + t (比逐点 TF 快 10-50 倍)
        pts = np.array(points_list)  # shape: (N, 3)
        transformed = (rot_matrix @ pts.T).T + translation  # shape: (N, 3)
        
        # ---- Step 6: 三重过滤 (距离 + 角度 + 高度) ----
        dx = transformed[:, 0] - robot_x
        dy = transformed[:, 1] - robot_y
        distances = np.hypot(dx, dy)
        angles = np.arctan2(dy, dx) - robot_yaw
        # 归一化角度到 [-pi, pi]
        angles = (angles + np.pi) % (2 * np.pi) - np.pi
        heights = transformed[:, 2]
        
        # 过滤条件: 在检测范围内、前方±90°、G1 行走高度范围
        valid_mask = (
            (distances < self.detection_range) &
            (np.abs(angles) < np.pi / 2) &
            (heights > self.obstacle_min_height) &
            (heights < self.obstacle_max_height)
        )
        filtered_pts = transformed[valid_mask]
        
        # 发布过滤后的点用于 RViz 可视化 + Action Server 障碍物反馈
        if len(filtered_pts) > 0:
            self.publish_filtered_points(filtered_pts)
            self._publish_dwa_obstacles(filtered_pts)
        else:
            # 无障碍物时发布空点云, 使 Action Server 能正确判定 OBSTACLE_CLEAR
            self._publish_dwa_obstacles(np.empty((0, 3), dtype=np.float32))
        
        # ---- Step 7: 将过滤后的点标记到 costmap 原始层 ----
        marked_count = 0
        for pt in filtered_pts:
            self.local_costmap.mark_obstacle(pt[0], pt[1])
            marked_count += 1
        
        # ---- Step 8: 距离变换 + 膨胀层 ----
        self.local_costmap.apply_inflation()
        
        # 标记 costmap 已就绪
        self.costmap_ready = True
        
        # ---- Step 9: 发布 OccupancyGrid 可视化 ----
        self._publish_costmap_visualization()
        
        if self.debug:
            self.get_logger().info(
                f"[Costmap] 原始点={len(points_list)}, "
                f"过滤后={len(filtered_pts)}, "
                f"标记栅格={marked_count}, "
                f"障碍栅格={self.local_costmap.obstacle_cell_count}",
                throttle_duration_sec=2.0
            )
    
    def _publish_costmap_visualization(self):
        """发布代价地图到 /dwa_local_costmap (OccupancyGrid 格式, RViz 可直接显示)"""
        costmap_msg = self.local_costmap.to_occupancy_grid_msg(
            stamp=self.get_clock().now().to_msg(),
            frame_id='odom'
        )
        self.costmap_pub.publish(costmap_msg)
    
    def _publish_dwa_obstacles(self, filtered_pts):
        """
        将过滤后的障碍物点发布为 PointCloud2 到 /dwa_obstacles

        目的: 修复与 point_to_point_action_server.py 的话题断链。
        Action Server 通过 read_points(field_names=("x","y","z")) 读取此消息,
        计算机器人到每个障碍物点的欧式距离, 取最小值更新 obstacle_proximity 等级。

        数据来源: update_costmap_from_pointcloud() 中经 TF变换 + 三重过滤
        (距离 < detection_range, 角度 ±90°, 高度在行走范围) 后的点, 坐标系为 odom。

        Args:
            filtered_pts: numpy array (N, 3), odom 坐标系下的障碍物点 [x, y, z]
        """
        header = self.pointcloud_data.header if self.pointcloud_data else None
        from std_msgs.msg import Header
        obs_header = Header()
        obs_header.stamp = self.get_clock().now().to_msg()
        obs_header.frame_id = 'odom'

        if len(filtered_pts) == 0:
            # 发布空点云 (0 个点), Action Server 的 read_points 返回空迭代器,
            # min_distance 保持 inf → obstacle_proximity = OBSTACLE_CLEAR
            empty_cloud = point_cloud2.create_cloud_xyz32(obs_header, [])
            self.dwa_obstacles_pub.publish(empty_cloud)
            return

        # 将 numpy (N,3) 转为 list of tuples (point_cloud2.create_cloud_xyz32 要求)
        pts_list = filtered_pts[:, :3].astype(np.float32).tolist()
        cloud_msg = point_cloud2.create_cloud_xyz32(obs_header, pts_list)
        self.dwa_obstacles_pub.publish(cloud_msg)
    
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
        基于 Costmap 分析障碍物分布, 智能选择逃逸方向

        原理: 将机器人周围 360° 分为 8 个扇区 (每个 45°),
        在每个扇区内采样 costmap 代价值, 计算平均代价。
        选择平均代价最低的扇区作为逃逸方向。

        相比原始方法 (遍历障碍物质心列表) 的改进:
        - 考虑了障碍物的完整形状和膨胀区域
        - 持久化障碍物不会因为单帧丢失而被忽略
        - 采样密度可控, 与障碍物数量无关

        Returns:
            best_escape_direction: float, 最佳逃逸方向 (弧度, odom 坐标系)
            should_reverse: bool, 是否需要后退 (前方三个扇区代价过高)
        """
        if not self.local_costmap.has_obstacles() or self.odom_data is None:
            return None, False
        
        robot_x = self.odom_data.pose.pose.position.x
        robot_y = self.odom_data.pose.pose.position.y
        orient = self.odom_data.pose.pose.orientation
        _, _, robot_yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        num_sectors = 8
        sector_angle = 2 * math.pi / num_sectors
        # 每个扇区的代价总和和采样计数
        sector_cost_sums = [0.0] * num_sectors
        sector_sample_counts = [0] * num_sectors
        
        # 在 2 米范围内、8 个扇区中采样 costmap 代价
        sample_step = self.local_costmap.resolution * 2  # 每隔 2 个栅格采样一次
        for dist in np.arange(sample_step, 2.0, sample_step):
            for angle_idx in range(num_sectors):
                # 每个扇区的中心角度 (相对机器人朝向)
                sector_center = -math.pi + (angle_idx + 0.5) * sector_angle
                # 在扇区内取 3 个采样方向
                for sub_angle in [-sector_angle * 0.3, 0, sector_angle * 0.3]:
                    abs_angle = robot_yaw + sector_center + sub_angle
                    sx = robot_x + dist * math.cos(abs_angle)
                    sy = robot_y + dist * math.sin(abs_angle)
                    cost = self.local_costmap.get_cost(sx, sy)
                    if cost != LocalCostmap.NO_INFORMATION:
                        sector_cost_sums[angle_idx] += cost
                        sector_sample_counts[angle_idx] += 1
        
        # 计算每个扇区的平均代价 (代价越低 = 越安全)
        sector_avg_costs = []
        for i in range(num_sectors):
            if sector_sample_counts[i] > 0:
                sector_avg_costs.append(sector_cost_sums[i] / sector_sample_counts[i])
            else:
                sector_avg_costs.append(0.0)  # 无数据的扇区视为安全
        
        # 选择平均代价最低的扇区 (最空旷的方向)
        best_sector = sector_avg_costs.index(min(sector_avg_costs))
        
        # 计算逃逸方向 (扇区中心的绝对角度)
        best_escape_angle = -math.pi + (best_sector + 0.5) * sector_angle
        best_escape_direction = self.normalize_angle(robot_yaw + best_escape_angle)
        
        # 判断是否需要后退: 前方三个扇区 (0, 1, 7) 的平均代价是否过高
        # 扇区编号: 0=正后方, 4=正前方 (相对于 -π 起始)
        # 调整为: 前方 = 扇区 3,4,5 (以正前方为中心)
        front_sectors = [3, 4, 5]  # 正前方及其两侧
        front_avg = sum(sector_avg_costs[s] for s in front_sectors) / len(front_sectors)
        should_reverse = front_avg > 150.0  # 平均代价超过 150 → 前方严重阻塞
        
        self.get_logger().info(
            f"[Costmap 扇区分析] 平均代价={[f'{c:.0f}' for c in sector_avg_costs]}, "
            f"最佳扇区={best_sector}, 是否后退={should_reverse}",
            throttle_duration_sec=1.0
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
        """
        基于 Costmap 检测前方是否有真实障碍物 (窄走廊适配版)

        设计目标:
        在窄走廊 (宽度仅比机器人宽 ~0.45m) 中, 走廊墙壁不应触发 DWA 避障模式。
        DWA 应仅被 "路径上的突发障碍" (行人、箱子等) 激活。
        走廊墙壁的安全距离由 costmap 膨胀梯度 + DWA 轨迹评分自动保障。

        原 ±90° 全向扫描的问题:
        ┌──────────────────────────────────┐
        │ 走廊墙壁  (LETHAL 254)            │
        │     ┌────────────────────┐       │ ← ±90° 扫描命中墙壁
        │     │  ↗↗↗ 机器人 ↗↗↗  │       │    → 每帧 ~200 个采样点超标
        │     │  → → → 前进 → → → │       │    → DWA 永久激活 → 振荡
        │     │  ↘↘↘         ↘↘↘  │       │
        │     └────────────────────┘       │
        │ 走廊墙壁  (LETHAL 254)            │
        └──────────────────────────────────┘

        修正: 窄锥角 (±30°) + 最小计数阈值
        ┌──────────────────────────────────┐
        │ 走廊墙壁                          │
        │            ╱ 30° ╲               │ ← 窄锥角不触及侧墙
        │     机器人 → → → → → 目标        │    → 仅检测前方路径上的障碍
        │            ╲ 30° ╱               │    → 走廊行走 = A* 模式
        │ 走廊墙壁                          │
        └──────────────────────────────────┘

        参数:
        - obstacle_detection_half_angle: 扫描半锥角 (默认 0.52 rad ≈ 30°)
        - min_obstacle_samples: 最小高代价采样点数 (默认 8)
          走廊边缘少量命中 (< 8) 不触发; 真实障碍产生密集命中 (>> 8) 触发
        - obstacle_cost_threshold: 代价门槛 (默认 253 = INSCRIBED)

        Returns:
            bool: True = 前方有真实障碍, DWA 应激活避障模式
        """
        if not self.local_costmap.has_obstacles() or self.odom_data is None:
            return False
        
        robot_x = self.odom_data.pose.pose.position.x
        robot_y = self.odom_data.pose.pose.position.y
        orient = self.odom_data.pose.pose.orientation
        _, _, robot_yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        
        high_cost_count = 0
        left_cost_sum = 0.0
        right_cost_sum = 0.0
        
        half_angle = self.obstacle_detection_half_angle  # 默认 ≈30° (0.52 rad)
        sample_step = self.local_costmap.resolution
        angle_step = 0.10  # 约 5.7°, 比旧值 0.15 更密以补偿窄锥角
        
        for dist in np.arange(sample_step, self.detection_range, sample_step):
            for angle_offset in np.arange(-half_angle, half_angle, angle_step):
                sample_x = robot_x + dist * math.cos(robot_yaw + angle_offset)
                sample_y = robot_y + dist * math.sin(robot_yaw + angle_offset)
                cost = self.local_costmap.get_cost(sample_x, sample_y)
                
                if cost >= self.obstacle_cost_threshold:
                    high_cost_count += 1
                    if angle_offset > 0:
                        left_cost_sum += cost
                    else:
                        right_cost_sum += cost
        
        # 只有高代价采样点数量超过阈值才判定为真实障碍
        # 走廊墙壁在窄锥角下仅产生少量边缘命中 (通常 < 5)
        # 真实障碍 (行人/箱子) 正对前方, 产生密集命中 (通常 > 20)
        obstacle_detected = (high_cost_count >= self.min_obstacle_samples)
        
        if obstacle_detected:
            if left_cost_sum > right_cost_sum * 1.2:
                self.preferred_turn_direction = -1
            elif right_cost_sum > left_cost_sum * 1.2:
                self.preferred_turn_direction = 1
            else:
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
        基于 Costmap 的轨迹碰撞风险评估 (替代原始逐障碍物距离检测)

        原理:
        对候选轨迹上的每个点, 直接查询 costmap 代价值 (O(1) 查找),
        根据代价值映射为碰撞风险分数。

        相比原始方法的改进:
        1. 代价值已包含膨胀梯度信息, 天然提供连续平滑惩罚
           (不再需要手动设计指数/反比例惩罚函数)
        2. O(n) 复杂度 (n=轨迹点数), 不依赖障碍物数量
        3. 障碍物形状被完整保留在栅格中, 不会因质心简化而遗漏

        Nav2 代价值与碰撞风险的映射:
        - cost = 254 (LETHAL):      必定碰撞, 极高惩罚
        - cost = 253 (INSCRIBED):   在机器人内切圆内, 极高惩罚
        - cost = 128-252:           接近障碍, 中等惩罚 (指数衰减)
        - cost = 1-127:             轻微接近, 低惩罚
        - cost = 0 (FREE):          无惩罚

        Args:
            path: 候选轨迹点列表 [(x, y), ...]
        Returns:
            float: 碰撞风险分数 (负值, 越小代表越危险)
        """
        if not self.local_costmap.has_obstacles():
            return 0.0
        
        collision_risk = 0.0
        
        for x, y in path:
            cost = self.local_costmap.get_cost(x, y)
            
            if cost >= LocalCostmap.LETHAL_OBSTACLE:
                # 致命障碍: 此轨迹直接通过障碍物, 给予极高惩罚
                collision_risk -= 500.0
            elif cost >= LocalCostmap.INSCRIBED_INFLATED:
                # 内切圆障碍: 机器人必定与障碍物碰撞
                collision_risk -= 300.0
            elif cost >= 128:
                # 高代价区域: 离障碍物很近, 指数级惩罚
                # 将代价 128-252 映射到 0-1 范围, 然后指数放大
                normalized = (cost - 128) / 124.0  # 0.0 ~ 1.0
                collision_risk -= 100.0 * (1.0 + normalized)
            elif cost >= 1:
                # 低代价区域 (膨胀区外围): 轻微惩罚, 鼓励保持距离
                collision_risk -= cost * 0.15
        
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
        """
        基于 Costmap 查找最近的安全航点 (替代原始质心距离检测)

        原理: 对每个候选航点, 查询其在 costmap 中的代价值。
        代价低于安全阈值 (< INSCRIBED=253) 的航点视为安全。

        相比原始方法 (逐航点检测到所有障碍物质心的距离) 的改进:
        - O(1) 查询替代 O(m) 遍历 (m=障碍物数量)
        - costmap 已包含膨胀信息, 无需手动设定 1m 安全距离
        - 障碍物形状完整, 不会因为质心偏移而误判

        Returns:
            PoseStamped: 最近的安全航点, 或 None
        """
        if not self.has_path or not self.odom_data:
            return None
        
        robot_x = self.odom_data.pose.pose.position.x
        robot_y = self.odom_data.pose.pose.position.y
        
        min_dist = float('inf')
        closest_idx = self.current_waypoint_index
        found_safe_waypoint = False
        # 安全代价阈值: 低于 INSCRIBED (253) 即认为可通行
        # 使用更保守的 128 (膨胀区中等代价) 确保有足够安全边距
        SAFE_COST_THRESHOLD = 128
        
        for i in range(self.current_waypoint_index, len(self.waypoints)):
            waypoint = self.waypoints[i]
            wx = waypoint.pose.position.x
            wy = waypoint.pose.position.y
            dist = math.hypot(wx - robot_x, wy - robot_y)
            
            # 使用 costmap 代价判断航点安全性
            cost = self.local_costmap.get_cost(wx, wy)
            is_safe = (cost < SAFE_COST_THRESHOLD)
            
            if is_safe:
                found_safe_waypoint = True
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i
        
        # 如果没有找到安全航点, 使用最终目标点
        if self.local_costmap.has_obstacles() and not found_safe_waypoint:
            self.get_logger().info("所有航点均在高代价区域, 使用最终目标点")
            return self.waypoints[-1]
        
        # 如果最近安全航点就是当前航点, 尝试向前跳过几个
        if closest_idx == self.current_waypoint_index and closest_idx + 3 < len(self.waypoints):
            for i in range(closest_idx + 3, len(self.waypoints)):
                waypoint = self.waypoints[i]
                cost = self.local_costmap.get_cost(
                    waypoint.pose.position.x, waypoint.pose.position.y
                )
                if cost < SAFE_COST_THRESHOLD:
                    self.get_logger().info(
                        f"跳至前方安全航点: 索引 {i}, 跳过 {i - closest_idx} 个点"
                    )
                    return self.waypoints[i]
            
            self.get_logger().info("前方无安全航点, 使用最终目标点")
            return self.waypoints[-1]
        
        return self.waypoints[closest_idx]
    
    # === Main Control Loop ===
    
    def movement_loop(self):
        """DWA 主控制循环 (10Hz, ROS2 Timer 回调)"""
        if self.odom_data is None:
            self.get_logger().warn("等待里程计数据...", throttle_duration_sec=2.0)
            return
        
        if not self.costmap_ready:
            self.get_logger().warn("等待 Costmap 就绪 (需要至少一帧点云)...", throttle_duration_sec=2.0)
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
                self.publish_obstacle_status(False, False)
                
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
                    self.publish_obstacle_status(False, False)
            
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
                self.publish_obstacle_status(True, True)
            
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
                self.publish_obstacle_status(False, False)
                
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
        marker.header.frame_id = "odom"
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
        """
        发布过滤后的点云可视化 (绿色点, 显示被 costmap 标记的障碍物点)

        Args:
            points: numpy array (N, 3) 或 list of [x, y, z]
        """
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        # 兼容 numpy array 和 list 格式
        pts = np.asarray(points)
        # 限制可视化点数, 避免大量点云卡顿 RViz
        max_viz_points = 500
        if len(pts) > max_viz_points:
            indices = np.random.choice(len(pts), max_viz_points, replace=False)
            pts = pts[indices]
        
        for i in range(len(pts)):
            p = Point()
            p.x = float(pts[i, 0])
            p.y = float(pts[i, 1])
            p.z = 0.0
            marker.points.append(p)
        
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
