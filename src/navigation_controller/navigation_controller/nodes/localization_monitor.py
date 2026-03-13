#!/usr/bin/env python3
"""
G1机器人定位监控和诊断工具
用于实时监控TF发布频率、定位质量和系统性能

作者: SLAM & ROS2 专家
日期: 2025-11-06
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32
import time
from collections import deque
import math


class LocalizationMonitor(Node):
    """监控定位系统性能"""
    
    def __init__(self):
        super().__init__('localization_monitor')
        
        # QoS配置
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅定位话题
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/map',
            self.odom_callback,
            sensor_qos
        )
        
        # 统计变量
        self.odom_timestamps = deque(maxlen=100)
        self.last_position = None
        self.total_distance = 0.0
        
        # 创建定时器输出统计信息
        self.stats_timer = self.create_timer(2.0, self.print_statistics)
        
        # 启动时间
        self.start_time = time.time()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("G1定位监控工具已启动")
        self.get_logger().info("=" * 60)
        
    def odom_callback(self, msg):
        """处理定位消息"""
        current_time = time.time()
        self.odom_timestamps.append(current_time)
        
        # 计算移动距离
        current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.last_position is not None:
            dx = current_pos[0] - self.last_position[0]
            dy = current_pos[1] - self.last_position[1]
            distance = math.sqrt(dx*dx + dy*dy)
            self.total_distance += distance
        
        self.last_position = current_pos
        
    def print_statistics(self):
        """打印统计信息"""
        if len(self.odom_timestamps) < 2:
            self.get_logger().warn("等待定位数据...")
            return
        
        # 计算频率
        time_diffs = []
        for i in range(1, len(self.odom_timestamps)):
            time_diffs.append(self.odom_timestamps[i] - self.odom_timestamps[i-1])
        
        avg_interval = sum(time_diffs) / len(time_diffs)
        avg_freq = 1.0 / avg_interval if avg_interval > 0 else 0
        min_interval = min(time_diffs)
        max_interval = max(time_diffs)
        max_freq = 1.0 / min_interval if min_interval > 0 else 0
        min_freq = 1.0 / max_interval if max_interval > 0 else 0
        
        # 计算标准差
        variance = sum((x - avg_interval) ** 2 for x in time_diffs) / len(time_diffs)
        std_dev = math.sqrt(variance)
        
        # 运行时间
        elapsed = time.time() - self.start_time
        
        # 评估健康状态
        if avg_freq >= 8.0:
            health = "✅ 优秀"
            color = "\033[92m"  # 绿色
        elif avg_freq >= 5.0:
            health = "⚠️ 良好"
            color = "\033[93m"  # 黄色
        elif avg_freq >= 3.0:
            health = "⚠️ 一般"
            color = "\033[93m"  # 黄色
        else:
            health = "❌ 差"
            color = "\033[91m"  # 红色
        
        reset_color = "\033[0m"
        
        # 输出统计
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"{color}定位频率健康度: {health}{reset_color}")
        self.get_logger().info("-" * 60)
        self.get_logger().info(f"平均频率: {avg_freq:.2f} Hz")
        self.get_logger().info(f"频率范围: {min_freq:.2f} - {max_freq:.2f} Hz")
        self.get_logger().info(f"时间间隔: 平均{avg_interval*1000:.1f}ms, "
                             f"最小{min_interval*1000:.1f}ms, "
                             f"最大{max_interval*1000:.1f}ms")
        self.get_logger().info(f"稳定性: 标准差 {std_dev*1000:.1f}ms")
        self.get_logger().info(f"总移动距离: {self.total_distance:.2f} 米")
        self.get_logger().info(f"运行时间: {int(elapsed)}秒")
        
        # 诊断建议
        if avg_freq < 5.0:
            self.get_logger().warn("⚠️ 诊断建议:")
            self.get_logger().warn("  1. 定位频率过低，可能导致RViz显示卡顿")
            self.get_logger().warn("  2. 建议查看 G1_LOCALIZATION_DIAGNOSIS.md")
            self.get_logger().warn("  3. 考虑实施方案一：分离TF发布线程")
        
        if std_dev > 0.1:
            self.get_logger().warn("⚠️ 定位更新不稳定，时间间隔波动大")
            self.get_logger().warn("  可能原因：点云配准计算时间波动")
        
        self.get_logger().info("=" * 60)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = LocalizationMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
