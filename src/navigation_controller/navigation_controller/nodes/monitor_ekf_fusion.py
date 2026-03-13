#!/usr/bin/env python3
"""
IMU融合系统验证工具
验证 EKF 融合节点的运行状态和数据质量
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray
import time
from collections import deque
import math


class EKFFusionMonitor(Node):
    def __init__(self):
        super().__init__('ekf_fusion_monitor')
        
        # 数据缓冲区
        self.ekf_data = deque(maxlen=50)
        self.lidar_data = deque(maxlen=50)
        self.imu_data = deque(maxlen=50)
        
        # 统计计数
        self.ekf_count = 0
        self.lidar_count = 0
        self.imu_count = 0
        
        # 订阅话题
        self.sub_ekf = self.create_subscription(
            Odometry, '/odometry/ekf', self.ekf_callback, 10)
        self.sub_lidar = self.create_subscription(
            Odometry, '/odometry/map', self.lidar_callback, 10)
        self.sub_imu = self.create_subscription(
            Imu, '/livox/imu', self.imu_callback, 10)
        self.sub_diag = self.create_subscription(
            DiagnosticArray, '/diagnostics', self.diag_callback, 10)
        
        # 定时打印统计信息
        self.timer = self.create_timer(2.0, self.print_stats)
        
        self.get_logger().info('🔍 EKF融合监控已启动')
        self.get_logger().info('监控话题: /odometry/ekf, /odometry/map, /livox/imu')
    
    def ekf_callback(self, msg):
        self.ekf_count += 1
        self.ekf_data.append({
            'time': time.time(),
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'orientation': msg.pose.pose.orientation
        })
    
    def lidar_callback(self, msg):
        self.lidar_count += 1
        self.lidar_data.append({
            'time': time.time(),
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        })
    
    def imu_callback(self, msg):
        self.imu_count += 1
        self.imu_data.append({
            'time': time.time(),
            'angular_z': msg.angular_velocity.z,
            'accel_x': msg.linear_acceleration.x
        })
    
    def diag_callback(self, msg):
        # 检查 EKF 诊断信息
        for status in msg.status:
            if 'ekf' in status.name.lower():
                level_str = ['OK', 'WARN', 'ERROR', 'STALE'][status.level]
                if status.level > 0:  # 非OK状态
                    self.get_logger().warn(
                        f'EKF诊断 [{level_str}]: {status.message}')
    
    def print_stats(self):
        """打印统计信息"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('📊 IMU融合系统状态监控')
        self.get_logger().info('='*60)
        
        # 频率统计
        self.get_logger().info(f'数据接收频率 (过去2秒):')
        self.get_logger().info(f'  ✓ EKF融合输出:  {self.ekf_count/2:.1f} Hz  {"✓ 正常" if self.ekf_count > 80 else "⚠️ 偏低"}')
        self.get_logger().info(f'  ✓ LiDAR定位:     {self.lidar_count/2:.1f} Hz  {"✓ 正常" if self.lidar_count > 15 else "⚠️ 偏低"}')
        self.get_logger().info(f'  ✓ IMU数据:       {self.imu_count/2:.1f} Hz  {"✓ 正常" if self.imu_count > 150 else "⚠️ 偏低"}')
        
        # 重置计数
        self.ekf_count = 0
        self.lidar_count = 0
        self.imu_count = 0
        
        # 数据质量检查
        if len(self.ekf_data) > 10 and len(self.lidar_data) > 5:
            # 计算 EKF 与 LiDAR 的位置偏差
            ekf_latest = self.ekf_data[-1]
            lidar_latest = self.lidar_data[-1]
            
            dx = ekf_latest['x'] - lidar_latest['x']
            dy = ekf_latest['y'] - lidar_latest['y']
            pos_diff = math.sqrt(dx**2 + dy**2)
            
            self.get_logger().info(f'\n数据质量分析:')
            self.get_logger().info(f'  EKF vs LiDAR 位置偏差: {pos_diff:.3f} m  {"✓ 正常" if pos_diff < 0.1 else "⚠️ 偏大"}')
            
            if pos_diff > 0.2:
                self.get_logger().warn(
                    f'⚠️  EKF与LiDAR偏差过大! 可能原因:\n'
                    f'    1. IMU漂移严重\n'
                    f'    2. LiDAR定位质量差\n'
                    f'    3. EKF参数需要调优')
        
        # IMU数据质量
        if len(self.imu_data) > 20:
            recent_imu = list(self.imu_data)[-20:]
            avg_angular = sum(abs(d['angular_z']) for d in recent_imu) / len(recent_imu)
            
            self.get_logger().info(f'  IMU角速度均值:        {avg_angular:.3f} rad/s')
            if avg_angular > 0.02:
                self.get_logger().info(f'    → 机器人正在旋转')
            elif avg_angular > 0.005:
                self.get_logger().warn(f'    → ⚠️ IMU零点偏差偏大,建议校准')
            else:
                self.get_logger().info(f'    → ✓ 静止状态,IMU质量良好')
        
        self.get_logger().info('='*60 + '\n')


def main():
    rclpy.init()
    monitor = EKFFusionMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
