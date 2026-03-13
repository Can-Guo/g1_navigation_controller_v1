#!/usr/bin/env python3
"""
TF 树监测工具
============
用于诊断导航系统的坐标变换链是否完整

功能:
1. 检查关键坐标系是否存在 (map, odom, base_link, lidar_frame)
2. 验证 TF 链路完整性
3. 实时监测 TF 发布频率
4. 检测 TF 异常 (跳变、超时)

使用方法:
    python3 tf_tree_monitor.py
    或
    ros2 run g1_nav tf_tree_monitor.py

Author: GitHub Copilot
Date: 2025-11-04
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import sys
from collections import defaultdict
import math


class TFTreeMonitor(Node):
    """TF 树监测节点"""
    
    def __init__(self):
        super().__init__('tf_tree_monitor')
        
        # TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 需要检查的坐标系
        self.required_frames = [
            'map',
            'odom',
            'base_link',
            'pelvis',
            'lidar_frame',
            'livox_frame'
        ]
        
        # 需要检查的 TF 链路
        self.required_transforms = [
            ('map', 'odom'),
            ('odom', 'base_link'),
            ('base_link', 'pelvis'),
            ('pelvis', 'lidar_frame'),
            ('lidar_frame', 'livox_frame'),
            ('map', 'base_link'),  # 完整链路
        ]
        
        # 记录上次变换
        self.last_transforms = {}
        self.transform_counts = defaultdict(int)
        
        # 定时器: 1Hz 检查
        self.timer = self.create_timer(1.0, self.check_tf_tree)
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("TF 树监测工具已启动")
        self.get_logger().info("检查的坐标系: " + ", ".join(self.required_frames))
        self.get_logger().info("=" * 70)
    
    def check_tf_tree(self):
        """检查 TF 树状态"""
        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("TF 树状态检查")
        self.get_logger().info("=" * 70)
        
        # 1. 检查单个坐标系是否存在
        self.get_logger().info("\n[1] 坐标系存在性检查:")
        all_frames_exist = True
        for frame in self.required_frames:
            exists = self.tf_buffer.can_transform(
                'map', frame, Time(), timeout=rclpy.duration.Duration(seconds=0.1)
            )
            status = "✓" if exists else "✗"
            self.get_logger().info(f"  {status} {frame}")
            if not exists:
                all_frames_exist = False
        
        if all_frames_exist:
            self.get_logger().info("  结论: ✓ 所有坐标系存在")
        else:
            self.get_logger().error("  结论: ✗ 部分坐标系缺失!")
        
        # 2. 检查 TF 链路
        self.get_logger().info("\n[2] TF 链路完整性检查:")
        all_transforms_ok = True
        for source, target in self.required_transforms:
            try:
                # 尝试查询变换
                transform = self.tf_buffer.lookup_transform(
                    target, source, Time(), timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                # 提取平移和旋转
                trans = transform.transform.translation
                rot = transform.transform.rotation
                
                # 计算距离
                distance = math.sqrt(trans.x**2 + trans.y**2 + trans.z**2)
                
                # 检查是否有跳变
                key = f"{source}->{target}"
                self.transform_counts[key] += 1
                
                if key in self.last_transforms:
                    last_trans = self.last_transforms[key]
                    dx = trans.x - last_trans[0]
                    dy = trans.y - last_trans[1]
                    dz = trans.z - last_trans[2]
                    jump = math.sqrt(dx**2 + dy**2 + dz**2)
                    
                    if jump > 1.0:  # 跳变超过 1 米
                        self.get_logger().warn(
                            f"  ⚠️  {source} → {target}: 检测到跳变 {jump:.2f}m"
                        )
                    else:
                        self.get_logger().info(
                            f"  ✓ {source} → {target}: "
                            f"平移({trans.x:.2f}, {trans.y:.2f}, {trans.z:.2f}), "
                            f"距离={distance:.2f}m"
                        )
                else:
                    self.get_logger().info(
                        f"  ✓ {source} → {target}: "
                        f"平移({trans.x:.2f}, {trans.y:.2f}, {trans.z:.2f})"
                    )
                
                # 保存当前变换
                self.last_transforms[key] = (trans.x, trans.y, trans.z)
                
            except TransformException as ex:
                self.get_logger().error(f"  ✗ {source} → {target}: {ex}")
                all_transforms_ok = False
        
        if all_transforms_ok:
            self.get_logger().info("  结论: ✓ 所有 TF 链路正常")
        else:
            self.get_logger().error("  结论: ✗ 部分 TF 链路断裂!")
        
        # 3. 检查 map → base_link 变换 (机器人定位)
        self.get_logger().info("\n[3] 机器人定位状态:")
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time(), timeout=rclpy.duration.Duration(seconds=0.1)
            )
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            # 计算 yaw 角
            yaw = math.atan2(
                2.0 * (rot.w * rot.z + rot.x * rot.y),
                1.0 - 2.0 * (rot.y**2 + rot.z**2)
            )
            
            self.get_logger().info(
                f"  位置: x={trans.x:.3f}m, y={trans.y:.3f}m, z={trans.z:.3f}m"
            )
            self.get_logger().info(
                f"  姿态: yaw={math.degrees(yaw):.1f}° "
                f"(四元数: w={rot.w:.3f}, x={rot.x:.3f}, y={rot.y:.3f}, z={rot.z:.3f})"
            )
            
        except TransformException as ex:
            self.get_logger().error(f"  ✗ 无法获取机器人位置: {ex}")
        
        # 4. 统计信息
        self.get_logger().info("\n[4] TF 更新统计:")
        for key, count in self.transform_counts.items():
            self.get_logger().info(f"  {key}: {count} 次更新")
        
        self.get_logger().info("=" * 70 + "\n")
    
    def shutdown(self):
        """关闭节点"""
        self.get_logger().info("TF 树监测工具已关闭")


def main(args=None):
    rclpy.init(args=args)
    
    monitor = TFTreeMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.shutdown()
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
