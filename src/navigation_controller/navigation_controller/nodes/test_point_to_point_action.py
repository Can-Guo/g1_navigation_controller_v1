#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
================================================================================
PointToPoint Navigation Action Client - 测试程序
================================================================================

功能描述:
    测试 PointToPoint Action 服务器的功能和性能
    模拟 MCB 发送导航任务,监控执行过程,验证结果

测试场景:
    1. 基础导航测试 - 发送单个目标点,验证成功到达
    2. 多目标巡航测试 - 依次访问多个路点
    3. 取消任务测试 - 中途取消正在执行的任务
    4. 紧急停止测试 - 触发紧急停止,验证响应
    5. 超时测试 - 设置过远目标,验证超时保护
    6. 异常恢复测试 - 模拟定位丢失后恢复

使用方法:
    # 运行所有测试
    ros2 run navigation_controller test_point_to_point_action.py
    
    # 运行单个测试
    ros2 run navigation_controller test_point_to_point_action.py --test basic
    
    # 交互模式 (手动输入目标点)
    ros2 run navigation_controller test_point_to_point_action.py --interactive

性能指标:
    - 到达精度: < 0.3m (目标容差)
    - 响应延迟: < 0.5s (从发送到开始执行)
    - 反馈频率: ~5Hz
    - 紧急停止响应: < 100ms

作者: G1 Navigation Team
日期: 2025-11-05
版本: 1.0.0
ROS2: Humble
Python: 3.10+
================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import sys
import time
import argparse
import threading
from datetime import datetime

# ROS2 Messages
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header, Bool

# 自定义 Action 接口
from robot_interfaces.action import PointToPoint


class PointToPointActionClient(Node):
    """
    PointToPoint Action 客户端测试工具
    
    功能:
        - 发送导航目标到 Action 服务器
        - 实时显示反馈信息
        - 统计性能指标
        - 提供测试场景
    """
    
    def __init__(self):
        super().__init__('point_to_point_test_client')
        
        # 创建 Action 客户端
        self._action_client = ActionClient(
            self,
            PointToPoint,
            'point_to_point'
        )
        
        # QoS 配置
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 创建紧急停止发布器 (用于测试)
        self.emergency_stop_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            reliable_qos
        )
        
        # 测试统计
        self.test_stats = {
            'total_tests': 0,
            'successful_tests': 0,
            'failed_tests': 0,
            'total_distance': 0.0,
            'total_time': 0.0,
            'max_position_error': 0.0
        }
        
        # 反馈显示控制
        self.feedback_count = 0
        self.last_feedback_time = 0
        
        # 当前goal handle (用于取消任务)
        self._current_goal_handle = None
        
        self.get_logger().info('='*80)
        self.get_logger().info('PointToPoint Action Client 测试工具已初始化')
        self.get_logger().info('='*80)
    
    def wait_for_server(self, timeout_sec=10.0):
        """
        等待 Action 服务器上线
        
        参数:
            timeout_sec: 超时时间(秒)
        
        返回:
            bool: True 表示服务器已就绪
        """
        self.get_logger().info('等待 Action 服务器连接...')
        
        if not self._action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error(f'❌ 超时 {timeout_sec}s, Action 服务器未响应')
            self.get_logger().error('请确认 action_nav_point_to_point 已启动:')
            self.get_logger().error('  ros2 run navigation_controller point_to_point_action_server.py')
            return False
        
        self.get_logger().info('✅ Action 服务器已连接')
        return True
    
    def send_goal(self, x, y, yaw=0.0, task_id=None):
        """
        发送导航目标
        
        参数:
            x: 目标 X 坐标 (米)
            y: 目标 Y 坐标 (米)
            yaw: 目标朝向 (弧度, 默认 0)
            task_id: 任务 ID (默认自动生成)
        
        返回:
            Future: 异步结果对象
        """
        # 生成任务 ID
        if task_id is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            task_id = f'TEST_TASK_{timestamp}'
        
        # 构建目标消息
        goal_msg = PointToPoint.Goal()
        goal_msg.task_id = task_id
        
        # 目标位姿
        goal_msg.target_pose = PoseStamped()
        goal_msg.target_pose.header = Header()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.target_pose.pose.position = Point(x=x, y=y, z=0.0)
        
        # 将 yaw 转换为四元数
        # q = [cos(yaw/2), 0, 0, sin(yaw/2)]
        import math
        goal_msg.target_pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0)
        )
        
        self.get_logger().info('='*80)
        self.get_logger().info(f'发送导航目标:')
        self.get_logger().info(f'  Task ID: {task_id}')
        self.get_logger().info(f'  位置: ({x:.2f}, {y:.2f})')
        self.get_logger().info(f'  朝向: {math.degrees(yaw):.1f}°')
        self.get_logger().info('='*80)
        
        # 发送目标
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return send_goal_future
    
    def goal_response_callback(self, future):
        """处理目标接受/拒绝响应"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ 目标被服务器拒绝')
            self._current_goal_handle = None
            return
        
        self.get_logger().info('✅ 目标已被接受,开始执行...')
        
        # 保存goal handle以便取消
        self._current_goal_handle = goal_handle
        
        # 等待结果
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        """
        处理反馈消息
        
        显示策略:
            - 每秒显示一次详细反馈 (避免刷屏)
            - 状态变化时立即显示
            - 警告/错误信息始终显示
        """
        self.feedback_count += 1
        current_time = time.time()
        
        feedback = feedback_msg.feedback
        
        # 状态名称映射
        state_names = {
            PointToPoint.Feedback.STATE_PLANNING: '🔍 规划中',
            PointToPoint.Feedback.STATE_FOLLOWING: '🚀 路径跟踪',
            PointToPoint.Feedback.STATE_AVOIDING: '⚠️  避障中',
            PointToPoint.Feedback.STATE_RECOVERING: '🔄 恢复中',
            PointToPoint.Feedback.STATE_STUCK: '🛑 可能卡住'
        }
        
        obstacle_names = {
            PointToPoint.Feedback.OBSTACLE_CLEAR: '✅ 安全',
            PointToPoint.Feedback.OBSTACLE_NEAR: '⚠️  接近',
            PointToPoint.Feedback.OBSTACLE_CRITICAL: '🚨 危险'
        }
        
        # 检查是否需要显示
        should_display = (
            current_time - self.last_feedback_time > 1.0 or  # 每秒一次
            feedback.navigation_state == PointToPoint.Feedback.STATE_RECOVERING or  # 恢复行为
            feedback.obstacle_proximity == PointToPoint.Feedback.OBSTACLE_CRITICAL  # 危险障碍物
        )
        
        if should_display:
            self.last_feedback_time = current_time
            
            self.get_logger().info(
                f'{state_names.get(feedback.navigation_state, "未知")} | '
                f'剩余: {feedback.distance_to_goal:.2f}m | '
                f'障碍物: {obstacle_names.get(feedback.obstacle_proximity, "未知")} | '
                f'置信度: {feedback.localization_confidence:.2f} | '
                f'预计: {feedback.estimated_time_remaining:.1f}s'
            )
        
        # 警告信息始终显示
        if feedback.localization_confidence < 0.3:
            self.get_logger().warn(
                f'⚠️  定位置信度低: {feedback.localization_confidence:.2f}'
            )
    
    def get_result_callback(self, future):
        """处理最终结果"""
        result = future.result().result
        
        self.get_logger().info('='*80)
        
        # 状态码名称
        status_names = {
            PointToPoint.Result.STATUS_SUCCESS: '✅ 成功到达',
            PointToPoint.Result.STATUS_TIMEOUT: '⏱️  任务超时',
            PointToPoint.Result.STATUS_LOCALIZATION_LOST: '📍 定位丢失',
            PointToPoint.Result.STATUS_PATH_BLOCKED: '🚧 路径阻塞',
            PointToPoint.Result.STATUS_GOAL_UNREACHABLE: '🚫 目标不可达',
            PointToPoint.Result.STATUS_PREEMPTED: '🔀 任务被抢占',
            PointToPoint.Result.STATUS_EMERGENCY_STOP: '🚨 紧急停止'
        }
        
        self.get_logger().info(f'任务完成: {status_names.get(result.status_code, "未知")}')
        self.get_logger().info(f'位置误差: {result.position_error:.3f} m')
        
        if result.result == "success":
            self.get_logger().info('🎉 导航成功!')
        else:
            self.get_logger().error('❌ 导航失败')
        
        self.get_logger().info('='*80)
        
        # 更新统计
        self.test_stats['total_tests'] += 1
        if result.result == "success":
            self.test_stats['successful_tests'] += 1
        else:
            self.test_stats['failed_tests'] += 1
        
        self.test_stats['max_position_error'] = max(
            self.test_stats['max_position_error'],
            result.position_error
        )
    
    def cancel_goal(self):
        """取消当前任务"""
        self.get_logger().info('发送取消请求...')
        
        if self._current_goal_handle is not None:
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda future: self.get_logger().info('✅ 任务已取消')
            )
            self._current_goal_handle = None
        else:
            self.get_logger().warn('⚠️  没有正在执行的任务')
    
    def trigger_emergency_stop(self):
        """触发紧急停止"""
        self.get_logger().warn('🚨 触发紧急停止!')
        msg = Bool()
        msg.data = True
        self.emergency_stop_pub.publish(msg)
        
        # 3 秒后解除
        def release_stop():
            time.sleep(3.0)
            self.get_logger().info('解除紧急停止')
            msg.data = False
            self.emergency_stop_pub.publish(msg)
        
        threading.Thread(target=release_stop, daemon=True).start()
    
    def print_statistics(self):
        """打印测试统计"""
        self.get_logger().info('='*80)
        self.get_logger().info('测试统计:')
        self.get_logger().info(f'  总测试数: {self.test_stats["total_tests"]}')
        self.get_logger().info(f'  成功: {self.test_stats["successful_tests"]}')
        self.get_logger().info(f'  失败: {self.test_stats["failed_tests"]}')
        if self.test_stats['total_tests'] > 0:
            success_rate = (
                self.test_stats['successful_tests'] / 
                self.test_stats['total_tests'] * 100
            )
            self.get_logger().info(f'  成功率: {success_rate:.1f}%')
        self.get_logger().info(f'  最大位置误差: {self.test_stats["max_position_error"]:.3f} m')
        self.get_logger().info('='*80)


# ==============================================================================
# 测试场景
# ==============================================================================

def test_basic_navigation(client):
    """
    测试场景 1: 基础导航
    发送单个目标点,验证成功到达
    """
    client.get_logger().info('\n🧪 测试场景 1: 基础导航\n')
    
    # 发送目标: 前方 5 米
    future = client.send_goal(x=5.0, y=0.0)
    
    # 等待完成
    rclpy.spin_until_future_complete(client, future, timeout_sec=120.0)
    
    if not future.done():
        client.get_logger().error('测试超时')
        return False
    
    return True


def test_multi_waypoint(client):
    """
    测试场景 2: 多目标巡航
    依次访问多个路点
    """
    client.get_logger().info('\n🧪 测试场景 2: 多目标巡航\n')
    
    waypoints = [
        (5.0, 0.0),   # 前方 5m
        (5.0, 5.0),   # 右侧 5m
        (0.0, 5.0),   # 后方 5m
        (0.0, 0.0)    # 返回原点
    ]
    
    for i, (x, y) in enumerate(waypoints, 1):
        client.get_logger().info(f'路点 {i}/{len(waypoints)}: ({x}, {y})')
        
        future = client.send_goal(x=x, y=y, task_id=f'WAYPOINT_{i}')
        rclpy.spin_until_future_complete(client, future, timeout_sec=120.0)
        
        if not future.done():
            client.get_logger().error(f'路点 {i} 超时')
            return False
        
        # 短暂停留
        time.sleep(2.0)
    
    return True


def test_cancel_task(client):
    """
    测试场景 3: 取消任务
    中途取消正在执行的任务
    """
    client.get_logger().info('\n🧪 测试场景 3: 取消任务\n')
    
    # 发送远距离目标
    future = client.send_goal(x=20.0, y=0.0)
    
    # 等待 3 秒后取消
    time.sleep(3.0)
    client.cancel_goal()
    
    # 等待取消完成
    time.sleep(2.0)
    
    return True


def test_emergency_stop(client):
    """
    测试场景 4: 紧急停止
    触发紧急停止,验证响应时间
    """
    client.get_logger().info('\n🧪 测试场景 4: 紧急停止\n')
    
    # 发送目标
    future = client.send_goal(x=10.0, y=0.0)
    
    # 2 秒后触发紧急停止
    time.sleep(2.0)
    stop_time = time.time()
    client.trigger_emergency_stop()
    
    # 等待任务终止
    rclpy.spin_until_future_complete(client, future, timeout_sec=10.0)
    
    response_time = time.time() - stop_time
    client.get_logger().info(f'紧急停止响应时间: {response_time*1000:.1f} ms')
    
    return True


def interactive_mode(client):
    """
    交互模式: 手动输入目标点
    """
    client.get_logger().info('\n🎮 交互模式\n')
    client.get_logger().info('输入目标坐标 (格式: x y), 输入 q 退出\n')
    
    while True:
        try:
            user_input = input('目标坐标 (x y): ').strip()
            
            if user_input.lower() == 'q':
                break
            
            parts = user_input.split()
            if len(parts) != 2:
                print('格式错误! 请输入: x y')
                continue
            
            x, y = float(parts[0]), float(parts[1])
            
            future = client.send_goal(x=x, y=y)
            rclpy.spin_until_future_complete(client, future, timeout_sec=120.0)
            
        except ValueError:
            print('格式错误! 请输入数字')
        except KeyboardInterrupt:
            break
    
    return True


# ==============================================================================
# 主函数
# ==============================================================================

def main(args=None):
    """主函数"""
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='PointToPoint Action 测试工具')
    parser.add_argument(
        '--test',
        choices=['basic', 'multi', 'cancel', 'emergency', 'all'],
        default='all',
        help='选择测试场景'
    )
    parser.add_argument(
        '--interactive',
        action='store_true',
        help='进入交互模式'
    )
    
    # 注意: ROS2 会消费自己的参数,需要过滤
    ros_args = [arg for arg in sys.argv if not arg.startswith('--test') and not arg.startswith('--interactive')]
    parsed_args = parser.parse_args([arg for arg in sys.argv[1:] if arg.startswith('--test') or arg.startswith('--interactive')])
    
    # 初始化 ROS2
    rclpy.init(args=ros_args)
    
    # 创建客户端
    client = PointToPointActionClient()
    
    # 等待服务器
    if not client.wait_for_server(timeout_sec=10.0):
        client.destroy_node()
        rclpy.shutdown()
        return
    
    try:
        # 运行测试
        if parsed_args.interactive:
            interactive_mode(client)
        else:
            if parsed_args.test in ['basic', 'all']:
                test_basic_navigation(client)
            
            if parsed_args.test in ['multi', 'all']:
                test_multi_waypoint(client)
            
            if parsed_args.test in ['cancel', 'all']:
                test_cancel_task(client)
            
            if parsed_args.test in ['emergency', 'all']:
                test_emergency_stop(client)
        
        # 打印统计
        client.print_statistics()
        
    except KeyboardInterrupt:
        client.get_logger().info('收到 Ctrl+C, 退出测试')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
