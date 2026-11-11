#!/usr/bin/env python3
"""
多点巡航演示程序
演示如何使用 PointToPoint Action 实现多目标点导航

功能:
1. 按顺序访问多个航点
2. 实时监控导航状态
3. 障碍物警告
4. 错误处理和重试
5. 任务统计

作者: G1 Navigation Team
日期: 2025-11-05
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from g1_nav.action import PointToPoint
import time
from math import sqrt


class MultiPointPatrolClient(Node):
    """多点巡航客户端"""
    
    def __init__(self):
        super().__init__('multi_point_patrol_client')
        
        # Action 客户端
        self._action_client = ActionClient(self, PointToPoint, 'navigate_to_point')
        
        # 紧急停止发布者
        self._emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        
        # 任务统计
        self.completed_tasks = 0
        self.failed_tasks = 0
        self.total_distance = 0.0
        self.total_time = 0.0
        
        # 定义巡航航点 (x, y, 描述)
        self.waypoints = [
            (2.0, 1.0, "入口大厅"),
            (5.0, 3.0, "接待区"),
            (4.0, 6.0, "展示区"),
            (1.0, 5.0, "休息区"),
            (0.0, 0.0, "起点"),
        ]
        
        self.get_logger().info('🤖 多点巡航演示程序已启动')
        self.get_logger().info(f'📍 共有 {len(self.waypoints)} 个航点')
        
    def send_goal(self, task_id, x, y, description):
        """发送单个导航目标"""
        
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'🎯 任务 [{task_id}]: 前往 {description}')
        self.get_logger().info(f'📍 目标坐标: ({x:.2f}, {y:.2f})')
        self.get_logger().info(f'{"="*60}')
        
        # 等待服务器可用
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Action 服务器不可用!')
            return False
            
        # 构建目标
        goal_msg = PointToPoint.Goal()
        goal_msg.task_id = task_id
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.target_pose.pose.position.x = x
        goal_msg.target_pose.pose.position.y = y
        goal_msg.target_pose.pose.position.z = 0.0
        goal_msg.target_pose.pose.orientation.w = 1.0  # 无旋转
        
        # 发送目标
        self.get_logger().info('📤 发送导航目标...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ 目标被拒绝!')
            return False
            
        self.get_logger().info('✅ 目标已接受,开始导航...')
        
        # 等待结果
        start_time = time.time()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        elapsed = time.time() - start_time
        
        # 分析结果
        return self.process_result(result, elapsed, description)
        
    def feedback_callback(self, feedback_msg):
        """处理反馈信息"""
        feedback = feedback_msg.feedback
        
        # 构建状态信息
        state_names = {
            0: "规划中",
            1: "跟踪路径",
            2: "避障中",
            3: "恢复中",
            4: "可能卡住"
        }
        
        obstacle_names = {
            0: "安全",
            1: "注意",
            2: "危险"
        }
        
        state = state_names.get(feedback.navigation_state, "未知")
        obstacle = obstacle_names.get(feedback.obstacle_proximity, "未知")
        
        # 显示关键信息
        self.get_logger().info(
            f'📊 状态: {state} | '
            f'障碍物: {obstacle} | '
            f'距离: {feedback.distance_to_goal:.2f}m | '
            f'置信度: {feedback.localization_confidence:.2f} | '
            f'ETA: {feedback.estimated_time_remaining:.1f}s'
        )
        
        # 障碍物警告
        if feedback.obstacle_proximity == 2:  # CRITICAL
            self.get_logger().warning('⚠️ 检测到危险障碍物!')
            
        # 定位警告
        if feedback.localization_confidence < 0.4:
            self.get_logger().warning('⚠️ 定位置信度较低!')
            
    def process_result(self, result, elapsed, description):
        """处理导航结果"""
        
        status_messages = {
            0: "✅ 成功到达",
            1: "⏱️ 导航超时",
            2: "📍 定位丢失",
            3: "🚧 路径被阻挡",
            4: "🚫 目标不可达",
            5: "🔄 任务被抢占",
            6: "🛑 紧急停止"
        }
        
        status = status_messages.get(result.status_code, "❓ 未知状态")
        
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'📋 任务结果: {status}')
        self.get_logger().info(f'⏱️ 耗时: {elapsed:.2f} 秒')
        self.get_logger().info(f'📏 最终误差: {result.position_error:.3f} 米')
        
        if result.result ="success":
            self.get_logger().info(f'🎉 成功到达 {description}!')
            self.completed_tasks += 1
            self.total_time += elapsed
            return True
        else:
            self.get_logger().error(f'❌ 未能到达 {description}')
            self.failed_tasks += 1
            return False
            
    def run_patrol(self):
        """执行巡航任务"""
        
        self.get_logger().info('\n🚀 开始多点巡航任务...')
        time.sleep(2)  # 等待系统稳定
        
        start_time = time.time()
        
        for i, (x, y, desc) in enumerate(self.waypoints):
            task_id = f"PATROL_{i+1:02d}"
            
            success = self.send_goal(task_id, x, y, desc)
            
            if success:
                # 计算已行驶距离 (简化估算)
                if i > 0:
                    prev_x, prev_y, _ = self.waypoints[i-1]
                    distance = sqrt((x - prev_x)**2 + (y - prev_y)**2)
                    self.total_distance += distance
                    
                # 短暂停留
                self.get_logger().info('⏸️ 在航点停留 3 秒...')
                time.sleep(3)
            else:
                # 任务失败,询问是否继续
                self.get_logger().warning(f'⚠️ 航点 {i+1} 失败,是否继续?')
                # 在实际应用中,这里可以添加用户确认逻辑
                # 演示程序中自动继续
                time.sleep(2)
                
        # 显示统计信息
        self.show_statistics(time.time() - start_time)
        
    def show_statistics(self, total_elapsed):
        """显示任务统计"""
        
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info('📊 巡航任务统计报告')
        self.get_logger().info(f'{"="*60}')
        self.get_logger().info(f'✅ 成功任务: {self.completed_tasks}/{len(self.waypoints)}')
        self.get_logger().info(f'❌ 失败任务: {self.failed_tasks}/{len(self.waypoints)}')
        self.get_logger().info(f'📏 总行驶距离: {self.total_distance:.2f} 米')
        self.get_logger().info(f'⏱️ 总耗时: {total_elapsed:.2f} 秒')
        
        if self.completed_tasks > 0:
            avg_time = self.total_time / self.completed_tasks
            avg_speed = self.total_distance / self.total_time if self.total_time > 0 else 0
            self.get_logger().info(f'📈 平均任务时间: {avg_time:.2f} 秒')
            self.get_logger().info(f'🚀 平均速度: {avg_speed:.3f} m/s')
            
        success_rate = (self.completed_tasks / len(self.waypoints)) * 100
        self.get_logger().info(f'🎯 成功率: {success_rate:.1f}%')
        self.get_logger().info(f'{"="*60}\n')
        

def main(args=None):
    """主函数"""
    
    rclpy.init(args=args)
    
    try:
        client = MultiPointPatrolClient()
        
        # 等待用户准备
        print("\n" + "="*60)
        print("🤖 G1 机器人多点巡航演示程序")
        print("="*60)
        print("\n请确保:")
        print("  1. ✅ fusion_ros2.launch.py 已启动")
        print("  2. ✅ 已在 RViz 中设置初始位姿")
        print("  3. ✅ g1_nav_action_server.py 已运行")
        print("\n按 Enter 键开始巡航...")
        input()
        
        # 执行巡航
        client.run_patrol()
        
        print("\n✅ 巡航任务完成!")
        
    except KeyboardInterrupt:
        print('\n\n⚠️ 用户中断,发送紧急停止...')
        try:
            client._emergency_stop_pub.publish(Bool(data=True))
            time.sleep(0.5)
        except:
            pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
