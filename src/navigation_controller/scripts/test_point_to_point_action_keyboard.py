#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
================================================================================
PointToPoint Navigation - 键盘控制版本
================================================================================

功能描述:
    通过键盘数字键快速控制机器人导航到预设路点
    支持从waypoints.yaml加载配置

使用方法:
    ros2 run navigation_controller waypoint_keyboard_control.py

键盘控制:
    0 - home_for_safety      (安全回家点)
    1 - pick_empty_box       (取空箱点)
    2 - place_empty_box      (放空箱点)
    3 - pick_product_box     (取产品箱点)
    4 - place_product_box    (放产品箱点)
    5 - ng                   (NG点)
    H - 显示帮助信息
    Q - 退出程序

作者: G1 Navigation Team
日期: 2025-11-25
版本: 1.0.0
================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import yaml
import threading
from pathlib import Path
from datetime import datetime

# ROS2 Messages
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header

# 自定义 Action 接口
from robot_interfaces.action import PointToPoint

# ANSI 颜色代码
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    END = '\033[0m'
    BOLD = '\033[1m'


class WaypointKeyboardController(Node):
    """
    路点键盘控制器
    
    功能:
        - 加载waypoints.yaml配置
        - 数字键快速导航
        - 实时反馈显示
    """
    
    def __init__(self, waypoints_file):
        super().__init__('waypoint_keyboard_controller')
        
        # 加载路点配置
        self.waypoints = self.load_waypoints(waypoints_file)
        if not self.waypoints:
            self.get_logger().error('❌ 路点配置加载失败')
            raise RuntimeError('Failed to load waypoints')
        
        # 创建 Action 客户端
        self._action_client = ActionClient(
            self,
            PointToPoint,
            'action_nav_point_to_point'
        )
        
        # 当前任务状态
        self._current_goal_handle = None
        self._current_task = None
        self._task_lock = threading.Lock()
        
        self.get_logger().info('='*80)
        self.get_logger().info('路点键盘控制器已初始化')
        self.get_logger().info(f'已加载 {len(self.waypoints)} 个路点')
        self.get_logger().info('='*80)
    
    def load_waypoints(self, yaml_file):
        """
        加载路点配置文件
        
        参数:
            yaml_file: YAML文件路径
        
        返回:
            dict: 路点字典 {name: {x, y, z, w}}
        """
        try:
            with open(yaml_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            
            waypoints = config.get('waypoints', {})
            self.get_logger().info(f'✅ 成功加载路点配置: {yaml_file}')
            
            # 验证路点格式
            for name, point in waypoints.items():
                if not all(k in point for k in ['x', 'y', 'z', 'w']):
                    self.get_logger().warn(f'⚠️  路点 {name} 缺少必要字段')
            
            return waypoints
            
        except FileNotFoundError:
            self.get_logger().error(f'❌ 找不到配置文件: {yaml_file}')
            return {}
        except yaml.YAMLError as e:
            self.get_logger().error(f'❌ YAML解析错误: {e}')
            return {}
    
    def wait_for_server(self, timeout_sec=10.0):
        """等待 Action 服务器上线"""
        self.get_logger().info('等待 Action 服务器连接...')
        
        if not self._action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error(f'❌ 超时 {timeout_sec}s, Action 服务器未响应')
            self.get_logger().error('请确认 point_to_point_action_server 已启动:')
            self.get_logger().error('  ros2 run navigation_controller point_to_point_action_server.py')
            return False
        
        self.get_logger().info('✅ Action 服务器已连接')
        return True
    
    def send_waypoint(self, waypoint_name):
        """
        发送路点导航请求
        
        参数:
            waypoint_name: 路点名称
        
        返回:
            bool: True 表示成功发送
        """
        if waypoint_name not in self.waypoints:
            self.get_logger().error(f'❌ 未知路点: {waypoint_name}')
            return False
        
        point = self.waypoints[waypoint_name]
        
        # 构建目标消息
        goal_msg = PointToPoint.Goal()
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        goal_msg.task_id = f'WAYPOINT_{waypoint_name.upper()}_{timestamp}'
        
        # 目标位姿
        goal_msg.target_pose = PoseStamped()
        goal_msg.target_pose.header = Header()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.target_pose.pose.position = Point(
            x=float(point['x']),
            y=float(point['y']),
            z=float(point['z'])
        )
        
        # 四元数 (从yaw角度转换,假设z轴旋转)
        # 如果w是yaw角度(弧度),转换为四元数
        import math
        yaw = float(point['w'])  # 假设w是yaw角度
        goal_msg.target_pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0)
        )
        
        separator = '=' * 60
        print(f'\n{Colors.BLUE}{separator}{Colors.END}')
        print(f'{Colors.BOLD}🚀 发送导航目标: {waypoint_name}{Colors.END}')
        print(f'  坐标: ({point["x"]:.2f}, {point["y"]:.2f})')
        print(f'  任务ID: {goal_msg.task_id}')
        print(f'{Colors.BLUE}{separator}{Colors.END}\n')
        
        # 发送目标
        with self._task_lock:
            self._current_task = waypoint_name
            send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True
    
    def goal_response_callback(self, future):
        """处理目标接受/拒绝响应"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            print(f'{Colors.RED}❌ 目标被服务器拒绝{Colors.END}')
            with self._task_lock:
                self._current_goal_handle = None
                self._current_task = None
            return
        
        print(f'{Colors.GREEN}✅ 目标已被接受,开始执行...{Colors.END}')
        
        with self._task_lock:
            self._current_goal_handle = goal_handle
        
        # 等待结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        """处理反馈消息 - 简化显示"""
        feedback = feedback_msg.feedback
        
        # 状态图标
        state_icons = {
            PointToPoint.Feedback.STATE_PLANNING: '🔍',
            PointToPoint.Feedback.STATE_FOLLOWING: '🚀',
            PointToPoint.Feedback.STATE_AVOIDING: '⚠️',
            PointToPoint.Feedback.STATE_RECOVERING: '🔄',
            PointToPoint.Feedback.STATE_STUCK: '🛑'
        }
        
        icon = state_icons.get(feedback.navigation_state, '❓')
        
        # 单行实时更新
        print(f'\r{icon} 剩余: {feedback.distance_to_goal:5.2f}m | '
              f'置信度: {feedback.localization_confidence:.2f} | '
              f'预计: {feedback.estimated_time_remaining:5.1f}s', end='', flush=True)
    
    def get_result_callback(self, future):
        """处理最终结果"""
        result = future.result().result
        
        # 清除实时更新行
        print('\r' + ' '*80 + '\r', end='')
        
        separator = '=' * 60
        print(f'\n{Colors.CYAN}{separator}{Colors.END}')
        
        # 状态码映射
        status_map = {
            PointToPoint.Result.STATUS_SUCCESS: (Colors.GREEN, '✅ 成功到达'),
            PointToPoint.Result.STATUS_TIMEOUT: (Colors.YELLOW, '⏱️  任务超时'),
            PointToPoint.Result.STATUS_LOCALIZATION_LOST: (Colors.RED, '📍 定位丢失'),
            PointToPoint.Result.STATUS_PATH_BLOCKED: (Colors.YELLOW, '🚧 路径阻塞'),
            PointToPoint.Result.STATUS_GOAL_UNREACHABLE: (Colors.RED, '🚫 目标不可达'),
            PointToPoint.Result.STATUS_PREEMPTED: (Colors.YELLOW, '🔀 任务被抢占'),
            PointToPoint.Result.STATUS_EMERGENCY_STOP: (Colors.RED, '🚨 紧急停止')
        }
        
        color, status_text = status_map.get(result.status_code, (Colors.END, '未知状态'))
        
        print(f'{color}{status_text}{Colors.END}')
        print(f'位置误差: {result.position_error:.3f} m')
        print(f'{Colors.CYAN}{separator}{Colors.END}\n')
        
        with self._task_lock:
            self._current_goal_handle = None
            self._current_task = None
    
    def cancel_current_task(self):
        """取消当前任务"""
        with self._task_lock:
            if self._current_goal_handle is not None:
                print(f'\n{Colors.YELLOW}🛑 取消当前任务...{Colors.END}')
                try:
                    cancel_future = self._current_goal_handle.cancel_goal_async()
                    cancel_future.add_done_callback(
                        lambda f: print(f'{Colors.GREEN}✅ 任务已取消{Colors.END}\n')
                    )
                except Exception as e:
                    print(f'{Colors.YELLOW}⚠️  取消请求失败 (任务可能已完成): {e}{Colors.END}')
                finally:
                    self._current_goal_handle = None
                    self._current_task = None
                return True
            else:
                # 静默返回,不打印警告
                return False
    
    def print_help(self):
        """打印帮助信息"""
        separator = '=' * 70
        print(f'\n{Colors.HEADER}{separator}{Colors.END}')
        print(f'{Colors.BOLD}            G1 路点键盘控制 - 帮助信息{Colors.END}')
        print(f'{Colors.HEADER}{separator}{Colors.END}\n')
        
        print(f'{Colors.CYAN}【路点列表】{Colors.END}')
        waypoint_keys = {
            '0': 'home_for_safety',
            '1': 'pick_empty_box',
            '2': 'place_empty_box',
            '3': 'pick_product_box',
            '4': 'place_product_box',
            '5': 'ng'
        }
        
        for key, name in waypoint_keys.items():
            if name in self.waypoints:
                point = self.waypoints[name]
                print(f'  {Colors.GREEN}{key}{Colors.END} - {name:20s} '
                      f'({point["x"]:6.2f}, {point["y"]:6.2f})')
        
        print(f'\n{Colors.CYAN}【控制命令】{Colors.END}')
        print(f'  {Colors.GREEN}H{Colors.END} - 显示此帮助信息')
        print(f'  {Colors.GREEN}C{Colors.END} - 取消当前任务')
        print(f'  {Colors.GREEN}S{Colors.END} - 显示当前状态')
        print(f'  {Colors.GREEN}Q{Colors.END} - 退出程序')
        
        print(f'\n{Colors.YELLOW}【提示】{Colors.END}')
        print(f'  - 直接按数字键快速导航')
        print(f'  - 任务执行中会显示实时反馈')
        print(f'  - 按 Ctrl+C 或 Q 安全退出')
        
        print(f'\n{Colors.HEADER}{separator}{Colors.END}\n')
    
    def print_status(self):
        """打印当前状态"""
        with self._task_lock:
            if self._current_task:
                print(f'\n{Colors.CYAN}当前任务:{Colors.END} {self._current_task}')
            else:
                print(f'\n{Colors.YELLOW}状态:{Colors.END} 空闲中,等待命令...')


def keyboard_control_loop(controller):
    """
    键盘控制主循环
    
    参数:
        controller: WaypointKeyboardController 实例
    """
    # 路点映射
    waypoint_map = {
        '0': 'home_for_safety',
        '1': 'pick_empty_box',
        '2': 'place_empty_box',
        '3': 'pick_product_box',
        '4': 'place_product_box',
        '5': 'ng'
    }
    
    # 显示欢迎信息
    controller.print_help()
    
    print(f'{Colors.BOLD}请输入命令 (H显示帮助): {Colors.END}', end='', flush=True)
    
    while True:
        try:
            command = input().strip().upper()
            
            if not command:
                print(f'{Colors.BOLD}请输入命令: {Colors.END}', end='', flush=True)
                continue
            
            # 处理命令
            if command == 'Q':
                print(f'\n{Colors.YELLOW}正在退出...{Colors.END}')
                try:
                    controller.cancel_current_task()
                except:
                    pass
                break
            
            elif command == 'H':
                controller.print_help()
            
            elif command == 'C':
                controller.cancel_current_task()
            
            elif command == 'S':
                controller.print_status()
            
            elif command in waypoint_map:
                waypoint_name = waypoint_map[command]
                controller.send_waypoint(waypoint_name)
            
            else:
                print(f'{Colors.RED}❌ 未知命令: {command}{Colors.END}')
                print(f'{Colors.YELLOW}提示: 按 H 查看帮助{Colors.END}')
            
            print(f'\n{Colors.BOLD}请输入命令: {Colors.END}', end='', flush=True)
            
        except KeyboardInterrupt:
            print(f'\n\n{Colors.YELLOW}检测到 Ctrl+C, 正在退出...{Colors.END}')
            try:
                controller.cancel_current_task()
            except:
                pass
            break
        except EOFError:
            print(f'\n{Colors.YELLOW}EOF 检测, 正在退出...{Colors.END}')
            break
        except Exception as e:
            print(f'\n{Colors.RED}❌ 命令处理错误: {e}{Colors.END}')
            continue


def main(args=None):
    """主函数"""
    # 初始化 ROS2
    rclpy.init(args=args)
    
    # 查找waypoints.yaml文件
    config_paths = [
        Path('/home/sunny/sunny/hr_ros2_ws/src/config/waypoints.yaml'),
        Path('src/config/waypoints.yaml'),
        Path('src/navigation_controller/config/waypoints.yaml'),
        Path.cwd() / 'src/config/waypoints.yaml'
    ]
    
    waypoints_file = None
    for path in config_paths:
        if path.exists():
            waypoints_file = str(path)
            break
    
    if not waypoints_file:
        print(f'{Colors.RED}❌ 找不到 waypoints.yaml 配置文件{Colors.END}')
        print(f'{Colors.YELLOW}搜索路径:{Colors.END}')
        for path in config_paths:
            print(f'  - {path}')
        rclpy.shutdown()
        return
    
    controller = None
    executor = None
    
    try:
        # 创建控制器
        controller = WaypointKeyboardController(waypoints_file)
        
        # 等待服务器
        if not controller.wait_for_server(timeout_sec=10.0):
            controller.destroy_node()
            rclpy.shutdown()
            return
        
        # 使用 MultiThreadedExecutor 替代 daemon 线程
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(controller)
        
        # 启动 executor 线程
        spin_thread = threading.Thread(
            target=executor.spin,
            daemon=True
        )
        spin_thread.start()
        
        # 运行键盘控制循环
        keyboard_control_loop(controller)
        
    except KeyboardInterrupt:
        print(f'\n{Colors.YELLOW}检测到 Ctrl+C{Colors.END}')
    except Exception as e:
        print(f'{Colors.RED}❌ 错误: {e}{Colors.END}')
    finally:
        # 安全清理
        if controller is not None:
            try:
                controller.cancel_current_task()
            except:
                pass
            try:
                controller.destroy_node()
            except:
                pass
        
        if executor is not None:
            try:
                executor.shutdown()
            except:
                pass
        
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass
        
        print(f'\n{Colors.GREEN}✅ 程序已安全退出{Colors.END}\n')


if __name__ == '__main__':
    main()
