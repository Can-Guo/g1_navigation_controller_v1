#!/usr/bin/env python3
"""
G1 Position Control Node
Author: shenlanxueyuan
Email: 1347893477@qq.com
Organization: Shen Lan xue yuan
Date: 2025-03-28

This node implements position control for the G1 robot, including:
- Path following using A* path
- Obstacle avoidance using DWA
- Position control with dynamic speed adjustment 
"""

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist, Point
import math
import numpy as np

class G1PositionControl:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('g1_position_control')
        
        # 订阅ROS1定位话题 - 获取机器人当前位置
        self.odom_sub = rospy.Subscriber(
            '/odometry/map',
            Odometry,
            self.odom_callback,
            queue_size=10)
            
        # 订阅2D Nav Goal设置的目标位置话题
        self.goal_sub = rospy.Subscriber(
            '/move_base_simple/goal',
            PoseStamped,
            self.goal_callback,
            queue_size=10)
            
        # 订阅导航路径话题
        self.path_sub = rospy.Subscriber(
            '/astar_path',
            Path,
            self.path_callback,
            queue_size=10)
            
        # 订阅DWA避障控制命令
        self.dwa_cmd_sub = rospy.Subscriber(
            '/dwa_cmd_vel',
            Twist,
            self.dwa_cmd_callback,
            queue_size=10)
            
        # 订阅障碍物状态
        self.obstacle_status_sub = rospy.Subscriber(
            '/obstacle_status',
            Point,
            self.obstacle_status_callback,
            queue_size=10)
            
        # 创建速度指令发布者
        self.vel_pub = rospy.Publisher(
            '/cmd_vel',  # 修改为标准的cmd_vel话题
            Twist,
            queue_size=10)
            
        # 初始化目标位置标志
        self.has_goal = False
        self.has_path = False
        
        # 障碍物避障模式标志
        self.has_obstacle = False
        self.obstacle_avoidance_active = False
        
        # 设置导航参数
        self.waypoint_distance = rospy.get_param('~waypoint_distance', 0.5) # 路点距离阈值
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.3) # 目标容忍距离
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.3)  # 最大线速度
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.3) # 最大角速度
        
        # 存储当前位置、目标位置和路径
        self.current_pose = None
        self.goal_pose = None
        self.waypoints = []
        
        # 存储DWA控制命令
        self.dwa_cmd = None
        self.last_dwa_cmd_time = None
        
        # 路径导航相关变量
        self.current_waypoint_index = 0
        
        # 消息计数器，用于减少日志频率
        self.count = 0

        # York, 20250731
        # 新增：超时检测相关变量
        self.target_start_time = None
        self.target_distance = None
        self.timeout_triggered = False
        # York, 20250731
        
        rospy.loginfo("G1导航控制节点已启动，等待路径信息...")
        
        # 创建定时器，每100ms检查是否需要移动
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        
    def odom_callback(self, msg):
        self.current_pose = msg
        
        # 输出当前位置，便于调试
        if self.count % 50 == 0:  # 每50次消息打印一次，避免日志过多
            rospy.loginfo(
                f"当前位置: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}, frame_id={msg.header.frame_id}")
        self.count += 1
        
    def goal_callback(self, msg):
        self.goal_pose = msg
        self.has_goal = True
        
        rospy.loginfo(
            f"收到新目标: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, frame_id={msg.header.frame_id}")
            
    def path_callback(self, msg):
        if not msg.poses:
            rospy.logwarn("收到空路径，忽略")
            return
            
        # 清空当前路径点
        self.waypoints.clear()
        
        # 提取路径中的路点
        rospy.loginfo("全部路径点:")
        for pose in msg.poses:
            self.waypoints.append(pose)
            
        self.has_path = True
        self.current_waypoint_index = 0
        
        rospy.loginfo(
            f"收到新路径，包含 {len(self.waypoints)} 个路点，frame_id={msg.header.frame_id}")
            
        # 设置最终目标
        if self.waypoints:
            self.goal_pose = self.waypoints[-1]
            self.has_goal = True
            
        # 重置避障状态
        if self.obstacle_avoidance_active:
            rospy.loginfo("收到新路径，重置避障状态")
            self.obstacle_avoidance_active = False
            self.has_obstacle = False
            
    def dwa_cmd_callback(self, msg):
        self.dwa_cmd = msg
        self.last_dwa_cmd_time = rospy.Time.now()
        
        # 添加调试信息
        if self.count % 50 == 0:
            rospy.loginfo(
                f"收到DWA控制命令: vx={msg.linear.x:.2f}, vyaw={msg.angular.z:.2f}")
                
    def obstacle_status_callback(self, msg):
        # msg.x: 0表示无障碍物，1表示有障碍物
        # msg.y: 0表示未激活避障，1表示激活避障
        new_has_obstacle = (msg.x > 0.5)
        new_avoidance_active = (msg.y > 0.5)
        
        # 状态变化时记录日志
        if new_has_obstacle != self.has_obstacle:
            if new_has_obstacle:
                rospy.loginfo("检测到障碍物")
            else:
                rospy.loginfo("障碍物已清除")
                
        if new_avoidance_active != self.obstacle_avoidance_active:
            if new_avoidance_active:
                rospy.loginfo("启动DWA避障模式")
            else:
                rospy.loginfo("退出DWA避障模式，恢复A*路径跟踪")
                # 避障模式关闭时，查找新的路径点
                self.find_closest_waypoint()
                
        self.has_obstacle = new_has_obstacle
        self.obstacle_avoidance_active = new_avoidance_active
        
    def get_current_waypoint(self):
        if self.has_path and self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        elif self.has_goal:
            return self.goal_pose
            
        # 默认返回当前位置（不移动）
        current = PoseStamped()
        current.header = self.current_pose.header
        current.pose = self.current_pose.pose.pose
        return current
        
    def update_waypoint_if_needed(self):
        if not self.has_path or self.current_waypoint_index >= len(self.waypoints):
            return
            
        robot_x = self.current_pose.pose.pose.position.x
        robot_y = self.current_pose.pose.pose.position.y
        
        waypoint = self.waypoints[self.current_waypoint_index]
        waypoint_x = waypoint.pose.position.x
        waypoint_y = waypoint.pose.position.y
        
        distance = math.sqrt((waypoint_x - robot_x)**2 + (waypoint_y - robot_y)**2)


        # York, 20250731
        # === 新增：每次切换目标点时，记录时间和距离 ===
        if not hasattr(self, 'last_waypoint_index'):
            self.last_waypoint_index = -1
        if self.current_waypoint_index != self.last_waypoint_index:
            self.target_start_time = rospy.Time.now()
            self.target_distance = distance
            self.timeout_triggered = False
            self.last_waypoint_index = self.current_waypoint_index
            rospy.loginfo(f"切换到新目标点{self.current_waypoint_index}，距离={distance:.2f}m，计时开始")
        # York, 20250731


        # 如果距离当前路点足够近，更新到下一个路点
        if distance < self.waypoint_distance:
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                rospy.loginfo(
                    f"到达路点 {self.current_waypoint_index-1}，前往下一路点 {self.current_waypoint_index}")


        # York, 20250731
    def check_timeout(self, distance):
        """根据距离和已用时间判断是否超时"""
        if self.target_start_time is None or self.timeout_triggered:
            return False

        elapsed = (rospy.Time.now() - self.target_start_time).to_sec()
        d = self.target_distance if self.target_distance is not None else distance

        if 0 <= d <= 3 and elapsed > 30:
            print(f"距离 {d:.2f}m，已用时间 {elapsed:.2f}秒，触发超时")
            return True
        elif 3 < d <= 10 and elapsed > 60:
            print(f"距离 {d:.2f}m，已用时间 {elapsed:.2f}秒，触发超时")
            return True
        elif 10 < d <= 20 and elapsed > 120:
            print(f"距离 {d:.2f}m，已用时间 {elapsed:.2f}秒，触发超时")
            return True
        elif d > 20 and elapsed > 180:
            print(f"距离 {d:.2f}m，已用时间 {elapsed:.2f}秒，触发超时")
            return True
        return False
        # York, 20250731

    def find_closest_waypoint(self):
        if not self.has_path or not self.current_pose or not self.waypoints:
            return
            
        robot_x = self.current_pose.pose.pose.position.x
        robot_y = self.current_pose.pose.pose.position.y
        
        closest_idx = self.current_waypoint_index
        min_dist = float('inf')
        
        # 从当前索引开始查找最近的路径点
        for i in range(self.current_waypoint_index, len(self.waypoints)):
            waypoint = self.waypoints[i]
            dist = math.sqrt(
                (waypoint.pose.position.x - robot_x)**2 + 
                (waypoint.pose.position.y - robot_y)**2
            )
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        # 如果找到的点就是当前点，尝试前进2-3个点以避开障碍物
        if closest_idx == self.current_waypoint_index and closest_idx + 3 < len(self.waypoints):
            closest_idx += 3
            
        if closest_idx != self.current_waypoint_index:
            rospy.loginfo(
                f"避障后重新定位到路径点 {closest_idx} (原路径点: {self.current_waypoint_index}，总点数: {len(self.waypoints)})")
            self.current_waypoint_index = closest_idx
            
    def control_loop(self, event):
        # 如果没有当前位置，则返回
        if not self.current_pose:
            rospy.logwarn("没有收到位置信息，无法执行控制")
            return
            
        # 添加调试信息
        rospy.loginfo(
            f"控制状态: has_goal={self.has_goal}, has_path={self.has_path}, "
            f"obstacle_avoidance_active={self.obstacle_avoidance_active}, "
            f"current_waypoint_index={self.current_waypoint_index}, "
            f"total_waypoints={len(self.waypoints) if self.has_path else 0}"
        )
            
        # 检查DWA控制命令是否过期（超过0.5秒）
        dwa_cmd_valid = False
        if self.dwa_cmd and self.last_dwa_cmd_time:
            elapsed = (rospy.Time.now() - self.last_dwa_cmd_time).to_sec()
            dwa_cmd_valid = (elapsed < 0.5)
            
        # 如果有障碍物并且DWA命令有效，并且有路径，则使用DWA控制
        if self.obstacle_avoidance_active and dwa_cmd_valid and self.has_path:
            # 使用DWA避障控制命令
            cmd_vel = Twist()
            cmd_vel.linear.x = self.dwa_cmd.linear.x
            cmd_vel.angular.z = self.dwa_cmd.angular.z
            
            self.vel_pub.publish(cmd_vel)

            print(f"cmd_vel: {cmd_vel}")
            
            if self.count % 50 == 0:  # 减少日志输出频率
                rospy.loginfo(
                    f"DWA避障控制: vx={cmd_vel.linear.x:.2f}, vyaw={cmd_vel.angular.z:.2f}")
            return
        elif self.obstacle_avoidance_active and (not self.has_path or self.current_waypoint_index >= len(self.waypoints)):
            # 如果处于避障模式但没有路径或路径已完成，则退出避障模式
            rospy.loginfo("没有有效路径或路径已完成，退出DWA避障模式")
            self.obstacle_avoidance_active = False
            
            # 发送停止命令
            cmd_vel = Twist()
            self.vel_pub.publish(cmd_vel)
            return
            
        # 如果有路径，根据当前位置更新要前往的路点
        if self.has_path and not self.obstacle_avoidance_active:
            self.update_waypoint_if_needed()
            
        # 如果没有目标或路径已完成，则停止
        if (not self.has_goal and not self.has_path) or \
           (self.has_path and self.current_waypoint_index >= len(self.waypoints)):
            cmd_vel = Twist()
            self.vel_pub.publish(cmd_vel)
            
            if self.has_path and self.current_waypoint_index >= len(self.waypoints):
                rospy.loginfo("路径导航完成，停止移动")
                self.has_path = False
            return
            
        # 如果处于避障模式，不执行A*路径跟踪
        if self.obstacle_avoidance_active:
            return
            
        # 获取当前导航目标点
        target = self.get_current_waypoint()
        
        # 提取当前位置和目标位置
        robot_x = self.current_pose.pose.pose.position.x
        robot_y = self.current_pose.pose.pose.position.y
        target_x = target.pose.position.x
        target_y = target.pose.position.y
        
        # 计算位置差异
        dx = target_x - robot_x
        dy = target_y - robot_y
        
        # 计算移动方向和速度
        distance = math.sqrt(dx*dx + dy*dy)


        # York, 20250731
        # 如果没有设置目标开始时间，则初始化
        # === 新增：超时检测 ===
        if self.check_timeout(distance):
            rospy.logwarn(f"到达目标点超时，已停止机器人！距离={distance:.2f}m")
            cmd_vel = Twist()
            self.vel_pub.publish(cmd_vel)
            self.timeout_triggered = True
            return
        # York, 20250731


        # 检查是否到达最终目标
        if not self.has_path and distance <= self.goal_tolerance:
            # 接近最终目标，停止
            cmd_vel = Twist()
            self.vel_pub.publish(cmd_vel)
            
            # 清除目标，避免重复控制
            self.has_goal = False
            rospy.loginfo("到达最终目标位置，停止移动")
            return
            
        # 计算移动方向
        theta = math.atan2(dy, dx)
        
        # 从四元数中提取朝向角
        q = self.current_pose.pose.pose.orientation
        # 计算yaw角 (z轴旋转)
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        angle_diff = theta - yaw
        
        # 归一化角度到[-PI, PI]
        while angle_diff > math.pi: angle_diff -= 2*math.pi
        while angle_diff < -math.pi: angle_diff += 2*math.pi
        
        # 创建控制命令
        cmd_vel = Twist()
        
        # 计算线速度 - 根据距离和角度差异动态调整
        vx = self.max_linear_speed
        if abs(angle_diff) > 0.7:
            vx = 0.0
            vyaw = self.max_angular_speed * (1 if angle_diff > 0 else -1)
        else:
            if abs(angle_diff) > 0.5:
                vx = self.max_linear_speed * (1.0 - abs(angle_diff) / math.pi)  # 角度偏差大时减速
            # York, 20260203
            # if distance < 0.3:
            #     vx = vx * (distance / 0.3)  # 接近目标时减速
            
        # 角速度与偏差成比例，但限制最大值
        vyaw = min(self.max_angular_speed, abs(angle_diff) * 1.0) * (1 if angle_diff > 0 else -1)
        
        # vx, vyaw round to 2 decimal places, York-20250717
        vx = round(vx, 2)
        vyaw = round(vyaw, 2)

        # 设置速度指令
        cmd_vel.linear.x = vx
        cmd_vel.angular.z = vyaw

        # 添加调试信息
        # print(f"g1 control cmd_vel:\t{cmd_vel}\r\n")
        
        # 发布命令 
        self.vel_pub.publish(cmd_vel)
        
        if self.count % 50 == 0:  # 减少日志输出频率
            rospy.loginfo(
                f"导航控制: vx={vx:.2f}, vyaw={vyaw:.2f}, 距离={distance:.2f}, 朝向={yaw:.2f}→{theta:.2f}, 路点={self.current_waypoint_index}/{len(self.waypoints) if self.has_path else 0}")

def main():
    try:
        node = G1PositionControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()