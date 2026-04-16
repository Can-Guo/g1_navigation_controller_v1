#!/usr/bin/env python3
"""
Simulation odometry publisher for this workspace.

High-level idea / 原理概述：
- 在「纯仿真 / 不启用激光定位」模式下，TF 树中通常只存在静态变换：
  map -> odom（单位变换）以及 odom -> base_link（静态或由其他节点维护）。
- 但是上层导航代码（A*、DWA、EKF 等）往往依赖一个连续发布的 `nav_msgs/Odometry`
  话题来表示机器人位姿；如果完全没有里程计话题，这些模块会报错或直接不工作。

What this node does / 这个节点具体做什么：
- 周期性从 TF 中读取 `odom_frame -> base_frame` 的变换（默认 `odom -> base_link`），
  相当于「问 TF：当前机器人在 odom 坐标系下的位姿是多少？」。
- 用这个变换直接构造并发布一条 `/odometry/map` 的 `nav_msgs/Odometry` 消息：
  - `header.frame_id` = `odom_frame`
  - `child_frame_id` = `base_frame`
  - `pose` 由 TF 的平移 + 旋转直接填充
- 由于仿真模式下 `map -> odom` 被设置为单位变换（identity），数值上
  「odom 坐标」与「map 坐标」等价，因此保留历史代码中使用的 `/odometry/map`
  话题名即可复用现有导航逻辑，而不需要大规模改动。

Why keep twist zero / 为什么速度为 0：
- 这里的目的是为导航栈提供一个「位姿来源」，而不是物理真实的动力学仿真。
- 如果上层需要速度信息，可以在以后接入更完整的仿真（例如 Gazebo、unitree SDK 等）；
  在当前最小实现中，将 `twist` 保持为 0 即可满足大多数定位/规划模块的输入要求。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration

from nav_msgs.msg import Odometry
import tf2_ros


class SimulationOdometryPublisher(Node):
    def __init__(self):
        super().__init__('simulation_odometry')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.odom_pub = self.create_publisher(Odometry, '/odometry/map', qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        period = 1.0 / max(1.0, self.publish_rate)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"Simulation odometry publisher started: /odometry/map from TF {self.odom_frame}->{self.base_frame} @ {self.publish_rate:.1f}Hz"
        )

    def _tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception as ex:
            self.get_logger().warn(
                f"Waiting for TF {self.odom_frame}->{self.base_frame}: {ex}",
                throttle_duration_sec=2.0,
            )
            return

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = tf.transform.translation.x
        msg.pose.pose.position.y = tf.transform.translation.y
        msg.pose.pose.position.z = tf.transform.translation.z
        msg.pose.pose.orientation = tf.transform.rotation

        # Twist left zero in simulation (no dynamics model here)
        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SimulationOdometryPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

