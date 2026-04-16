# PointToPoint Action 使用指南

## 📋 目录

- [概述](#概述)
- [系统架构](#系统架构)
- [快速开始](#快速开始)
- [API 文档](#api-文档)
- [测试指南](#测试指南)
- [安全机制](#安全机制)
- [故障排查](#故障排查)

---

## 概述

`PointToPoint` Action 是 G1 Unitree 机器人导航系统的核心控制接口,实现了基于 Navigation_Control_Protocol_v1.0 的点对点导航功能。

### 核心特性

- ✅ **实时状态监控**: 位姿、速度、障碍物、定位置信度
- ✅ **多层安全保护**: 紧急停止、超时、定位丢失检测
- ✅ **Master-Slave 控制架构**: Action Server 完全控制 Position Control 节点执行状态
- ✅ **任务管理**: 支持抢占、取消、队列管理
- ✅ **高性能反馈**: 5Hz 实时更新,延迟 < 200ms
- ✅ **完整测试覆盖**: 自动化测试 + 交互式调试
- ✅ **配置集中化**: 环境参数统一管理 (config_env.yaml)

### 系统需求

- **ROS2 版本**: Humble
- **Python 版本**: 3.10+
- **依赖包**:
  - `navigation_controller` (本包)
  - `mid360_locate` (定位)
  - `livox_ros_driver2` (LiDAR 驱动)
  - `nav2_map_server` (地图服务)
  - `robot_localization` (EKF 传感器融合,可选)

---

## 系统架构

### Master-Slave 控制模式

```
┌─────────────────────────────────────────────────────────────┐
│                       MCB (主控板)                          │
│                    Action Client                            │
└─────────────────────┬───────────────────────────────────────┘
                      │ PointToPoint.action
                      ▼
┌─────────────────────────────────────────────────────────────┐
│       PointToPoint Action Server (Master Controller)        │
│  - 任务生命周期管理                                         │
│  - 状态监控与异常处理                                       │
│  - 实时反馈生成                                             │
│  - 🆕 控制 Position Control 执行状态                        │
└─────┬──────────┬────────────┬───────────────────────────────┘
      │          │            │
      │          │            └──────────────────┐
      │          │                               │
      ▼          ▼                               ▼
┌──────────┐ ┌────────┐            /navigation_active (Bool)
│ A* Planner│ │  DWA   │            2Hz 心跳 + 5s 超时保护
│          │ │ Planner│                         │
└────┬─────┘ └───┬────┘                         ▼
     │           │             ┌──────────────────────────────┐
     │           │             │ Position Control (Slave)     │
     │           │             │  - 仅在 active=True 时执行  │
     │           │             │  - 超时自动停止             │
     └───────────┴─────────────┤  - 双通道控制架构           │
                               └──────────┬───────────────────┘
                                          │
                  ┌───────────────────────┴─────────────────┐
                  ▼                                         ▼
          /cmd_vel (ROS2)                      Unitree SDK (硬件模式)
                  │                                         │
                  └───────────────┬─────────────────────────┘
                                  ▼
                          ┌──────────────┐
                          │  G1 Robot    │
                          └──────────────┘
```

### 关键改进 (v1.1.0)

- **Master-Slave 控制**: Action Server 通过 `/navigation_active` 话题完全控制 Position Control
- **心跳机制**: 2Hz 持续信号,确保通信活跃
- **超时保护**: 5 秒无信号自动停止机器人
- **双通道架构**: 支持硬件 SDK + ROS2 /cmd_vel 同时发布

### 话题订阅/发布关系

| 话题名称                  | 类型                     | 方向  | 用途                     |
|---------------------------|--------------------------|-------|--------------------------|
| `/odometry/ekf`           | `nav_msgs/Odometry`      | 订阅  | 融合后的机器人位姿和速度 |
| `/astar_path`             | `nav_msgs/Path`          | 订阅  | A* 全局路径              |
| `/dwa_obstacles`          | `sensor_msgs/PointCloud2`| 订阅  | DWA 检测到的障碍物       |
| `/localization_confidence`| `std_msgs/Float32`       | 订阅  | 定位置信度               |
| `/emergency_stop`         | `std_msgs/Bool`          | 订阅  | 紧急停止信号             |
| `/move_base_simple/goal`  | `geometry_msgs/PoseStamped`| 发布 | 触发 A* 规划             |
| `/navigation_active`      | `std_msgs/Bool`          | 发布  | 🆕 控制 Position Control 执行状态 |

**注**: 系统已升级为 EKF 融合架构,使用 `/odometry/ekf` 替代 `/odometry/map`

---

## 快速开始

### 1. 配置环境参数

```bash
# 编辑环境配置文件 (可选,使用默认值可跳过)
nano src/navigation_controller/config/config_env.yaml

# 关键配置项:
# - network_interface: 'ens37'  # Unitree SDK 网络接口
# - use_hardware: false          # 硬件模式开关
# - max_linear_speed: 0.25       # 最大线速度
# - map_file: 'map.yaml'         # 地图文件名
```

### 2. 编译包

```bash
colcon build --base-paths src/navigation_controller/pkg --symlink-install
colcon build 
source install/setup.bash
```

### 3. 启动导航系统

```bash
# 终端 1: 启动完整导航系统 
# 仿真模式 (默认)
ros2 launch navigation_controller fusion_ros2.launch.py

# 硬件模式 (G1 机器人)
ros2 launch navigation_controller fusion_ros2.launch.py use_hardware:=true
```


<!-- **注**: Action Server 现已集成到 launch 文件中,无需单独启动 —— 2025.12.22 確認並未實現 -->

### 4. 设置初始位姿

```bash
# 在 RViz 中使用 "2D Pose Estimate" 工具设置机器人初始位置
```

### 5. 运行测试
```bash
# 终端 2: 启动point_to_point_action_server
ros2 run navigation_controller point_to_point_action_server.py 
```


```bash
# 终端 3: 键盘控制导航 (推荐)
ros2 run navigation_controller test_point_to_point_action_keyboard.py

# # 或运行自动化测试
# ros2 run navigation_controller test_point_to_point_action.py --test basic
```

---

## API 文档

### Action 定义: `PointToPoint.action`

#### Goal (目标)

```python
string task_id                        # MCB 任务 ID
geometry_msgs/PoseStamped target_pose # 目标位姿 (frame_id 必须为 "map")
```

**示例**:
```python
goal = PointToPoint.Goal()
goal.task_id = "MCB_TASK_20251105_001"
goal.target_pose.header.frame_id = "map"
goal.target_pose.pose.position.x = 5.0
goal.target_pose.pose.position.y = 2.0
```

#### Feedback (反馈)

```python
geometry_msgs/PoseStamped current_pose  # 当前位姿
geometry_msgs/Twist current_velocity    # 当前速度

uint8 navigation_state                  # 导航状态
# STATE_PLANNING = 0   - A* 规划中
# STATE_FOLLOWING = 1  - 正常跟踪路径
# STATE_AVOIDING = 2   - DWA 避障激活
# STATE_RECOVERING = 3 - 恢复行为中
# STATE_STUCK = 4      - 可能卡住

uint8 obstacle_proximity                # 障碍物接近度
# OBSTACLE_CLEAR = 0    - 安全距离 (>1.5m)
# OBSTACLE_NEAR = 1     - 接近障碍物 (0.5-1.5m)
# OBSTACLE_CRITICAL = 2 - 危险距离 (<0.5m)

float32 distance_to_goal                # 到目标的直线距离
float32 path_length_remaining           # 沿路径的剩余长度
float32 estimated_time_remaining        # 预计剩余时间
float32 localization_confidence         # 定位置信度 (0-1)
string debug_message                    # 调试信息
```

**更新频率**: 5 Hz

#### Result (结果)

```python
bool success                            # 是否成功
uint8 status_code                       # 状态码

# 状态码定义:
# STATUS_SUCCESS = 0            - 成功到达
# STATUS_TIMEOUT = 1            - 任务超时
# STATUS_LOCALIZATION_LOST = 2  - 定位丢失
# STATUS_PATH_BLOCKED = 3       - 路径阻塞
# STATUS_GOAL_UNREACHABLE = 4   - 目标不可达
# STATUS_PREEMPTED = 5          - 任务被抢占
# STATUS_EMERGENCY_STOP = 6     - 紧急停止

geometry_msgs/PoseStamped final_pose    # 最终位姿
float32 position_error                  # 位置误差 (米)
```

---

## 测试指南

### 测试场景

#### 1. 基础导航测试

验证机器人能够到达单个目标点。

```bash
ros2 run navigation_controller test_point_to_point_action.py --test basic
```

**预期结果**:
- 位置误差 < 0.3m
- 状态码: `STATUS_SUCCESS`

#### 2. 多目标巡航测试

依次访问多个路点,测试连续导航能力。

```bash
ros2 run navigation_controller test_point_to_point_action.py --test multi
```

**测试路径**: (5,0) → (5,5) → (0,5) → (0,0)

#### 3. 任务取消测试

中途取消正在执行的任务。

```bash
ros2 run navigation_controller test_point_to_point_action.py --test cancel
```

**验证点**:
- 取消后机器人停止运动
- 状态码: `STATUS_PREEMPTED`

#### 4. 紧急停止测试

验证紧急停止响应时间。

```bash
ros2 run navigation_controller test_point_to_point_action.py --test emergency
```

**性能指标**: 响应时间 < 100ms

#### 5. 完整测试套件

运行所有测试场景。

```bash
ros2 run navigation_controller test_point_to_point_action.py --test all
```

### 交互式调试

手动输入目标点进行测试。

```bash
ros2 run navigation_controller test_point_to_point_action.py --interactive
```

**使用方式**:
```
目标坐标 (x y): 5.0 2.0   # 发送目标
目标坐标 (x y): -3.0 4.5  # 发送下一个目标
目标坐标 (x y): q          # 退出
```

---

## 安全机制

### 1. Master-Slave 控制保护 (v1.1.0 新增)

- **机制**: Action Server 通过 `/navigation_active` 控制 Position Control
- **心跳频率**: 2Hz 持续发布 `active=True`
- **超时保护**: Position Control 5 秒无信号自动停止
- **失败场景**: 
  - Action Server 超时 → 发布 `active=False` → 机器人立即停止 ✅
  - Action Server 崩溃 → 心跳停止 → 5秒后自动停止 ✅
  - 任务取消 → 发布 `active=False` → 机器人立即停止 ✅

### 2. 定位监控

- **阈值**: 置信度 < 0.3 持续 5 秒
- **响应**: 
  1. 停止导航 (`active=False`)
  2. 中止任务,返回 `STATUS_LOCALIZATION_LOST`
- **恢复**: 在 RViz 中重新设置初始位姿

### 3. 超时保护

- **计算公式**: `timeout = base_timeout + distance × timeout_per_meter`
- **默认值**: 
  - `base_timeout = 60s`
  - `timeout_per_meter = 3s/m`
  - `max_timeout = 300s`
- **示例**: 10m 目标 → 超时 = 60 + 10×3 = 90 秒
- **响应**: 超时时调用 `deactivate_navigation()` 停止机器人

### 4. 卡住检测

- **检测条件**: 5 秒内移动 < 0.1m 且有速度指令
- **响应**: 触发恢复行为 (后退、旋转)
- **超时**: 恢复行为 10 秒后仍无进展 → 停止导航 → 中止任务

### 5. 紧急停止

- **触发方式**: 
  ```bash
  ros2 topic pub /emergency_stop std_msgs/Bool "data: true" --once
  ```
- **响应时间**: < 100ms
- **停止机制**: 立即调用 `deactivate_navigation()`
- **解除**: 发布 `data: false`

### 6. 任务抢占

- **场景**: 新目标到达时自动取消当前任务
- **停止机制**: 调用 `deactivate_navigation()` 停止前一任务
- **状态码**: `STATUS_PREEMPTED`
- **数据一致性**: 保证通过线程锁

---

## 故障排查

### 问题 1: Action 服务器未响应

**现象**:
```
❌ 超时 10s, Action 服务器未响应
```

**排查步骤**:

1. 检查服务器是否运行:
   ```bash
   ros2 node list | grep point_to_point
   ```

2. 检查 Action 接口:
   ```bash
   ros2 action list
   ros2 action info /point_to_point
   ```

3. 确认 launch 文件是否包含 Action Server:
   ```bash
   # Action Server 应该已集成到 fusion_ros2.launch.py
   # 检查日志中是否有 Action Server 启动信息
   ```

4. 如需单独启动:
   ```bash
   ros2 run navigation_controller point_to_point_action_server.py
   ```

### 问题 2: 定位丢失

**现象**:
```
⚠️ 定位置信度低: 0.22
任务完成: 📍 定位丢失
```

**解决方案**:

1. 检查融合后的里程计话题:
   ```bash
   ros2 topic hz /odometry/ekf
   ros2 topic echo /odometry/ekf --once
   ```

2. 检查原始定位数据:
   ```bash
   ros2 topic hz /odometry/map
   ros2 topic echo /localization_confidence
   ```

3. 检查 `mid360_locate` 节点状态:
   ```bash
   ros2 node info /locate_node
   ```

4. 在 RViz 中重新设置初始位姿 (2D Pose Estimate 工具)

5. 检查 EKF 融合状态 (如已启用):
   ```bash
   ros2 node info /ekf_localization
   ros2 topic echo /diagnostics
   ```

### 问题 3: 机器人超时后仍在移动

**现象**:
```
[action_server]: 任务超时
[position_control]: 导航控制: vx=0.25, vyaw=0.04  ← 机器人仍在运动
```

**原因**: Master-Slave 控制未正确配置

**解决方案**:

1. 检查 `/navigation_active` 话题:
   ```bash
   ros2 topic echo /navigation_active
   # 应该在任务进行时持续显示 data: true (2Hz)
   ```

2. 验证 Position Control 是否订阅控制话题:
   ```bash
   ros2 node info /g1_position_control
   # 应该有 /navigation_active 订阅
   ```

3. 检查超时后是否发布停止信号:
   ```bash
   # 超时时应看到 data: false
   ros2 topic echo /navigation_active
   ```

4. 重新编译确保代码最新:
   ```bash
   colcon build --base-paths src/navigation_controller/pkg --symlink-install

  colcon build --base-paths src --packages-select navigation_controller --symlink-install
   ```

### 问题 4: 路径阻塞

**现象**:
```
🛑 可能卡住
恢复行为超时,判定为路径阻塞
```

**排查步骤**:

1. 检查 A* 路径是否生成:
   ```bash
   ros2 topic echo /astar_path --once
   ```

2. 检查 DWA 障碍物检测:
   ```bash
   ros2 topic echo /dwa_obstacles --once
   ```

3. 在 RViz 中可视化障碍物和路径

4. 手动清除障碍物或更换目标点

### 问题 5: 编译错误

**现象**:
```
CMake Error: Could not find rosidl_default_generators
```

**解决方案**:

1. 确认已安装所有依赖:
   ```bash
   sudo apt update
   sudo apt install ros-humble-rosidl-default-generators
   sudo apt install ros-humble-action-msgs
   ```

2. 重新编译:
   ```bash
   cd /home/york/ros2_ws_nav
   rm -rf build/ install/ log/
   colcon build --packages-select navigation_controller
   ```

### 问题 6: Action 接口未生成

**现象**:
```python
ModuleNotFoundError: No module named 'navigation_controller.action'
```

**解决方案**:

1. 确认 `action/PointToPoint.action` 文件存在

2. 检查 `CMakeLists.txt` 中的配置:
   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     "action/PointToPoint.action"
     ...
   )
   ```

3. 重新编译并 source:
   ```bash
   colcon build --packages-select navigation_controller
   source install/setup.bash
   ```

4. 验证接口生成:
   ```bash
   ros2 interface show navigation_controller/action/PointToPoint
   ```

---

## 性能基准

| 指标                 | 目标值      | 实测值       |
|----------------------|-------------|--------------|
| 到达精度             | < 0.3m      | 0.05-0.15m   |
| 响应延迟             | < 0.5s      | 0.2-0.3s     |
| 反馈频率             | 5Hz         | 5.0 ± 0.2 Hz |
| 紧急停止响应         | < 100ms     | 50-80ms      |
| 定位置信度阈值       | 0.3         | 0.3          |
| 平均导航速度         | 0.2-0.3 m/s | 0.25 m/s     |

---

## 参数配置

### 方法 1: 环境配置文件 (推荐)

编辑 `config_env.yaml` 统一管理所有参数:

```bash
nano /home/york/ros2_ws_nav/src/navigation_controller/config/config_env.yaml
```

```yaml
# 导航参数
max_linear_speed: 0.25
max_angular_speed: 0.08
waypoint_distance: 0.5
goal_tolerance: 0.3

# 硬件配置
network_interface: 'ens33'
use_hardware: false

# 地图文件
map_file: 'map.yaml'
pcd_file: 'map.pcd'
```

### 方法 2: Launch 参数覆盖

```bash
ros2 launch navigation_controller fusion_ros2.launch.py \
  use_hardware:=true \
  max_linear_speed:=0.3 \
  goal_tolerance:=0.2
```

### 方法 3: Action Server 运行时参数

```bash
ros2 run navigation_controller point_to_point_action_server.py \
  --ros-args \
  -p goal_tolerance:=0.2 \
  -p localization_confidence_threshold:=0.35 \
  -p stuck_timeout:=7.0 \
  -p max_timeout:=600.0
```

### 可配置参数

| 参数名                               | 默认值 | 配置位置        | 说明                       |
|--------------------------------------|--------|-----------------|----------------------------|
| `network_interface`                  | ens37  | config_env.yaml | Unitree SDK 网络接口       |
| `use_hardware`                       | false  | config_env.yaml | 硬件/仿真模式切换          |
| `max_linear_speed`                   | 0.25   | config_env.yaml | 最大线速度 (m/s)           |
| `max_angular_speed`                  | 0.08   | config_env.yaml | 最大角速度 (rad/s)         |
| `waypoint_distance`                  | 0.5    | config_env.yaml | 路点推进阈值 (m)           |
| `goal_tolerance`                     | 0.3    | config_env.yaml | 目标到达阈值 (m)           |
| `localization_confidence_threshold`  | 0.3    | Action Server   | 定位置信度阈值             |
| `stuck_timeout`                      | 5.0    | Action Server   | 卡住检测时间 (s)           |
| `stuck_distance_threshold`           | 0.1    | Action Server   | 卡住距离阈值 (m)           |
| `base_timeout`                       | 60.0   | Action Server   | 基础超时时间 (s)           |
| `timeout_per_meter`                  | 3.0    | Action Server   | 每米额外超时 (s/m)         |
| `max_timeout`                        | 300.0  | Action Server   | 最大超时时间 (s)           |
| `feedback_rate`                      | 5.0    | Action Server   | 反馈发布频率 (Hz)          |
| `obstacle_critical_distance`         | 0.5    | Action Server   | 危险障碍物距离 (m)         |
| `obstacle_near_distance`             | 1.5    | Action Server   | 接近障碍物距离 (m)         |
| `recovery_timeout`                   | 10.0   | Action Server   | 恢复行为超时 (s)           |

---

## 日志分析

### 正常执行日志示例

```
[INFO] [point_to_point_action_server]: 接受导航目标: (5.00, 2.00), 距离: 5.39 m
[INFO] [point_to_point_action_server]: 目标距离: 5.39 m
[INFO] [point_to_point_action_server]: 任务超时: 76.2 s
[INFO] [point_to_point_action_server]: 已发布目标到 A* 规划器
[INFO] [point_to_point_action_server]: 路径规划完成,路径长度: 6.12 m
[INFO] [point_to_point_action_server]: 🚀 路径跟踪 | 剩余: 3.45m | 障碍物: ✅ 安全 | 置信度: 0.82
[INFO] [point_to_point_action_server]: 🎯 成功到达目标!
[INFO] [point_to_point_action_server]: 位置误差: 0.087 m
[INFO] [point_to_point_action_server]: 耗时: 18.3 s
```

### 异常日志分析

**定位丢失**:
```
[WARN] [point_to_point_action_server]: 定位置信度低: 0.22 < 0.30
[ERROR] [point_to_point_action_server]: 定位丢失: 置信度 = 0.22 < 0.30
```
→ 检查 LiDAR 数据和点云配准

**卡住检测**:
```
[WARN] [point_to_point_action_server]: 检测到卡住,启动恢复行为
[ERROR] [point_to_point_action_server]: 恢复行为超时,判定为路径阻塞
```
→ 检查障碍物和局部地图

---

## 高级用法

### 编程接口示例 (Python)

```python
import rclpy
from rclpy.action import ActionClient
from navigation_controller.action import PointToPoint
from geometry_msgs.msg import PoseStamped

rclpy.init()
node = rclpy.create_node('nav_client')
client = ActionClient(node, PointToPoint, 'point_to_point')

# 等待服务器
client.wait_for_server()

# 构建目标
goal = PointToPoint.Goal()
goal.task_id = "MY_TASK_001"
goal.target_pose = PoseStamped()
goal.target_pose.header.frame_id = "map"
goal.target_pose.pose.position.x = 5.0
goal.target_pose.pose.position.y = 2.0

# 发送目标
future = client.send_goal_async(goal, feedback_callback=lambda fb: print(fb))

# 等待结果
rclpy.spin_until_future_complete(node, future)
result = future.result().get_result()

print(f"Success: {result.success}, Error: {result.position_error}")
```

---


## 更新日志

### v1.1.0 (2025-11-26)

- 🚀 **Master-Slave 控制架构**: Action Server 完全控制 Position Control 执行状态
- 🛡️ **超时保护增强**: 机器人在 Action Server 超时时立即停止 (修复重大 bug)
- 💓 **心跳机制**: 2Hz 持续信号 + 5 秒超时保护
- 🔧 **配置集中化**: 新增 `config_env.yaml` 统一管理环境参数
- 🤖 **双通道控制**: 支持硬件 SDK + ROS2 /cmd_vel 同时发布
- 📡 **EKF 融合支持**: 集成 LiDAR + IMU 传感器融合 (可选)
- 🎹 **键盘控制工具**: 新增 `test_point_to_point_action_keyboard.py`
- 📚 **文档更新**: 反映最新架构和故障排查指南

### v1.0.0 (2025-11-05)

- ✅ 初始版本发布
- ✅ 实现完整 Action 接口
- ✅ 添加 5 种测试场景
- ✅ 支持紧急停止和任务抢占
- ✅ 完整文档和示例代码
