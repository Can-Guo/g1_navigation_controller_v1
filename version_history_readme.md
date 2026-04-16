# 项目版本历史与差异说明（当前修订版）

## 1. 版本基线（截至当前工作区）
- 仓库路径：`/home/york/ros2_ws_nav`
- 当前最新提交（HEAD）：`68e78f2`（`20260313-2 add threshold level, fix lost`）
- 上一基线提交：`bf68a46`（`20260313-1 first commit for navigation`）
- 当前文档采用“**提交差异口径**”（commit-based），不再使用“暂存区差异口径”。

## 2. 提交级差异总览（`bf68a46` -> `68e78f2`）

本次修订聚焦于“定位稳定性、运行安全、可观测性”三条主线，关键变更文件如下：
- `src/navigation_controller/pkg/mid360_locate/src/locate_node_ros2.cpp`
- `src/navigation_controller/pkg/mid360_locate/src/main_ros2.cpp`
- `src/navigation_controller/pkg/mid360_locate/src/locate_node.h`
- `src/navigation_controller/config/ekf_localization.yaml`
- `src/navigation_controller/launch/fusion_ros2.launch.py`
- `src/navigation_controller/scripts/point_to_point_action_server.py`
- `src/navigation_controller/navigation_controller/nodes/point_to_point_action_server.py`
- `src/navigation_controller/scripts/dwa_planner_ros2_mid360.py`
- `src/navigation_controller/scripts/simulation_odometry_publisher.py`
- `src/navigation_controller/CMakeLists.txt`
- `version_history_readme.md`（本文档）

## 3. 关键能力升级与专业说明

### 3.1 定位鲁棒性：引入旋转去畸变（deskew）
- 在 `mid360_locate` 中加入基于 IMU 角速度 + 点时间戳的旋转去畸变流程。
- 对 MID360 转身场景（墙面扇形、配准抖动）有直接抑制作用。
- 当去畸变条件不足时保留回退路径，避免系统硬失败。

**专业背景**
- 旋转期间，一帧点云不是“同一时刻”采样；不做 deskew 会把动态姿态误差映射为几何畸变。
- 对配准器（NDT/GICP）而言，这类畸变常表现为 yaw 漂移与局部极值吸附，进而触发位姿跳变。

### 3.2 定位状态机：自动初始化 + 可控重定位 + ready 门控
- 启动支持默认原点自动初始化（可参数化）。
- 保留 RViz `/initialpose` 运行时重定位能力（可开关）。
- 增加 `/localization_ready`，通过连续高置信度计数（streak）判定是否进入可导航状态。

**专业背景**
- 实际部署中，“能输出位姿”不等于“位姿可用于任务决策”。
- `ready` 门控将算法质量转换为上层可消费状态，能显著降低“未稳定位姿驱动控制”的风险。

### 3.3 融合层优化：EKF 对旋转阶段更稳健
- `imu0_config` 调整为不融合 IMU 绝对姿态角（roll/pitch/yaw），保留角速度通道。
- 提高 LiDAR yaw 对应协方差，减少急转时被瞬时配准噪声牵引。

**专业背景**
- 人形机器人动态激烈，IMU 姿态解算易受漂移和机体振动干扰。
- “少信绝对角，多信角速度 + 配准约束”通常更利于转向阶段的连续性和抗抖性。

### 3.4 导航安全：置信度升级为 GREEN / YELLOW / RED 三段
- 在点到点任务服务端增加三阈值参数：
  - `localization_confidence_green_threshold`
  - `localization_confidence_red_threshold`
  - `localization_red_hold_timeout`
- GREEN 正常、YELLOW 警告、RED 危险；RED 连续超时触发任务中止（定位丢失保护）。

**专业背景**
- 单阈值“过/不过”在现场运维中信息粒度不足。
- 三段分级更符合安全工程实践：先预警、再限时观测、最后保护停机。

### 3.5 启动可观测性与安全语义（`fusion_ros2.launch.py`）
- 启动日志按运行参数动态展示控制模式，不再假设硬件控制默认开启。
- 明确：仅在显式 `use_hardware:=true` 时才进入真实硬件控制链路。
- 启动日志新增定位置信度等级定义（GREEN/YELLOW/RED）及当前状态说明。

**专业背景**
- 机器人系统事故里，人因误判常来自“日志文本与实际配置不一致”。
- 动态日志绑定真实参数，能降低误操作概率，提升现场可审计性。

### 3.6 接口一致性与仿真补链
- 修复 `/obstacle_status` 消息类型不一致问题（与控制侧预期对齐）。
- 新增 `simulation_odometry_publisher.py`，在无实机/无定位时补齐仿真里程计来源。
- 在 `CMakeLists.txt` 中补充该脚本安装项，确保部署后可执行。

## 4. 当前系统形态定位（SLAM 视角）
- 当前系统属于“先验地图定位 + 规划控制”架构（Localization + Navigation）。
- 不属于完整在线 SLAM（未覆盖在线建图、回环闭环、图优化后端）。
- 本轮修订重点是“工程可用性与安全稳定性”，而非扩展到全链路 SLAM 后端。

## 5. 运行建议（当前版本）
- 默认安全启动（不启用硬件控制）：
  - `ros2 launch navigation_controller fusion_ros2.launch.py`
- 真实机器人启用（需显式）：
  - `ros2 launch navigation_controller fusion_ros2.launch.py use_hardware:=true`
- 纯仿真联调（无 Livox/无定位）：
  - `ros2 launch navigation_controller fusion_ros2.launch.py use_hardware:=false use_livox:=false use_localization:=false`

## 6. 文档维护建议
- 后续每次合入建议保持“提交级版本条目”更新（版本号、目标、关键文件、风险点、回归建议）。
- 若进入多分支协作，建议新增“变更影响矩阵”（定位/规划/控制/安全/运维）便于评审与回滚。
