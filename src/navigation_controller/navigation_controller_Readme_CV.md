> 项目文档说明书：G1 人形机器人导航控制器（navigation_controller）

> 本文档基于对 `./navigation_controller` 源码、launch、配置、地图与文档的实际核查编写。
- 目标平台：Unitree G1 人形机器人（单轴移动：每次只沿 X 或 Y 轴行走）
- 传感器：Livox MID360 非重复扫描固态激光雷达（内置 IMU）
- 中间件：ROS 2 Humble（Python 3.10 + C++17）
- 定位：基于先验 PCD 地图的 LiDAR 点云配准（NDT 初始化 + FastGICP 跟踪）
- 导航：自研 A\* 全局规划 + DWA 局部规划 + 位置控制（非 Nav2 完整方案）
- 集成：通过 ROS 2 Action 协议与 MCB（主控板）通信，采用主从门控架构

---

## 目录

1. [项目概述](#1-项目概述)
2. [系统架构](#2-系统架构)
3. [数据流与通信拓扑](#3-数据流与通信拓扑)
4. [定位子系统：SLAM/地图配准原理](#4-定位子系统)
5. [导航子系统：规划与控制算法](#5-导航子系统)
6. [与 MCB 的通信协议](#6-与-mcb-的通信协议)
7. [工程实现细节](#7-工程实现细节)
8. [快速开始](#8-快速开始)

---

## 1. 项目概述

本工程是一个**面向人形机器人 G1 的 LiDAR 自主导航软件包**，以「单包 + 内部 vendor 依赖」的方式组织，形成可整体部署到机器人的导航单元。

核心结论（重要）：

- **本工程本身不是在线 SLAM 系统**，而是「**地图定位 + 导航**」栈：它加载一份**预先构建好的 PCD 点云地图**，在线仅做「点云配准定位」，不做地图的在线增量更新。
- 地图的来源未包含在本工程内；由默认地图路径 `fast_lio/PCD/map.pcd`（见 `pkg/mid360_locate/src/main.cpp`、`main_ros2.cpp` 中的默认参数）**推断**为 FAST-LIO 系列 LiDAR-惯性 SLAM 离线生成。
- 定位算法采用「**粗到细两阶段配准**」：首帧用 **NDT**（Normal Distributions Transform）做全局初始化，后续帧用 **FastGICP**（快速广义 ICP，平面到平面）做逐帧跟踪，并带有**置信度评估与位姿跳变门控**。
- 导航采用「**A\* 全局规划 + DWA 局部避障 + 位置控制**」三层自研栈，为适配 G1 单轴移动特性，A\* 使用曼哈顿启发式并把路径轴对齐。
- 与上位 MCB 通过 **ROS 2 Action**（`PointToPoint`）通信，Action 服务器作为**主（Master）**，通过 `/navigation_active` 心跳门控**从（Slave）**位置控制节点，实现任务生命周期的安全管控。

---

## 2. 系统架构

### 2.1 模块划分

| 层 | 模块 | 语言/形态 | 作用 |
|---|---|---|---|
| 定位层 | `pkg/mid360_locate`（locate_node） | C++ / ROS 2 | 地图加载、NDT/GICP 配准、置信度评估、发布位姿/TF/点云 |
| 规划层 | A\* 全局规划器 | Python | 基于占用栅格 + 膨胀地图搜索全局路径 |
| 规划层 | DWA 局部规划器 | Python | 基于实时点云聚类避障，输出局部速度指令 |
| 执行层 | 位置控制 | Python | 路径点跟踪、DWA 命令仲裁、输出 `/cmd_vel`、主从门控 |
| 任务层 | `point_to_point_action_server` | Python | Action 服务器（Master），任务生命周期、心跳门控、反馈/结果 |
| 接口层 | `robot_interfaces` | IDL | `PointToPoint.action` 共享接口定义 |
| 支撑层 | `pkg/livox_ros_driver2`、`pkg/fast_gicp` | C++ | MID360 驱动、FastGICP/NDT 配准库（第三方 vendor） |
| 融合层 | `robot_localization` EKF | C++（外部依赖） | LiDAR+IMU 位姿融合（50 Hz） |

### 2.2 架构总览图

```
┌───────────────────────────────────────────────────────────────────────────┐
│                           上层应用 / 控制方                                 │
│        MCB（主控板）· RViz（2D Nav Goal / 2D Pose Estimate）· 演示客户端   │
└───────────────────────────────┬───────────────────────────────────────────┘
                                │  ROS2 Action: point_to_point
                                ▼
┌───────────────────────────────────────────────────────────────────────────┐
│                       任务管理层 —— Master                                  │
│              point_to_point_action_server（scripts/ 版）                   │
│   · 目标校验（frame_id / 定位就绪 / 紧急停止门）                             │
│   · 动态超时 · 抢占 · 定位丢失检测 · 卡住/恢复监控 · 进度反馈                 │
│   · /navigation_active 心跳门控（2 Hz）→ 控制执行许可                        │
└───────────────────────────────┬───────────────────────────────────────────┘
                                │ /move_base_simple/goal
                                ▼
┌───────────────────────────────────────────────────────────────────────────┐
│                    导航规划与控制层 —— Slave                                 │
│                                                                            │
│   astar_planner（全局 A*）                                                  │
│     地图膨胀 → 曼哈顿A* → 平滑+轴对齐 ──/astar_path──►                      │
│                                                                  ▼         │
│   dwa_planner（局部 DWA）                                                  │
│     点云 → FOV过滤 → DBSCAN聚类 → 轨迹采样评分 → 逃逸行为                    │
│                                ──/dwa_cmd_vel──►                           │
│                                                                  ▼         │
│   g1_position_control（位置控制）                                            │
│     主从门控 → 路径点跟踪 → DWA命令仲裁 → /cmd_vel → G1硬件/仿真             │
└───────────────────────────────┬───────────────────────────────────────────┘
                                │ /odometry/map  /aligned  /localization_confidence
                                ▼
┌───────────────────────────────────────────────────────────────────────────┐
│                        定位层 locate_node（C++）                            │
│   地图预处理 → NDT 首帧初始化 → FastGICP 逐帧跟踪                           │
│   → 置信度/内点率/跳变评估 → 接受/拒绝 → 发布位姿/TF/置信度                  │
└───────────────────────────────┬───────────────────────────────────────────┘
                                │ /livox/lidar（CustomMsg, BEST_EFFORT）
                                ▼
┌───────────────────────────────────────────────────────────────────────────┐
│                        传感器 / 执行层                                       │
│   Livox MID360（livox_ros_driver2）              Unitree G1（unitree_sdk） │
└───────────────────────────────────────────────────────────────────────────┘
```

### 2.3 TF 坐标树

```
map ──→ odom ──→ base_link ──→ pelvis ──→ lidar_frame ──→ livox_frame
   ↑EKF       ↑locate_node / EKF
```

| 边 | 发布者 | 说明 |
|---|---|---|
| `map → odom` | EKF（动态）/ 静态发布器（仿真模式，单位变换） | 仿真模式下使用单位静态变换 |
| `odom → base_link` | EKF / 静态发布器 | 仿真模式下固定于原点 |
| `map → base_link` | locate_node（10 Hz 主循环直接发布） | 定位层直接给出的机器人位姿 |
| `base_link → pelvis` | 静态发布器 | 机身内部零偏移 |
| `pelvis → lidar_frame` | 静态发布器 | 假设 LiDAR 安装在骨盆上方 0.3 m、无旋转 |
| `lidar_frame → livox_frame` | 静态发布器 | 单位变换，兼容 Livox 驱动坐标系 |

> 注意：硬件模式下 locate_node 直接发布 `map → base_link`，同时 EKF 发布 `map → odom`
> 与 `odom → base_link`，即 `base_link` 存在两条 TF 路径，部署时需核对 TF 树一致性。

### 2.4 目录结构与文件职责

> 本目录结构是AI自动生成的，需要用户自行确认是否与项目架构完整吻合，注意

```
navigation_controller/
├── launch/fusion_ros2.launch.py      # 系统级启动编排（TF/地图/定位/EKF/导航/RViz/LiDAR）
├── config/
│   ├── config_env.yaml               # 环境参数：网络接口、地图、速度上限、硬件/仿真开关
│   └── ekf_localization.yaml         # EKF 融合参数（LiDAR+IMU、50 Hz）
├── robot_interfaces/                 # PointToPoint.action 共享接口（独立包，MCB 端共用）
├── pkg/
│   ├── mid360_locate/                # 定位节点（ROS2 版实际编译：locate_node_ros2.cpp）
│   ├── fast_gicp/                    # FastGICP/NDT 配准库（vendor）
│   └── livox_ros_driver2/            # MID360 驱动（vendor）
├── scripts/                          # 部署版节点（launch 实际执行）
│   ├── astar_planner_ros2_Manhattan.py      # A* 全局规划
│   ├── dwa_planner_ros2_mid360.py           # DWA 局部避障
│   ├── g1_position_control_ros2.py          # 位置控制（主从门控 Slave）
│   ├── point_to_point_action_server.py      # Action 服务器（Master）
│   └── ros1/                               # 旧 ROS1 版本
├── navigation_controller/nodes/      # 旧版节点（Python 模块，与 scripts/ 分叉）
├── map/                              # 多场景 PCD + PGM 地图（map.pcd / map.pgm / map.yaml）
├── docs/                             # 协议、快速开始等文档
└── CMakeLists.txt / package.xml      # ament_cmake 构建（安装 scripts/ 为可执行）
```

---

## 3. 数据流与通信拓扑

### 3.1 节点—话题拓扑（以 launch 实际启动的 `scripts/` 部署版为准）

```
[locate_node]                          [nav2_map_server]
  ├─ 订阅 /initialpose                       └─ 发布 /map (TransientLocal)
  ├─ 订阅 /livox/lidar  (BEST_EFFORT)
  └─ 发布
      ├─ /odometry/map        (nav_msgs/Odometry, frame map→base_link)  10Hz
      ├─ /aligned             (sensor_msgs/PointCloud2, frame map)
      ├─ /veh_pose            (geometry_msgs/PoseStamped)
      ├─ /path                (nav_msgs/Path, 逐帧累积)
      ├─ /localization_confidence (std_msgs/Float32)
      ├─ /points_map          (PointCloud2, 1Hz)
      └─ TF: map→base_link

[astar_planner]  (scripts/astar_planner_ros2_Manhattan.py)
  订阅: /map · /move_base_simple/goal · /odometry/map
  发布: /astar_path · /inflated_map

[dwa_planner]    (scripts/dwa_planner_ros2_mid360.py)
  订阅: /aligned · /odometry/map · /astar_path
  发布: /dwa_cmd_vel · /dwa_path_visualization · /detected_obstacles
        · /filtered_points · /obstacle_status (Int32)

[g1_position_control] (scripts/g1_position_control_ros2.py)
  订阅: /odometry/map · /move_base_simple/goal · /astar_path
        · /dwa_cmd_vel · /obstacle_status (Point) · /navigation_active (Bool)
  发布: /cmd_vel

[point_to_point_action_server] (scripts/)
  订阅: /odometry/map · /astar_path · /dwa_obstacles(无发布方)
        · /localization_confidence · /emergency_stop
  发布: /move_base_simple/goal · /navigation_active (2Hz 心跳)
  提供: Action 服务器 `point_to_point`（robot_interfaces/PointToPoint）

[ekf_localization]（robot_localization）
  订阅: /odometry/map · /livox/imu
  发布: /odometry/ekf · /diagnostics · TF map→odom, odom→base_link
```

### 3.2 一次导航任务的时序（MCB → 到达）

```
MCB ──Goal{task_id, target_pose}──► ActionServer
  1. 校验通过 → GoalResponse.ACCEPT
  2. ── /move_base_simple/goal ──► A*（全局规划）
  3. ◄── /astar_path ── ActionServer 记录路径、切换状态 PLANNING→FOLLOWING
  4. ── /navigation_active=True（2Hz 心跳持续发送）──► PositionControl（授权执行）
  5. DWA 输出 /dwa_cmd_vel；PositionControl 仲裁后 ──/cmd_vel──► G1
  6. ActionServer ◄── /odometry/map, /localization_confidence（实时监控）
  7. ActionServer ──Feedback（2–5 Hz：状态/障碍物/距离/置信度/ETA）──► MCB
  8. 距离<容差 且 速度<0.05m/s → ──Result{success, status_code, final_pose, position_error}──► MCB
  9. ── /navigation_active=False ──► PositionControl 立即停车
```

---

## 4. 定位子系统

> 本子系统是整套导航的「眼睛」。定位质量直接决定路径跟踪与速度上限，工程上为它配置了
> 一系列保守参数（低速 0.2 m/s、低角速度 0.08 rad/s），核心原因即「旋转时点云配准易发散」。

### 4.1 离线建图（地图来源，推断 + 工具）

- 在线定位所依赖的 `map.pcd` 由**离线 SLAM** 生成。从默认参数路径 `fast_lio/PCD/map.pcd`
  **推断**使用 FAST-LIO / FAST-LIO2（LiDAR-惯性紧耦合里程计）建图：IMU 提供运动先验，
  激光帧与增量式 ikd-Tree 地图配准并在线修正漂移，最终输出全局一致的点云地图。
- 二维栅格地图 `map.pgm` 由 `map/pcd2pgm.py` 从点云降维生成：按 Z 高度带 `[-0.8, 1.0] m`
  过滤地面与天花板，保留障碍物点，以 0.05 m/像素投影为占位栅格并输出 `map.yaml`
  （脚本产物中的 `resolution` 字段需手工修正，仓库内已交付的 `map/map.yaml` 已修正为 0.05）。

### 4.2 地图预处理（`initializeMap`，`locate_node_ros2.cpp`）

加载 PCD 后按序处理：

1. **体素下采样**（0.2 m leaf）——控制点密度与计算量；
2. **RANSAC 平面分割**（`SACMODEL_PLANE`，Z 轴约束，距离阈值 0.1 m）——提取主地面法线；
3. **平面旋转对齐**（`Quaternionf::FromTwoVectors` 将地面法线旋到 +Z）——地图被「放平」，
   并写入 `T_to_plane` 的平移项，使地面位于 z≈0；
4. **法向空间采样至一半点数**；
5. 再按标定参数 `map_roll / map_pitch / map_yaw` 旋转，完成坐标系约定。

### 4.3 逐帧定位流程（livox_callback → pointCloudProcess → registration）

```
逐帧（每个 Livox 数据包）
  │
  ├─ 1. 原始帧滤波：保留 XY 距离²∈[0.25, 2500] m²（即 0.5~50 m 环形带）
  │         坐标轴约定：(x, −y, −z)（与 MID360 安装朝向的固定握手，硬编码于源码）
  ├─ 2. 法向空间采样至固定 4000 点
  │         · NormalEstimation(k=20) 估计法向
  │         · NormalSpaceSampling：按法向 8×8×8 直方图分箱、均匀抽样
  │         · 目的：保留边缘/角点等约束丰富结构，避免被大面积墙面点稀释
  ├─ 3. 自适应局部地图裁剪（半径 15~80 m，见 §4.7）
  └─ 4. 配准：
         · 首帧  → NDT（全局初始化，初值来自 RViz /initialpose）
         · 后续帧 → FastGICP（跟踪，初值=上一帧位姿）
         · 配准后 → 质量评估 → 接受/拒绝 → 更新 veh_pose
  └─ 发布：/odometry/map、/aligned、/localization_confidence、TF map→base_link
```

其中「法向空间采样」是整个流程的**关键降本手段**，其意图可从 PCL 调用直接读出：

```cpp
// locate_node_ros2.cpp —— 法向估计（k=20 邻域拟合局部平面法线）
pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
ne.setKSearch(20);
ne.compute(*normals);

// 法向空间采样：按法向在 8×8×8 球面直方图中均匀抽样，
// 保证平面、边缘、角点的点在采样结果中都有代表性
pcl::NormalSpaceSampling<pcl::PointNormal, pcl::PointNormal> nss;
nss.setBins(8, 8, 8);
nss.setSample(target_points);          // 每帧固定 4000 点
nss.filter(*sampled_cloud);
```

> 为什么这么做？若按空间均匀采样，大面积墙面会占据绝大多数样本，而真正对位姿约束
> 「贡献最大」的是边缘、角点、门窗等结构。法向空间采样把样本按几何类型均分，
> 使每帧点云的信息熵最大化，同时把配准规模固定在 4000 点、计算量可预期。

### 4.4 NDT 初始化（全局配准）

`pcl::NormalDistributionsTransform`，参数：分辨率 1.0 m、步长 0.1、最大迭代 30、收敛阈值 1e-8。

```cpp
// locate_node_ros2.cpp —— 首帧冷启动
pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
ndt.setInputSource(points);            // 当前扫描（已采样至 4000 点）
ndt.setInputTarget(local_map);         // 局部地图
ndt.setTransformationEpsilon(1e-8);
ndt.setStepSize(0.1);
ndt.setMaximumIterations(30);
ndt.setResolution(1.0);                // 体素栅格尺寸
ndt.align(*aligned, veh_pose);         // 以 RViz /initialpose 为初值
veh_pose = ndt.getFinalTransformation();
```

算法原理：

- 将目标点云（局部地图）划分为**体素栅格**，每个体素内用**高斯分布**（均值 μ、协方差 Σ）
  描述点的统计特性；
- 源点云在待求变换下投影到目标体素，目标函数为**马氏距离之和**：
  `Σ_i exp(−½ (p_i−μ_i)ᵀ Σ_i⁻¹ (p_i−μ_i))`，通过牛顿法/Gauss-Newton 迭代求解；
- 相比点对点 ICP，NDT 不依赖显式最近邻点对，**收敛盆地大**，适合处理 RViz 手动初始位姿
  与真实位姿存在较大偏差的冷启动场景。

### 4.5 FastGICP 跟踪（逐帧精配准）

`fast_gicp::FastGICP`，参数：8 线程、最大对应距离 0.5 m、初值为上一帧位姿。

```cpp
// locate_node_ros2.cpp —— 跟踪帧
fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> gicp;
gicp.setInputSource(points);
gicp.setInputTarget(local_map);
gicp.setNumThreads(8);
gicp.setMaxCorrespondenceDistance(0.5);
gicp.align(*aligned, veh_pose);        // 以上一帧位姿为初值（预测式数据关联）
```

算法原理：

- **GICP** 是「点对点 ICP」与「点对平面 ICP」的统一推广：为每个点用其邻域点估计一个
  **局部表面协方差**（描述局部形状是平面、直线还是散点）；
- 目标为最小化**点分布之间的广义距离**：
  `Σ_i d_iᵀ (C_i^B + T C_i^A Tᵀ)⁻¹ d_i`，其中 `d_i` 为对应点差向量，`C^A/C^B` 为两帧协方差。
  当表面为平面时退化为平面到平面配准，鲁棒性和精度优于点到点/点到面；
- **FastGICP**（Koide 等）的关键加速：对目标点云**按体素预计算协方差**（体素化协方差），
  运行期只查表即可获得高质量 `C^B`，并支持多线程与 CUDA 版本，实时性显著优于传统 GICP。

### 4.6 配准质量评估与跳变门控（G1 行走适配的关键改进）

ROS2 版 locate_node 在每次配准后评估 `RegistrationQuality`。综合置信度（0~1）由
`locate_node.h` 中的实现计算：

```cpp
// locate_node.h —— 综合置信度（0~1，越高越可靠）
float getConfidence() const {
    float fitness_norm   = std::max(0.0f, 1.0f - fitness_score / 0.5f);  // 适应度归一化
    float iteration_penalty = num_iterations < 20 ? 1.0f : 0.7f;          // 迭代多→收敛难
    float change_penalty = transformation_change < 0.5f ? 1.0f
                         : (transformation_change < 1.0f ? 0.8f : 0.5f); // 位移突增→降权
    float density_weight = std::min(1.0f, point_density / 1000.0f);      // 点数不足→降权
    return (0.35f * fitness_norm + 0.30f * inlier_ratio +
            0.20f * iteration_penalty + 0.15f * density_weight)
           * change_penalty;
}
```

各输入的含义：

| 指标 | 计算 | 含义 |
|---|---|---|
| fitness_score | 配准器返回的均方距离 | 越小越好 |
| inlier_ratio | 变换后源点云经 KD-Tree 找最近邻，距离 <0.5 m 的点占比 | 越大越可靠 |
| transformation_change | 相对上一帧位姿的位移范数 | 用于检测异常跳变 |
| point_density | 采样后点数 | 点太少则信息不足 |

接受门控（硬编码阈值，`locate_node_ros2.cpp`）：

```cpp
const float CONFIDENCE_THRESHOLD = 0.15f;   // 置信度过低 → 拒绝
const float MAX_POSITION_JUMP   = 4.0f;     // 位置跳变 > 4 m → 拒绝
const float MAX_ROTATION_JUMP   = 2.0f;     // 旋转跳变 > 2 rad → 拒绝

// 被拒绝时保持上一帧位姿（宁停不错），并发布警告
veh_pose = prev_veh_pose;
```

这是工程上用于防范「G1 行走晃动导致误匹配」的关键机制——宁可短暂停顿，也不让位姿跳变
打乱控制环。置信度同时发布到 `/localization_confidence`，供 Action 服务器做「定位丢失」判定。

### 4.7 自适应局部地图半径

按运动速度与角速度动态调整局部地图裁剪半径：

```cpp
// locate_node_ros2.cpp —— computeAdaptiveLocalMapRadius
const float BASE_RADIUS = 25.0f, MIN_RADIUS = 15.0f, MAX_RADIUS = 80.0f;
float velocity_factor = 1.0f + std::min(velocity / 1.0f, 2.0f);       // 速度越快范围越大
float angular_factor  = 1.0f + std::min(angular_velocity / 0.3f, 1.5f); // 转向时需更大范围
float radius = BASE_RADIUS * velocity_factor * angular_factor;
radius = std::max(MIN_RADIUS, std::min(radius, MAX_RADIUS));          // 钳制 [15, 80] m
```

静止/低速时用小半径省算力，转向/高速时放大以覆盖足够约束特征；并叠加
「加/减速剧烈时再 ×1.2」的裕度。代码注释说明该参数针对 G1 步行时的周期性晃动设计。

---

## 5. 导航子系统

### 5.1 全局规划：A\*（`scripts/astar_planner_ros2_Manhattan.py`）

- **代价地图**：接收 `/map`，用 `scipy.ndimage.distance_transform_edt` 做**精确欧氏距离变换**
  生成膨胀地图（部署版 `inflation_radius_m = 0.15 m`，另含机器人半径 0.35 m）；膨胀区标记为
  99 并以 2 s 周期重新发布 `/inflated_map` 供可视化。
- **搜索**：经典 A\* + 二叉堆，4 连通邻居、单位代价；起点=当前 `/odometry/map` 位姿，
  终点=RViz 2D Nav Goal。
- **启发式**：**曼哈顿距离**。这是对 G1「单轴移动」约束的显式适配——机器人每次只能走 X 或
  Y 方向，曼哈顿距离是该约束下的可采纳下界：

```python
# astar_planner_ros2_Manhattan.py —— 曼哈顿启发式
def heuristic(self, x, y):
    dx = abs(self.goal_x - x)
    dy = abs(self.goal_y - y)
    return dx + dy                 # L1 范数：单轴移动机器人的最优下界估计
```

- **路径后处理**（`smooth_path_for_g1`）：
  1. 角度阈值去冗余（夹角 <30° 的点剔除）；
  2. 线性插值加密至 ~0.3 m 间隔；
  3. 3 点移动平均平滑，并**逐点碰撞回检**（平滑点不安全则退回原始点）；
  4. **轴对齐**：把斜线段分解为「先 X 后 Y（或先 Y 后 X）」的 L 形路径，保证每一段都单轴可行：

```python
# astar_planner_ros2_Manhattan.py —— align_path_to_axes
# 斜线段分解为 L 形：先走主方向，再走次方向
if abs(dx) >= abs(dy):
    mid_point = (next_pt[0], curr[1])   # 先走 X，再走 Y
else:
    mid_point = (curr[0], next_pt[1])   # 先走 Y，再走 X
if mid_point not in aligned and self.is_valid(*mid_point):
    aligned.append(mid_point)
```

- 工程细节：起点未就绪时的**目标点缓存重试**（0.5 s 定时器）；起点/终点合法性详细诊断；
  路径落盘为 CSV。

### 5.2 局部规划：DWA（`scripts/dwa_planner_ros2_mid360.py`）

- **障碍物感知**（10 Hz，直接消费 `/aligned`，`target_frame=map`）：
  - 距离 `detection_range`（launch 传 1.5 m）内、前向 ±90°、高度 `[-0.3, 0.6] m` 过滤；
  - **DBSCAN 式连通聚类**（`clustering_distance` 0.4 m），保留点数 ≥ `min_obstacle_points`(60)
    的聚类，输出**质心障碍物**。
- **轨迹采样**：每周期生成约 100 条候选轨迹（速度×转向随机组合），按「偏好方向 60% /
  直行 30% / 随机 10%」加权；每条轨迹用自行车模型前向积分 `time_horizon`(1.5 s)。
- **代价函数**（加权标量化）：

```python
# dwa_planner_ros2_mid360.py —— choose_best_path 核心评分
goal_distance_score = -goal_distance * 15        # 目标吸引（-15/米）
heading_score       = -heading_diff  * 10        # 朝向对准（-10/rad）
collision_risk      = self.check_for_collisions(path) * 2.0
smoothness_score    = -3.0 * speed_change - 5.0 * turn_change
speed_reward        = speed * 3.0                # 鼓励合理高速
total_score = goal_distance_score + heading_score + collision_risk \
              + smoothness_score + speed_reward + direction_bonus
```

  其中 `collision_risk` 采用**连续惩罚**而非传统 DWA 的硬约束阶梯函数：

```python
# dwa_planner_ros2_mid360.py —— check_for_collisions
if effective_distance < ROBOT_SAFETY_RADIUS:     # < 0.4 m 危险区：指数惩罚
    collision_risk -= 100 * math.exp(-effective_distance / 0.2)
elif effective_distance < 1.0:                   # 警告区：反比例惩罚
    collision_risk -= 30.0 / (effective_distance + 0.1)
```

  连续惩罚避免评分在阈值处突变，使轨迹选择在临近障碍物时平滑减速而非「急刹/硬拐」。

- **速度平滑**：对 v、ω 做一阶低通（`smooth_factor` 0.3），避免对腿式平台的冲击。

### 5.3 恢复与逃逸行为

- **卡住检测**：5 s 内累计位移 <0.15 m 且存在线速度指令（且非原地旋转）→ 判卡住。
- **逃逸行为**（`escape_mode` 三级状态机）：
  - 将 360° 划分为 8 个扇区，统计各扇区障碍物密度与平均距离，评分 `avg_dist/(count+1)`，
    选出最空旷方向；
  - 前方扇区（0/1/7）密集（>5 个）→ 阶段 0 **后退**（2 s）→ 阶段 1 **转向最空旷方向**（3 s）
    → 阶段 2 **朝该方向前进**（2 s），结束后复位并恢复正常导航。
- 该机制同样体现在 Action 服务器的 `STATE_RECOVERING` / `STATE_STUCK` 状态中。

### 5.4 位置控制与主从门控（`scripts/g1_position_control_ros2.py`）

- **主从门控**：订阅 `/navigation_active`（Bool），仅在激活时发布 `/cmd_vel`；
  激活信号 **5 s 无更新则视为 Action 服务器异常并强制停车**（看门狗）：

```python
# scripts/g1_position_control_ros2.py —— 主从门控
def control_loop(self):
    # 主从门控：无激活信号 → 立即停车
    if not self.navigation_active:
        self._send_velocity_command(0.0, 0.0, 0.0)
        return

    # 5 秒心跳看门狗：Action 服务器掉线/崩溃 → 停车兜底
    elapsed = (self.get_clock().now() - self.last_active_time).nanoseconds / 1e9
    if elapsed > 5.0:
        self._send_velocity_command(0.0, 0.0, 0.0)
        return
```

- **路径点跟踪**：到达 `waypoint_distance`(0.5 m) 内前进到下一个路径点；
  `goal_tolerance`(0.20 m) 判到达。
- **控制律**：线速度随距离与航向偏差缩放；|Δheading|>0.4 rad 时原地转向；接近目标减速。
- **超时保护**：按目标距离分档（≤3 m→30 s，3–10 m→60 s，10–20 m→120 s，>20 m→180 s）。
- **仲裁**：DWA 避障激活期间直接采用 `/dwa_cmd_vel`（0.5 s 有效性看门狗），否则走 A\* 路径跟踪。

---

## 6. 与 MCB 的通信协议

### 6.1 为什么用 Action 而非普通话题/服务

MCB（主控板，机器人的业务大脑）向导航系统下达「去某地」这类**长时间任务**。ROS 2 Action
（建立在话题/服务之上，但带服务端状态机）恰好覆盖这类语义，选择它的具体理由：

1. **任务生命周期完整**：Goal（含 Accept/Reject）→ 执行 → Feedback（异步周期反馈）→ Result。
   普通消息或请求/应答无法表达「长任务 + 中途状态 + 最终结果」。
2. **可抢占（Preemption）**：新目标到来可取消旧目标，MCB 可随时切换任务，符合巡检/配送场景。
3. **结构化失败诊断**：Result 带 `status_code`（超时/定位丢失/路径阻塞/目标不可达/抢占/紧急停止），
   MCB 能依据失败原因做上层决策（重试、改点、告警）。
4. **共享接口防漂移**：`PointToPoint.action` 独立放在 `robot_interfaces` 包，MCB 与导航端
   构建同一份 IDL，避免两端协议不一致。
5. **标准消息复用**：`target_pose` 直接复用 `geometry_msgs/PoseStamped`（frame_id=map），
   与 RViz 2D Nav Goal 天然兼容。

### 6.2 协议定义（`robot_interfaces/action/PointToPoint.action`）

**Goal**：`task_id`（任务唯一标识，MCB 日志/并发追踪）+ `target_pose`（目标位姿，frame=map）。

**Feedback**（建议 5 Hz）：
- `current_pose` / `current_velocity`（实时位姿速度）；
- `navigation_state`：`PLANNING=0 / FOLLOWING=1 / AVOIDING=2 / RECOVERING=3 / STUCK=4`；
- `obstacle_proximity`：`CLEAR=0 / NEAR=1(0.5–1.5m) / CRITICAL=2(<0.5m)`；
- `distance_to_goal`、`path_length_remaining`、`estimated_time_remaining`（进度/ETA）；
- `localization_confidence`（0~1，>0.7 优秀 / 0.3–0.7 正常 / <0.3 危险）；
- `debug_message`。

**Result**：
- `success`、`final_pose`、`position_error`（到达误差）；
- `status_code`：`SUCCESS=0 / TIMEOUT=1 / LOCALIZATION_LOST=2 / PATH_BLOCKED=3 /
  GOAL_UNREACHABLE=4 / PREEMPTED=5 / EMERGENCY_STOP=6`。

### 6.3 主从心跳机制（安全设计核心）

Action 服务器（Master）在任务期间以 **2 Hz** 发布 `/navigation_active=True`（心跳），
任务结束/取消/异常时发布 `False`。位置控制（Slave）仅在收到心跳且 5 s 内未超时才允许输出速度。

该设计解决的问题：

- **无任务时机器人绝不移动**：即使 `/astar_path` 等陈旧数据仍在线，只要无心跳则强制停车；
- **Action 服务器崩溃兜底**：心跳中断 5 s 看门狗停车，避免「服务器死了机器人还在动」；
- **任务间状态隔离**：每次激活时位置控制完整复位（路点索引、超时计时、位置历史），
  防止上一任务的残留状态污染下一任务。

> 说明：该主从机制存在于**部署版 `scripts/`** 的 Action 服务器与位置控制中；
> `navigation_controller/nodes/` 下的旧版无此机制。

### 6.4 一次完整任务时序

```
 MCB                     ActionServer                A* / DWA / Ctrl             G1
  │──Goal{task_id,target}──►│                          │                       │
  │◄──Accept───────────────│──/move_base_simple/goal──►│A* 规划               │
  │                         │◄──/astar_path─────────────│                       │
  │                         │──/navigation_active=1(2Hz心跳)──►│Ctrl 授权       │
  │                         │                          │──/cmd_vel─────────────►│
  │◄──Feedback(2-5Hz)───────│◄──/odometry/map  /confidence（实时监控）          │
  │                         │                          │                       │
  │◄──Result{success,status_code,final_pose,error}─────│                       │
  │                         │──/navigation_active=0────►│Ctrl 停车              │
```

### 6.5 紧急停止通道

`/emergency_stop`（Bool）是一条独立于 Action 的**最高优先级安全通道**：
- 目标校验阶段（`goal_callback`）若处于急停状态直接 **REJECT**；
- 执行循环每周期检查急停位，置位则 `abort` 并以 `STATUS_EMERGENCY_STOP` 返回；
- 测试/演示客户端可独立发布该话题，不依赖是否有进行中的任务。

---

## 7. 工程实现细节

### 7.1 QoS 策略

| 类别 | QoS | 应用 |
|---|---|---|
| 传感器流（点云、IMU、LiDAR 定位） | BEST_EFFORT, KEEP_LAST | `/aligned`、`/livox/lidar`、`/odometry/map` |
| 控制/规划（速度、路径、目标） | RELIABLE, KEEP_LAST | `/cmd_vel`、`/astar_path`、`/move_base_simple/goal` |
| 地图 | RELIABLE + TRANSIENT_LOCAL | `/map`（后加入节点也能收到） |

### 7.2 ROS1 / ROS2 迁移与双套代码

- 本工程由 ROS1（Noetic）工程迁移而来：`scripts/ros1/` 保留 ROS1 版本，
  `scripts/` 与 `navigation_controller/nodes/` 为 ROS2 版本；
- 关键事实：**launch 实际启动的是 `scripts/` 下脚本**（CMakeLists 以
  `install(PROGRAMS scripts/...)` 安装为可执行程序），而 `navigation_controller/nodes/`
  是作为 Python 模块安装。两者在多个节点上**内容不一致**，`scripts/` 版更新（含主从门控、
  更大的超时、更稳的状态机），`nodes/` 版更旧；
- `pkg/mid360_locate` 同时保留 ROS1（`locate_node.cpp`/`main.cpp`）与 ROS2
  （`locate_node_ros2.cpp`/`main_ros2.cpp`），CMakeLists **实际编译 ROS2 版**。

### 7.3 构建与部署（参考 `docs/QUICK_START.md`）

```bash
# 1) vendor 依赖包（pkg/ 目录，单一部署单元）
colcon build --base-paths src/navigation_controller/pkg --symlink-install
source install/setup.bash

# 2) 共享接口包（必须先于导航控制器）
colcon build --packages-select robot_interfaces --symlink-install
source install/setup.bash

# 3) 导航控制器
colcon build --packages-select navigation_controller --symlink-install

# 4) 启动
ros2 launch navigation_controller fusion_ros2.launch.py
```

MCB 端部署：将 `robot_interfaces` 与 `navigation_controller` 打包传输后按同顺序构建即可，
MCB 与导航端共用同一 `robot_interfaces` 接口定义。

### 7.4 参数配置体系

- `config/config_env.yaml`：环境级参数（网络接口、地图路径、速度上限、容差、硬件/仿真开关）；
- `config/ekf_localization.yaml`：EKF 融合参数（50 Hz、LiDAR+IMU 权重、过程噪声）；
- `launch/fusion_ros2.launch.py`：节点级参数（DWA 速度/检测、定位参数、TF 等）。

> 注意事项：launch 中为 locate_node 传入的定位鲁棒性参数（`max_rotation_error`、
> `min_confidence`、`ndt_resolution`、`enable_motion_compensation` 等）在 ROS2 版
> `main_ros2.cpp` 中**并未读取**（仅读取 map_file_path / points_type / topic_name /
> map_roll / map_pitch / map_yaw 6 项），实际生效的置信度阈值、跳变上限与 NDT 参数均为
> **源码硬编码**（如 CONFIDENCE_THRESHOLD=0.15、MAX_POSITION_JUMP=4.0 m、
> MAX_ROTATION_JUMP=2.0 rad、NDT res=1.0）。调参时请以源码常量为准。

---

## 8. 快速开始

```bash
# 仿真（无硬件）：机器人显示在原点，用 RViz 2D Pose Estimate 设初始位姿
ros2 launch navigation_controller fusion_ros2.launch.py \
    use_hardware:=false use_livox:=false use_localization:=false

# 硬件模式（G1 + MID360 + 定位）
ros2 launch navigation_controller fusion_ros2.launch.py \
    use_hardware:=true hardware_interface:=<网卡名>

# 无 RViz（headless）
ros2 launch navigation_controller fusion_ros2.launch.py use_rviz:=false

# 自定义地图
ros2 launch navigation_controller fusion_ros2.launch.py \
    map_file:=/path/map.yaml pcd_file:=/path/map.pcd
```

操作流程：启动 → RViz 设置 2D Pose Estimate（locate_node 等待该消息才进入定位循环）
→ 2D Nav Goal 下发目标 → A\* 规划 → DWA 避障 → 位置控制执行。
