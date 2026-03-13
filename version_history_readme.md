# 项目版本历史与差异说明（最新）

## 1. Git 基线与当前状态
- 当前仓库根目录：`/home/york/ros2_ws_nav`
- 当前可见提交基线（HEAD）：
  - `bf68a46` `20260313-1 first commit for navigation`
- 当前相对该提交的变更状态（已暂存）：
  - 修改 `src/navigation_controller/launch/fusion_ros2.launch.py`
  - 修改 `src/navigation_controller/pkg/mid360_locate/src/locate_node.h`
  - 修改 `src/navigation_controller/pkg/mid360_locate/src/locate_node_ros2.cpp`
  - 修改 `src/navigation_controller/pkg/mid360_locate/src/main_ros2.cpp`
- 变更规模统计（`git diff --cached --stat`）：
  - 4 files changed, 222 insertions(+), 24 deletions(-)

## 2. 基于提交差异的技术解读（HEAD vs 当前改动）

### 2.1 启动层与安全语义（`fusion_ros2.launch.py`）
- 新增定位初始化参数透传到 `mid360_locate`（自动初始化、手动重定位、ready streak 等）。
- 启动日志从“静态文本判断”改为“基于运行参数动态显示”，避免误导。
- 强化安全门控语义：明确硬件控制默认关闭，只有显式 `use_hardware:=true` 才启用。

**专业背景说明**
- 在机器人系统里，启动日志不仅是“打印信息”，也是运维与现场安全的第一道防线。
- 把模式判定绑定到 `LaunchConfiguration` 可以避免“配置改了、日志还在说旧状态”的人因风险。

### 2.2 定位接口层（`locate_node.h`）
- 增加 `localization_ready` 发布器接口与相关全局状态。
- 增加自动初始化/运行时重定位相关参数声明。

**专业背景说明**
- 人形机器人场景里，定位通常需要一个“可操作状态机”：`未就绪 -> 就绪 -> 失锁 -> 重定位`。
- 单纯发布位姿不够，发布 `ready` 状态有助于上层任务器做安全决策（如禁止发目标点）。

### 2.3 定位核心逻辑（`locate_node_ros2.cpp`）
- 增加 `localization_ready` 话题发布与连续高置信度判定（streak）。
- 抽象 `applyInitialPose()`，实现：
  - 启动初始化位姿应用
  - 运行时 RViz `/initialpose` 重定位
  - 重定位时强制回到稳健初始化路径（重置配准阶段）
- 当配准被拒绝时，清空 streak 并把 `localization_ready` 置为 false。

**专业背景说明**
- 这是从“单点算法”向“可运行系统”升级的关键步骤：
  - 算法层：NDT/GICP 给出置信度
  - 系统层：把置信度转成可消费的状态信号，驱动任务调度和安全停机逻辑
- 对 G1 这类动态平台，定位结果有时“可计算但不可用”，ready gating 能减少误用位姿带来的行为风险。

### 2.4 启动流程行为（`main_ros2.cpp`）
- 新增参数读取与日志打印（初始化策略、重定位策略、阈值策略）。
- 启动阶段支持自动初始化（默认 true），不再强制等待 RViz 首次手动位姿。
- 同时保留并允许运行时手动重定位。

**专业背景说明**
- 这类改动提升的是“系统可用性”：
  - 默认自动启动减少人工步骤
  - 手动重定位保留现场纠偏能力
- 两者并存是工程上更稳妥的方案，尤其适合真实部署与调试混用阶段。

## 3. 当前系统定位（SLAM 视角）
- 当前形态属于：**基于先验地图的定位 + 导航**（Localization + Navigation）。
- 尚不属于完整在线 SLAM（缺少在线建图、回环检测、图优化后端）。
- 现有改动重点是提升“定位可用性与安全可控性”，不是引入完整 SLAM 后端。

## 4. 运行约束与现象解释（结合近期日志）
- 若 Livox 初始化失败（例如 `bind failed`）：
  - 无有效 `/livox/lidar`（常伴随 IMU链路不完整）
  - DWA 会持续 `Waiting for point cloud data...`
  - 即使 A* 与 map_server 正常，局部避障也无法工作
- 这不是 DWA 算法本身故障，而是上游感知链路缺数据。

## 5. 推荐启动基线
- 安全默认（不启用硬件 SDK）：
  - `ros2 launch navigation_controller fusion_ros2.launch.py`
- 显式启用硬件控制（现场作业前确认环境安全）：
  - `ros2 launch navigation_controller fusion_ros2.launch.py use_hardware:=true`
- 无硬件/无传感器纯仿真联调：
  - `ros2 launch navigation_controller fusion_ros2.launch.py use_hardware:=false use_livox:=false use_localization:=false`

## 6. 后续建议（提交治理）
- 当前仓库仅有 1 个基线提交，建议将本轮 4 个文件变更拆分为 2~3 个逻辑提交：
  - 启动安全语义与日志
  - 定位状态机（auto init + re-localization + readiness）
  - （可选）参数/注释整理
- 这样后续回溯、回滚与评审会更清晰。
