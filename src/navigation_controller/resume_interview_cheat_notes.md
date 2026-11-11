# 简历 & 面试 Cheat Notes — navigation_controller（G1 导航）

> 用法：Part 1 直接粘贴进简历；Part 2 是面试口头应答，每问按「思路 → 答案」组织——先想清回答的角度，再说出完整的话。
> 原则：只讲你真正做过、能讲透的点；凡是承认取舍的地方，都比你嘴硬辩解加分。

---

## Part 1：社招简历条目（≤3 点，共 4 行内）

### 中文版

> **G1 人形机器人 LiDAR 自主导航系统（ROS 2）｜项目负责人**
> - 从零搭建基于 Livox MID360、ROS 2（C++/Python）的定位、规划、控制一体导航系统，核心算法为 NDT/GICP 点云配准、A* 全局规划与 DWA 局部避障，以单包加 vendor 依赖交付，主控板侧一键部署。
> - 设计 NDT 初始化与 FastGICP 跟踪的定位方案，加入置信度门控与 EKF 惯性融合，解决人形行走晃动导致的定位漂移，多楼层、多场景长期稳定运行。
> - 主导定义主控板与导航端基于 ROS 2 Action 的任务协议，覆盖任务抢占、失败诊断与心跳门控安全机制，保障导航任务执行安全可靠。
> - 量化指标（实测）：定位频率 ___ Hz，到达误差 ___ m，任务成功率 ___ %，覆盖场景 ___ 个。

（量化行只填实测数字；暂无数值可先删除本行。）

### English Version

> **G1 Humanoid Robot LiDAR Autonomous Navigation System (ROS 2) | Project Lead**
> - Built from scratch a localization-planning-control navigation stack for the G1 humanoid on a Livox MID360 LiDAR, powered by ROS 2 (C++/Python) with core algorithms of NDT/GICP scan registration, A* global planning, and DWA local avoidance, shipped as a single package with vendored dependencies for one-command deployment on the main control board.
> - Designed NDT-initialized, FastGICP-tracked LiDAR localization with confidence gating and EKF-based IMU fusion, resolving pose drift from humanoid walking and keeping the system stable across multiple floors and scenarios in long-term operation.
> - Led the ROS 2 Action-based task protocol between the main control board and the navigation stack, covering goal preemption, failure diagnosis, and a heartbeat gating mechanism for safe and reliable task execution.
> - Measured metrics: localization rate ___ Hz, arrival error ___ m, task success rate ___ %, deployed across ___ scenarios.

(Only fill in measured values; remove this line if no data yet.)

---

---

## Part 2：技术面试官问答（问题 + 思路 + 答案）

> 每问先给「思路」（回答前想清的角度），再给「答案」（可直接说出口的完整回答）。
> 答案偏长，面试时按语感压缩即可，但务必保持「先给结论、再讲依据、最后承认局限」的顺序。

### 必考题 1｜为什么不用 Nav2，要自研？

**思路**：先承认 Nav2 是标准成熟方案，避免显得傲慢；再给三个落到本项目特定约束上的自研理由；最后补一句换成通用平台会用 Nav2，展示技术判断的客观性。

**答案**：

Nav2 确实是业界标准的导航框架，我并不是否定它，而是这个项目有几项特定的需求。第一，G1 人形机器人只能沿单轴移动，每次只走 X 或 Y 方向，而 Nav2 默认的局部控制器面向差速或全向底盘，我们自研的 A 星规划器用曼哈顿距离做启发式，并把路径做了轴对齐，能精确贴合它的运动学。第二，我们的定位是低频的激光点云配准，只有两三赫兹，靠 EKF 融合提高到五十赫兹，整个控制栈和定位层耦合得很深，自研可以让我们对每一层的表现都有完全的控制。第三，从部署角度，我们希望整个系统是一个单包、内部带齐依赖的交付单元，主控板侧一条命令就能构建，引入完整的 Nav2 会明显增加部署和调参的成本。当然我也承认，Nav2 在代价地图、行为树这些工程规范上更成熟，如果将来换成通用的移动底盘，我会优先考虑 Nav2。

### 必考题 2｜定位方案的先进性与局限？（对比在线 SLAM）

**思路**：先澄清概念，我做的是定位而不是在线建图；把亮点锚定在鲁棒性工程而非算法新颖；主动说出局限，再给演进方向。切忌吹嘘算法前沿。

**答案**：

首先澄清一下，这个系统做的是基于先验地图的定位，而不是在线建图，这也是工业量产的主流路线，特点是计算量、内存和行为都可预测，适合产品落地。它的先进性不在提出新算法，而在把这些成熟算法做稳：我设计了置信度评估，把配准适应度、内点率和相对上一帧的位姿变化加权成一个综合分数，再配合位置和旋转跳变门控，被判定为坏帧的结果直接丢弃、保持上一帧位姿，这样机器人行走晃动造成的误匹配不会进入控制环，定位绝不会瞬间跳变。它的局限我也很清楚：地图是静态的，不在线更新；没有 IMU 紧耦合的退化感知，环境发生大改时定位可能退化。如果要演进，我会接入 FAST-LIO2 这类在线里程计做局部地图更新，再加全局重定位处理失效恢复。

### 必考题 3｜DWA 是不是过时了？为什么不换 MPPI？

**思路**：承认 DWA 是经典算法，但强调它仍在量产使用、核心思想未过时；对比 MPPI 时给客观取舍（能力与成本的权衡）；说明本项目选择它是因为低速场景够用。

**答案**：

DWA 是 1997 年提出的经典算法，但它的核心思想，也就是动态窗口、轨迹采样、代价评估，至今仍被大量产品使用，并不算过时。而且我们在实现上做了改进，用连续惩罚函数替代传统的硬约束阶梯函数，机器人接近障碍物时会平滑减速而不是急刹硬拐。MPPI 是现在更主流的采样式控制器，轨迹更平滑、动态避障能力更强，但代价是算力更高、参数更多、调参周期更长。我们这台机器的限速只有每秒零点二米，DWA 每周期采样一百条轨迹已经足够覆盖可行解，所以在当前场景下选择 DWA 是更划算的取舍；如果将来需要更激进或更顺滑的避障，平滑升级到 Nav2 内置的 MPPI 是很自然的路径。

### 必考题 4｜如果重构或改进这个系统，怎么规划？

**思路**：给出有优先级的演进路线，体现自己清楚系统的短板；每个阶段都有明确目的，避免空泛地说"会优化"。

**答案**：

我会按优先级分四步。第一步先补定位闭环，现在的定位丢失只能停车告警，我会接入在线里程计加全局重定位，让系统能够自恢复；第二步是多模态感知，加相机和视觉、做语义过滤，提升动态环境下的适应性；第三步是控制和框架的现代化，把局部控制器从 DWA 升级到 MPPI，并逐步对齐 Nav2 的分层代价地图和行为树规范；第四步是工程规范化，把源码里硬编码的参数全部参数化、收敛当前存在的两套重复代码、补上测试和持续集成。每一步都解决一个明确的短板，而不是推翻重来。

### 加分题 1｜定位丢失怎么处理？

**思路**：按先不跳、再停、后告警三层讲，每层对应代码里的一个机制；最后承认没有自动恢复，并给出方向。

**答案**：

我对定位丢失做了三层防护。第一层在配准层，置信度和跳变门控会拒绝坏帧，被拒的帧保持上一帧位姿，所以定位不会瞬间跳变，控制环不会被扰动。第二层在任务层，Action 服务器监控置信度，持续低于阈值超过五秒就中止任务并返回定位丢失的状态码，由主控板决定重试、重新初始化还是告警。第三层是停车兜底，一旦判定丢失机器人立即停止。它的局限是目前只能停车告警，还不能自动重新定位，我的改进方向是加基于 NDT 全局搜索的全局重定位，让系统自己恢复。

### 加分题 2｜换到室外或大动态环境怎么办？

**思路**：先明确这个系统的设计边界是室内结构化场景；坦诚说出局限（特征稀疏、地图静态）；再给出换场景需要的措施。

**答案**：

这个系统是按室内结构化场景设计的，设计目标是可控、确定、可预测，所以它并不声称是通用的 SLAM 库。局限我很清楚：局部地图的搜索半径最大八十米，室外空旷、特征稀疏的环境下点云配准容易退化；同时地图是静态的，动态物体多的时候会引入干扰。如果换到这种场景，我需要从三个方面补强：一是加强 IMU 和轮速的约束，减少对纯配准的依赖；二是加视觉和语义，过滤掉动态物体再配准；三是引入更强的特征描述子，改善特征稀疏环境下的数据关联。这些改动本质上是把它从室内定位演进成一个多传感器融合的系统。

### 技术先进性表达模板（防止被问"哪里先进"卡住）

**思路**：面试官问哪里先进时，把评判标准从算法新颖拉到在工程约束下把成熟算法做稳，再用三点具体贡献支撑，最后落到可部署、可预测、可调优。

**答案**：

我理解这里的先进不是指提出新算法，而是指在工程约束下把成熟算法做稳。这个系统的真正难点是低频的激光定位加人形行走晃动，我的贡献集中在三点：一是置信度与跳变门控，让坏帧不进入控制环；二是主从心跳门控，让任务生命周期可控、异常自动停车；三是针对单轴运动学的规划适配。相比前沿 SLAM，我更强调可部署、可预测、可调优这三个工程属性。

---

## 附：简历/面试通用提醒

- 所有数字（Hz、m、%、场景数）只写实测值；没有就不写，绝不编。
- 面试被追问细节时，能讲透的点优先讲：NDT 与 GICP 收敛原理、为什么旋转时配准易发散、Action 与普通话题的差异、心跳看门狗如何防服务器掉线后机器人还在移动。
- 「主导」的措辞要能扛住追问：确认是你做的设计决策，再这么写。

---

## Part 3：岗位投递策略与 TODO（2026-08，针对"多传感器融合定位 + RTK + 在线 SLAM"岗位）

### 一、竞争力定位（一句话）
硬缺口在 **RTK 融合** 与 **在线 SLAM 部署**；强项是 **LiDAR 定位鲁棒性工程化、松耦合融合、开源算法工程化**。按「多传感器融合定位 + 开源算法工程化」投递，不要投成「SLAM 专家」。

### 二、简历调整 TODO（投递前）
- [ ] 三点改为：① LiDAR 定位鲁棒性工程化（置信度门控、自适应局部地图）；② LiDAR+IMU 松耦合 EKF 融合，注明"该融合框架可扩展 RTK 作为第三个位置观测源"；③ 开源算法栈的部署与调优能力
- [ ] 投递说明加一句：熟悉 Fast-LIO 系列输出与 LiDAR 定位全链路，能快速将开源 SLAM 工程化
- [ ] 量化补齐实测：定位频率 / 到达误差（务必有真实数字）

### 三、补课 TODO（2 周内，按优先级）
- [ ] **RTK/GNSS 原理**：基站/流动站、载波相位、固定解/浮点解、NTRIP、WGS84/ENU、松耦合 EKF 位置更新；把 RTK 作为第三个观测源接进现有 EKF
- [ ] **Fast-LIO2 源码深读**：ikd-Tree 增量地图、紧耦合 IEKF、前向传播/后向运动补偿；能对照源码讲清它与"先验地图定位"的本质区别
- [ ] **Ceres 最小例子**：代价函数、求解器、加一个因子；会跑通一个最小图优化
- [ ] **李群/李代数基础**：SO(3)/SE(3)、指数/对数映射、左/右扰动（《视觉SLAM十四讲》第 4 章即可）
- [ ] 泛读 LIO-SAM（GTSAM 因子图 + 回环），理解"因子图后端"与"滤波"两条路线

### 四、面试策略 TODO
- [ ] 开场 30 秒定位话术：先讲清"我做过先验地图定位 + 松耦合融合，不是在线 SLAM"，再转成工程化优势（同上文 Part 2）
- [ ] 3 个必讲故事：置信度门控防漂移 / **EKF 把 2-4Hz LiDAR 提到 50Hz（王牌，可自然引出"RTK 也是第三个观测源"）** / 主从心跳协议
- [ ] 3 个必补知识点能讲：RTK 固定解判定、紧耦合 vs 松耦合、Ceres 最小因子图
- [ ] 应对"你没做过在线 SLAM"：承认 + 行动（"已通读 Fast-LIO2 源码，2-4 周可独立部署调通"）

### 五、开源学习资料（落地清单，均已核实链接）
按缺口分组，从"原理 → 代码 → 实践"顺序学：

**① 在线 SLAM 部署（Fast-LIO / LIO-SAM / LOAM）**
- [FAST-LIO2 官方源码（hku-mars/FAST_LIO，含 ikd-Tree）](https://github.com/hku-mars/FAST_LIO)
- [Fast-LIO2 带注释精读版（Fast-lio2-Supernote）](https://github.com/huashu996/Fast-lio2-Supernote)
- [ROS2 版 Fast-LIO（KBKN-Autonomous-Robotics-Lab/FAST_LIO）](https://github.com/KBKN-Autonomous-Robotics-Lab/FAST_LIO)
- [LIO-SAM（TixiaoShan，紧耦合 + GTSAM 因子图，附带示例数据集）](https://github.com/TixiaoShan/LIO-SAM)
- 扩展阅读：A-LOAM / LeGO-LOAM / SC-LIO-SAM / LVI-SAM（在 GitHub 按名搜索即可）

**② RTK / GNSS+IMU 融合**
- [INSTINCT：开源 PNT 框架，含 RTK + INS 松耦合 Kalman + 因子图（GPS Solutions 2025）](https://link.springer.com/article/10.1007/s10291-025-01927-4)
- [Aceinna docs_openrtk：EKF 推导 + GNSS RTK/IMU 松耦合教程（algorithms.rst）](https://raw.githubusercontent.com/Aceinna/docs_openrtk/refs/heads/master/algorithms.rst)
- [《自动驾驶中的 SLAM》开源书（gaoxiang12/slam_in_autonomous_driving）：IMU+RTK+LiDAR 的 ESKF 融合 + NDT](https://deepwiki.com/gaoxiang12/slam_in_autonomous_driving/5.3-fusion-based-localization)
- RTKLIB：RTK 领域标准开源库（GitHub 搜索 "RTKLIB tomojitakasu" 获取官方仓库）

**③ 优化库（Ceres / g2o / GTSAM）**
- [Ceres Solver 官方文档](https://ceres-solver.org/)
- [GTSAM（borglab）](https://github.com/borglab/gtsam)
- g2o：GitHub 搜索 "RainerKuemmerle/g2o"
- 《视觉SLAM十四讲》配套代码（GitHub 搜索 "gaoxiang12/slambook2"），重点读第 4 章（李群李代数/Sophus）、第 6 章（非线性优化与 Ceres/g2o）、第 10 章（后端优化）

**④ 李群 / 李代数 / 状态估计数学**
- [State Estimation for Robotics（Barfoot，免费 PDF，asrl.utias.utoronto.ca）](https://asrl.utias.utoronto.ca)
- "A tutorial on SE(3) transformation parameterizations and on-manifold optimization"（J. Blanco，可搜索获取）
- 《视觉SLAM十四讲》第 4 章（中文，最容易上手）

### 六、学习顺序建议（2 周）
第 1 周：RTK 原理（2 天）+ Fast-LIO2 源码深读（3 天，配注释版）+ 李群入门（周末）。
第 2 周：Ceres 最小例子（1 天）+ LIO-SAM 泛读理解因子图（2 天）+ 每天 30 分钟把"RTK 接进我现有 EKF"的融合框图画熟（面试高频）。
重心永远放在：**能对着白板讲清楚"我怎么做、为什么这么做、换成 RTK/在线 SLAM 怎么扩展"**。
