# 0. Navigation Control: PointToPoint.action 


# 1. PointToPoint.action (Goal部分)

# ========== 任务标识 ==========
string task_id                # MCB任务ID，用于日志追踪

# ========== 核心目标 ==========, 理由：复用ROS2标准消息，与现有/goal_pose话题兼容
geometry_msgs/PoseStamped target_pose



# 2. PointToPoint.action (Feedback部分)

# ========== 当前状态（从/odometry/filtered获取）==========
geometry_msgs/PoseStamped current_pose
geometry_msgs/Twist current_velocity

# ========== 导航状态（监控三个节点）==========
uint8 navigation_state 导航状态 

uint8 STATE_PLANNING = 0        # A*规划中
uint8 STATE_FOLLOWING = 1       # 正常跟踪路径
uint8 STATE_AVOIDING = 2        # DWA避障激活
uint8 STATE_RECOVERING = 3      # 恢复行为中（改进2的逃逸模式）
uint8 STATE_STUCK = 4           # 可能卡住

# ========== 障碍物信息（从DWA获取）==========
uint8 obstacle_proximity

uint8 OBSTACLE_CLEAR = 0        # >1.5m
uint8 OBSTACLE_NEAR = 1         # 0.5-1.5m
uint8 OBSTACLE_CRITICAL = 2     # < 0.5m



# 3. PointToPoint.action (Result部分)

# ========== 执行结果 ==========
bool success

uint8 status_code # status_code 状态码（保持不变，但聚焦实际场景）

uint8 STATUS_SUCCESS = 0
uint8 STATUS_TIMEOUT = 1
uint8 STATUS_LOCALIZATION_LOST = 2      # confidence持续<0.3
uint8 STATUS_PATH_BLOCKED = 3           # A*无法规划或DWA长期卡住
uint8 STATUS_GOAL_UNREACHABLE = 4       # 目标点在障碍物内
uint8 STATUS_PREEMPTED = 5              # 新目标抢占
uint8 STATUS_EMERGENCY_STOP = 6         # 外部停止信号

# ========== 最终状态 ==========
geometry_msgs/PoseStamped final_pose    # 实际到达位姿
float32 position_error                   # 与目标的欧几里得距离



