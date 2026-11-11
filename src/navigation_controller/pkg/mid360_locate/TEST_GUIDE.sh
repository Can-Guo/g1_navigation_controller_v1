#!/bin/bash

# mid360_locate ROS2 包测试指南
# ================================

echo "mid360_locate ROS2 包测试指南"
echo "=============================="
echo ""

# 检查必要的文件
echo "1. 检查关键文件是否存在..."
echo ""

if [ -f "/home/york/ros2_ws_nav/install/mid360_locate/lib/mid360_locate/locate_node" ]; then
    echo "✓ locate_node 可执行文件存在"
else
    echo "✗ locate_node 可执行文件不存在"
fi

if [ -f "/home/york/ros2_ws_nav/install/fast_gicp/lib/libfast_gicp.so" ]; then
    echo "✓ fast_gicp 库文件存在"
else
    echo "✗ fast_gicp 库文件不存在"
fi

if [ -f "/home/york/ros2_ws_nav/src/g1_nav/map/map.pcd" ]; then
    echo "✓ 地图文件 map.pcd 存在"
    MAP_SIZE=$(du -h /home/york/ros2_ws_nav/src/g1_nav/map/map.pcd | cut -f1)
    echo "  地图文件大小: $MAP_SIZE"
else
    echo "✗ 地图文件 map.pcd 不存在"
fi

echo ""
echo "2. 验证依赖关系..."
echo ""

# Source 环境
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/york/ros2_ws_nav/install/setup.bash 2>/dev/null

# 检查包是否可用
if ros2 pkg list | grep -q "mid360_locate"; then
    echo "✓ mid360_locate 包已注册"
else
    echo "✗ mid360_locate 包未注册"
fi

if ros2 pkg list | grep -q "fast_gicp"; then
    echo "✓ fast_gicp 包已注册"
else
    echo "✗ fast_gicp 包未注册"
fi

if ros2 pkg list | grep -q "livox_ros_driver2"; then
    echo "✓ livox_ros_driver2 包已注册"
else
    echo "✗ livox_ros_driver2 包未注册"
fi

echo ""
echo "3. 测试步骤建议:"
echo ""
echo "步骤 1: 启动 Livox 驱动 (在终端 1)"
echo "  $ source /home/york/ros2_ws_nav/install/setup.bash"
echo "  $ ros2 launch livox_ros_driver2 msg_MID360_launch.py"
echo ""
echo "步骤 2: 启动定位节点 (在终端 2)"
echo "  $ source /home/york/ros2_ws_nav/install/setup.bash"
echo "  $ ros2 launch mid360_locate localization.launch.py"
echo ""
echo "步骤 3: 启动 RViz2 (在终端 3)"
echo "  $ source /home/york/ros2_ws_nav/install/setup.bash"
echo "  $ rviz2 -d /home/york/ros2_ws_nav/src/g1_nav/rviz/visualization.rviz"
echo ""
echo "步骤 4: 在 RViz2 中设置初始位姿"
echo "  - 使用 '2D Pose Estimate' 工具在地图上点击设置初始位置"
echo "  - 观察 /veh_pose 话题是否有数据发布"
echo "  - 检查 TF 树中是否有 map → base_link 的变换"
echo ""
echo "4. 调试命令:"
echo ""
echo "查看所有话题:"
echo "  $ ros2 topic list"
echo ""
echo "查看激光雷达数据:"
echo "  $ ros2 topic hz /livox/lidar"
echo "  $ ros2 topic echo /livox/lidar --no-arr"
echo ""
echo "查看定位输出:"
echo "  $ ros2 topic echo /veh_pose"
echo "  $ ros2 topic echo /path"
echo ""
echo "查看 TF 树:"
echo "  $ ros2 run tf2_tools view_frames"
echo "  $ ros2 run tf2_ros tf2_echo map base_link"
echo ""
echo "查看节点信息:"
echo "  $ ros2 node list"
echo "  $ ros2 node info /locate_node"
echo ""
echo "5. 预期结果:"
echo ""
echo "成功的标志:"
echo "  ✓ RViz2 中 Fixed Frame [map] 错误消失"
echo "  ✓ 可以看到激光雷达点云数据"
echo "  ✓ /veh_pose 话题有持续的位姿输出"
echo "  ✓ /path 话题显示机器人轨迹"
echo "  ✓ TF 树包含完整的 map → base_link → ... → lidar_frame 链"
echo "  ✓ 对齐的点云 (/aligned) 与地图匹配"
echo ""
echo "6. 常见问题排查:"
echo ""
echo "问题: 没有激光雷达数据"
echo "解决: 检查 Livox 驱动是否正常运行,设备是否连接"
echo ""
echo "问题: map frame 仍然不存在"
echo "解决: 确保在 RViz2 中设置了初始位姿 (使用 2D Pose Estimate)"
echo ""
echo "问题: 定位不准确"
echo "解决: 调整 map_roll, map_pitch, map_yaw 参数"
echo "      检查初始位姿是否设置正确"
echo ""
echo "=============================="
echo "准备好开始测试了!"
echo "=============================="
