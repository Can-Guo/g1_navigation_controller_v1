#!/bin/bash
# ==================================================================
# Navigation Controller 快速参考 - MCB部署版本
# ==================================================================

【构建命令】
  # 手动构建 (分三阶段)
  # 1. 构建依赖包
  colcon build --base-paths src/navigation_controller/pkg --symlink-install
  source install/setup.bash
  
  # 2. 构建接口包 (必须先构建)
  colcon build --packages-select robot_interfaces --symlink-install
  source install/setup.bash
  
  # 3. 构建导航控制器
  colcon build --packages-select navigation_controller --symlink-install

【运行】
  source install/setup.bash
  ros2 launch navigation_controller fusion_ros2.launch.py

【MCB部署】
  # 打包 (包含接口定义)
  cd src
  tar -czf robot_interfaces.tar.gz robot_interfaces/
  tar -czf navigation_controller_mcb.tar.gz navigation_controller/

  # MCB端解压构建
  tar -xzf robot_interfaces.tar.gz
  tar -xzf navigation_controller_mcb.tar.gz
  cd ~/ros2_ws_nav
  colcon build --packages-select robot_interfaces --symlink-install
  source install/setup.bash
  colcon build --packages-select navigation_controller --symlink-install

【验证】
  # 包列表
  ros2 pkg list | grep -E "(fast_gicp|livox|mid360|robot_interfaces|navigation_controller)"

  # 接口验证
  ros2 interface list | grep robot_interfaces
  ros2 interface show robot_interfaces/action/PointToPoint

  # 路径检查
  ros2 launch navigation_controller fusion_ros2.launch.py --show-args

  # 节点测试
  ros2 run navigation_controller point_to_point_action_server.py

【关键点】
  ✓ pkg/包含所有依赖,单一部署单元
  ✓ 必须分阶段构建 (pkg/ → robot_interfaces → navigation_controller)
  ✓ robot_interfaces包含共享接口定义,MCB和SLAM端共用
  ✓ Launch文件自动适配,无需修改
  ✓ 保持向后兼容,现有工作流不变

【故障排查】
  问题: colcon找不到pkg/包
  解决: colcon build --base-paths src/navigation_controller/pkg

  问题: ModuleNotFoundError: No module named 'robot_interfaces'
  解决: 先构建robot_interfaces包
       colcon build --packages-select robot_interfaces --symlink-install
       source install/setup.bash

  问题: 依赖包找不到
  解决: source install/setup.bash 后再构建navigation_controller

  问题: Launch报错找不到包
  解决: source install/setup.bash

