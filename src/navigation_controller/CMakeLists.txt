cmake_minimum_required(VERSION 3.8)
project(navigation_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_localization REQUIRED)
find_package(std_msgs REQUIRED)

# Robot interfaces dependency (action definitions)
find_package(robot_interfaces REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../../unitree_sdk2/thirdparty/include
  ${CMAKE_SOURCE_DIR}/../../unitree_sdk2/thirdparty/include/ddscxx
  ${CMAKE_SOURCE_DIR}/../../unitree_sdk2/thirdparty/include/ddsc
  ${CMAKE_SOURCE_DIR}/../../unitree_sdk2/include
)

# Link directories
link_directories(
  ${CMAKE_SOURCE_DIR}/../../unitree_sdk2/lib/x86_64
  ${CMAKE_SOURCE_DIR}/../../unitree_sdk2/thirdparty/lib/x86_64
)

# Install Python executable scripts from scripts/ directory (traditional approach)
# rosidl_generate_interfaces automatically creates Python package for action interfaces
# Additional nodes are installed as executable scripts for backward compatibility
install(PROGRAMS
  scripts/astar_planner_ros2.py
  scripts/astar_planner_ros2_Manhattan.py
  scripts/dwa_planner_ros2_mid360.py
  scripts/g1_position_control_ros2.py
  scripts/g1_position_control_ros2_manhattan.py
  scripts/point_to_point_action_server.py
  # scripts/test_point_to_point_action.py
  scripts/test_point_to_point_action_keyboard.py
  # scripts/nav_goal_pub_key_demo2.py
  scripts/pcd2pgm.py
  scripts/simulation_odometry_publisher.py
  # scripts/demo_multi_point_patrol.py  # 文件缺失
  # scripts/localization_monitor.py  # 文件缺失
  # scripts/monitor_ekf_fusion.py  # 文件缺失
  # scripts/tf_tree_monitor.py  # 文件缺失
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python module sources for library-style imports (optional but recommended)
# This allows: from navigation_controller.nodes import astar_planner_ros2
install(DIRECTORY navigation_controller/nodes
  DESTINATION lib/python3.10/site-packages/navigation_controller
  FILES_MATCHING PATTERN "*.py"
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install map files
install(DIRECTORY
  map
  DESTINATION share/${PROJECT_NAME}/
)

# Install path files
install(DIRECTORY
  path
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package configuration
ament_package()

# Output configuration information
message(STATUS "")
message(STATUS "===============================================")
message(STATUS "Configuring ${PROJECT_NAME} for ROS2 Humble")
message(STATUS "===============================================")
message(STATUS "CMake version: ${CMAKE_VERSION}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Unitree SDK: ${CMAKE_SOURCE_DIR}/../../unitree_sdk2")
message(STATUS "===============================================")
message(STATUS "")
