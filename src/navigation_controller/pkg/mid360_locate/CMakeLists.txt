cmake_minimum_required(VERSION 3.8)
project(mid360_locate)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(fast_gicp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${fast_gicp_INCLUDE_DIRS}
)

# Add executable
add_executable(locate_node 
  src/locate_node_ros2.cpp
  src/main_ros2.cpp
)

# Dependencies for the executable
ament_target_dependencies(locate_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_ros
  pcl_conversions
  livox_ros_driver2
  ament_index_cpp
  fast_gicp
)

# Link libraries
target_link_libraries(locate_node
  ${PCL_LIBRARIES}
  ${fast_gicp_LIBRARIES}
)

# Install targets
install(TARGETS locate_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz files
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install other directories if needed
install(DIRECTORY script tum
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
