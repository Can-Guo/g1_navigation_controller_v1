#include "locate_node.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("locate_node");
    g_node = node;
    
    // 声明参数
    node->declare_parameter<std::string>("map_file_path", "");
    node->declare_parameter<int>("points_type", 0);
    node->declare_parameter<std::string>("topic_name", "/livox/lidar");
    node->declare_parameter<float>("map_roll", 0.0f);
    node->declare_parameter<float>("map_pitch", 0.0f);
    node->declare_parameter<float>("map_yaw", 0.0f);

    // =========================================================================
    // Deskew parameters (rotation-only)
    // =========================================================================
    node->declare_parameter<bool>("enable_deskew", true);
    node->declare_parameter<std::string>("imu_topic", "/livox/imu");
    node->declare_parameter<double>("imu_buffer_sec", 5.0);
    node->declare_parameter<int64_t>("imu_time_offset_ns", 0);
    // 0=start, 1=mid, 2=end
    node->declare_parameter<int>("deskew_reference_mode", 0);

    // =========================================================================
    // Registration acceptance parameters
    // =========================================================================
    // 默认从 0.20 提升到 0.30，降低低质量配准通过率，优先保证定位稳定性。
    node->declare_parameter<double>("confidence_threshold", 0.30);
    node->declare_parameter<double>("max_position_jump_m", 4.0);
    node->declare_parameter<double>("max_rotation_jump_rad", 2.0);
    node->declare_parameter<int>("reject_check_warmup_frames", 5);

    // =========================================================================
    // Startup initialization / runtime re-localization behavior
    // =========================================================================
    node->declare_parameter<bool>("auto_initialize_pose", true);
    node->declare_parameter<bool>("allow_runtime_relocalization", true);
    node->declare_parameter<double>("auto_init_x", 0.0);
    node->declare_parameter<double>("auto_init_y", 0.0);
    node->declare_parameter<double>("auto_init_z", 1.3);
    node->declare_parameter<double>("auto_init_yaw", 0.0);
    node->declare_parameter<int>("localization_ready_streak_needed", 3);
    
    // 获取参数（使用默认值初始化，避免未初始化警告）
    std::string map_file_path = "";
    int points_type = 0;
    std::string topic_name = "/livox/lidar";
    float map_yaw = 0.0f;
    float map_roll = 0.0f;
    float map_pitch = 0.0f;
    
    node->get_parameter("map_file_path", map_file_path);
    node->get_parameter("points_type", points_type);
    node->get_parameter("topic_name", topic_name);
    node->get_parameter("map_roll", map_roll);
    node->get_parameter("map_pitch", map_pitch);
    node->get_parameter("map_yaw", map_yaw);

    // Read deskew params into global config (used by locate_node_ros2.cpp)
    node->get_parameter("enable_deskew", g_enable_deskew);
    node->get_parameter("imu_topic", g_imu_topic);
    node->get_parameter("imu_buffer_sec", g_imu_buffer_sec);
    node->get_parameter("imu_time_offset_ns", g_imu_time_offset_ns);
    node->get_parameter("deskew_reference_mode", g_deskew_reference_mode);

    // Read registration thresholds into global config (used by locate_node_ros2.cpp)
    {
      double v = 0.0;
      node->get_parameter("confidence_threshold", v);
      g_confidence_threshold = static_cast<float>(v);
      node->get_parameter("max_position_jump_m", v);
      g_max_position_jump_m = static_cast<float>(v);
      node->get_parameter("max_rotation_jump_rad", v);
      g_max_rotation_jump_rad = static_cast<float>(v);
      node->get_parameter("reject_check_warmup_frames", g_reject_check_warmup_frames);
    }

    // Read startup/relocalization behavior parameters.
    node->get_parameter("auto_initialize_pose", g_auto_initialize_pose);
    node->get_parameter("allow_runtime_relocalization", g_allow_runtime_relocalization);
    node->get_parameter("auto_init_x", g_auto_init_x);
    node->get_parameter("auto_init_y", g_auto_init_y);
    node->get_parameter("auto_init_z", g_auto_init_z);
    node->get_parameter("auto_init_yaw", g_auto_init_yaw);
    node->get_parameter("localization_ready_streak_needed", g_localization_ready_streak_needed);
    
    RCLCPP_INFO(node->get_logger(), "Map file path: %s", map_file_path.c_str());
    RCLCPP_INFO(node->get_logger(), "Points type: %d (0: livox, 1: pointcloud2)", points_type);
    RCLCPP_INFO(node->get_logger(), "Topic name: %s", topic_name.c_str());

    RCLCPP_INFO(node->get_logger(),
      "Deskew: enable=%s, imu_topic=%s, imu_buffer_sec=%.1f, imu_time_offset_ns=%ld, ref_mode=%d",
      g_enable_deskew ? "true" : "false", g_imu_topic.c_str(), g_imu_buffer_sec,
      static_cast<long>(g_imu_time_offset_ns), g_deskew_reference_mode);

    RCLCPP_INFO(node->get_logger(),
      "Registration thresholds: confidence>=%.2f, pos_jump<=%.2fm, rot_jump<=%.2frad, warmup=%d frames",
      g_confidence_threshold, g_max_position_jump_m, g_max_rotation_jump_rad, g_reject_check_warmup_frames);

    RCLCPP_INFO(node->get_logger(),
      "Initialization: auto_initialize_pose=%s, allow_runtime_relocalization=%s, "
      "auto_init=(%.2f, %.2f, %.2f, yaw=%.2f), ready_streak=%d",
      g_auto_initialize_pose ? "true" : "false",
      g_allow_runtime_relocalization ? "true" : "false",
      g_auto_init_x, g_auto_init_y, g_auto_init_z, g_auto_init_yaw,
      g_localization_ready_streak_needed);
    
    // 配置 QoS: 使用 BEST_EFFORT 以兼容传感器数据的特性
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    // 订阅 initialpose 话题 (使用 RELIABLE QoS)
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    auto initial_pose_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", reliable_qos, initialPoseCallback);
    RCLCPP_INFO(node->get_logger(), "Waiting for initial pose from RViz...");

    // Subscribe IMU for deskew. Use BEST_EFFORT to match typical sensor QoS.
    // Note: Even if deskew is disabled, we still allow subscribing; it costs little.
    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        g_imu_topic, sensor_qos, imu_callback);
    
    // 初始化地图
    initializeMap(points_map, map_file_path, map_yaw, map_pitch, map_roll);
    
    // 将pcl点云转换为ROS消息
    pcl::toROSMsg(*points_map, cloud_msg);
    cloud_msg.header.stamp = node->now();
    cloud_msg.header.frame_id = "map";
    
    // 创建订阅者 (使用 BEST_EFFORT QoS 匹配 livox 驱动)
    rclcpp::SubscriptionBase::SharedPtr points_sub;
    if (points_type == 0) {
        auto livox_sub = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            topic_name, sensor_qos, livox_callback);
        points_sub = livox_sub;
    } else {
        auto pcl_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name, sensor_qos, pcl_callback);
        points_sub = pcl_sub;
    }
    
    // 创建发布者 (使用 BEST_EFFORT QoS 以提高性能)
    points_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned", sensor_qos);
    pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/veh_pose", reliable_qos);
    path_pub = node->create_publisher<nav_msgs::msg::Path>("/path", reliable_qos);
    
    // 新增：配准置信度发布者（对G1人形机器人监控定位质量很重要）
    confidence_pub = node->create_publisher<std_msgs::msg::Float32>("/localization_confidence", reliable_qos);
    localization_ready_pub = node->create_publisher<std_msgs::msg::Bool>("/localization_ready", reliable_qos);
    RCLCPP_INFO(node->get_logger(), "配准置信度将发布到 /localization_confidence 话题");
    
    // 创建TF广播器
    auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    
    // 创建odometry发布器 (使用 RELIABLE QoS,因为里程计数据很重要)
    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/odometry/map", reliable_qos);
    
    // 创建地图发布器 (使用 BEST_EFFORT QoS,地图是大数据)
    auto map_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/points_map", sensor_qos);
    
    // 创建地图发布线程
    std::thread publisher_thread(pointCloudPublisherThread, map_pub);
    publisher_thread.detach();
    
    // 初始化策略：
    // - auto_initialize_pose=true (默认): 启动后直接使用配置的 map 原点（或自定义初始位姿）
    //   开始定位，不再强制等待 RViz 手动点击 initial pose。
    // - auto_initialize_pose=false: 维持旧行为，必须等待 RViz /initialpose。
    if (g_auto_initialize_pose) {
        veh_pose = Eigen::Matrix4f::Identity();
        veh_pose(0, 3) = static_cast<float>(g_auto_init_x);
        veh_pose(1, 3) = static_cast<float>(g_auto_init_y);
        veh_pose(2, 3) = static_cast<float>(g_auto_init_z);
        veh_pose.topLeftCorner<3, 3>() =
            Eigen::AngleAxisf(static_cast<float>(g_auto_init_yaw), Eigen::Vector3f::UnitZ()).toRotationMatrix();

        prev_veh_pose = veh_pose;
        received_initial_pose = true;
        inited = false;  // force robust NDT-style re-init from configured pose
        g_localization_ready = false;

        RCLCPP_INFO(node->get_logger(),
            "Auto-initialized pose: x=%.3f y=%.3f z=%.3f yaw=%.3f rad. "
            "You can still manually re-localize via RViz /initialpose.",
            g_auto_init_x, g_auto_init_y, g_auto_init_z, g_auto_init_yaw);
    } else {
        rclcpp::Rate wait_rate(10);
        while (rclcpp::ok() && !received_initial_pose) {
            rclcpp::spin_some(node);
            wait_rate.sleep();
        }
    }
    
    RCLCPP_INFO(node->get_logger(), "Initial pose received, starting localization...");
    
    // 主循环
    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
        // 发布从map到base_link的TF变换
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = node->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";
        
        // 使用实际位姿作为变换
        transform.transform.translation.x = veh_pose(0, 3);
        transform.transform.translation.y = veh_pose(1, 3);
        transform.transform.translation.z = veh_pose(2, 3);
        
        // 将旋转矩阵转换为四元数
        Eigen::Quaterniond rotation_quat(veh_pose.block<3, 3>(0, 0).cast<double>());
        transform.transform.rotation = tf2::toMsg(rotation_quat);
        
        // 发布TF变换
        tf_broadcaster->sendTransform(transform);
        
        // 发布odometry消息
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = node->now();
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";
        
        // 设置位置
        odom_msg.pose.pose.position.x = veh_pose(0, 3);
        odom_msg.pose.pose.position.y = veh_pose(1, 3);
        odom_msg.pose.pose.position.z = veh_pose(2, 3);
        
        // 设置方向
        odom_msg.pose.pose.orientation = tf2::toMsg(rotation_quat);
        
        // 发布odometry消息
        odom_pub->publish(odom_msg);
        
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}
