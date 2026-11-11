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
    
    RCLCPP_INFO(node->get_logger(), "Map file path: %s", map_file_path.c_str());
    RCLCPP_INFO(node->get_logger(), "Points type: %d (0: livox, 1: pointcloud2)", points_type);
    RCLCPP_INFO(node->get_logger(), "Topic name: %s", topic_name.c_str());
    
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
    
    // 等待直到收到初始位姿
    rclcpp::Rate wait_rate(10);
    while (rclcpp::ok() && !received_initial_pose) {
        rclcpp::spin_some(node);
        wait_rate.sleep();
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
