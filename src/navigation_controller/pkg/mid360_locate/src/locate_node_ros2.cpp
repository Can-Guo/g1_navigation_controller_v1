#include "locate_node.h"

// 全局变量定义
bool inited = false;
Eigen::Matrix4f veh_pose = Eigen::Matrix4f::Identity();
Eigen::Matrix4f prev_veh_pose = Eigen::Matrix4f::Identity();  // 新增：上一次位姿
pcl::PointCloud<pcl::PointXYZ>::Ptr points_map(new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::msg::PointCloud2 cloud_msg;
nav_msgs::msg::Path path_msg;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr confidence_pub;  // 新增：置信度发布者
rclcpp::Time t_msg = rclcpp::Time(0);
bool received_initial_pose = false;
std::shared_ptr<rclcpp::Node> g_node;
std::vector<RegistrationQuality> quality_history;  // 新增：配准质量历史

// ============================================================================
// 配准质量评估函数实现
// 专为G1人形机器人优化，考虑行走时的动态特性
// ============================================================================

float computeInlierRatio(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    const Eigen::Matrix4f& transformation,
    float max_correspondence_distance
) {
    // 变换源点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source, *transformed, transformation);
    
    // 构建KD树用于最近邻搜索
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(target);
    
    int inliers = 0;
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    
    for (const auto& point : transformed->points) {
        if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            if (pointNKNSquaredDistance[0] < max_correspondence_distance * max_correspondence_distance) {
                inliers++;
            }
        }
    }
    
    return static_cast<float>(inliers) / static_cast<float>(source->points.size());
}

RegistrationQuality evaluateRegistrationQuality(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    const Eigen::Matrix4f& transformation,
    float fitness_score,
    int num_iterations
) {
    RegistrationQuality quality;
    
    quality.fitness_score = fitness_score;
    quality.num_iterations = num_iterations;
    
    // 计算内点比率
    quality.inlier_ratio = computeInlierRatio(source, target, transformation, 0.5f);
    
    // 计算相对上一次的位姿变化量（用于检测异常跳变）
    Eigen::Matrix4f delta = prev_veh_pose.inverse() * transformation;
    Eigen::Vector3f translation = delta.block<3, 1>(0, 3);
    quality.transformation_change = translation.norm();
    
    // 计算点云密度
    quality.point_density = static_cast<float>(source->points.size());
    
    return quality;
}

// ============================================================================
// 自适应局部地图半径计算
// 专为G1人形机器人优化：根据运动速度动态调整搜索范围
// ============================================================================
float computeAdaptiveLocalMapRadius(
    const Eigen::Matrix4f& current_pose,
    const Eigen::Matrix4f& previous_pose,
    float dt
) {
    // 计算位移
    Eigen::Vector3f displacement = current_pose.block<3, 1>(0, 3) - previous_pose.block<3, 1>(0, 3);
    float distance = displacement.norm();
    
    // 计算速度（米/秒）
    float velocity = distance / dt;
    
    // 计算旋转变化
    Eigen::Matrix3f rot_change = previous_pose.block<3, 3>(0, 0).transpose() * 
                                 current_pose.block<3, 3>(0, 0);
    Eigen::AngleAxisf angle_axis(rot_change);
    float angular_velocity = std::abs(angle_axis.angle()) / dt;
    
    // ========================================================================
    // G1人形机器人特定参数：
    // - 步行速度范围：0-1.5 m/s
    // - 转向速度范围：0-1.0 rad/s
    // - 行走时有周期性晃动，需要适当增大搜索范围
    // ========================================================================
    
    // 基础半径：考虑G1的身高（约130cm），基础范围设为25米
    const float BASE_RADIUS = 25.0f;
    const float MIN_RADIUS = 15.0f;   // 最小半径（静止或低速）
    const float MAX_RADIUS = 80.0f;   // 最大半径（高速运动）
    
    // 速度系数：速度越快，范围越大
    // G1最大步行速度约1.5m/s，设计安全系数为2.0
    float velocity_factor = 1.0f + std::min(velocity / 1.0f, 2.0f);
    
    // 角速度系数：转向时需要更大范围
    // 考虑G1转身时需要调整整个身体姿态
    float angular_factor = 1.0f + std::min(angular_velocity / 0.3f, 1.5f);
    
    // 综合半径计算
    float radius = BASE_RADIUS * velocity_factor * angular_factor;
    
    // 限制在合理范围内
    radius = std::max(MIN_RADIUS, std::min(radius, MAX_RADIUS));
    
    // 额外考虑：如果检测到高频晃动（G1步态特征），适当增大范围
    // 这里通过速度变化率来粗略估计
    static float prev_velocity = 0.0f;
    float velocity_change_rate = std::abs(velocity - prev_velocity) / dt;
    if (velocity_change_rate > 2.0f) {  // 快速加减速
        radius *= 1.2f;  // 增大20%
    }
    prev_velocity = velocity;
    
    return radius;
}

void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if (!received_initial_pose) {
        // 从消息中提取位置
        veh_pose(0, 3) = msg->pose.pose.position.x;
        veh_pose(1, 3) = msg->pose.pose.position.y;
        veh_pose(2, 3) = 1.3;

        // 从消息中提取姿态（四元数）
        Eigen::Quaterniond q;
        tf2::fromMsg(msg->pose.pose.orientation, q);

        // 将两个旋转组合起来
        Eigen::Quaternionf q_final = q.cast<float>();

        // 将四元数转换为旋转矩阵
        veh_pose.topLeftCorner(3, 3) = q_final.toRotationMatrix();

        received_initial_pose = true;
        RCLCPP_INFO(g_node->get_logger(), "Received initial pose from RViz");
    }
}
 
void pub_path(Eigen::Matrix4f &pose) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = t_msg;
    pose_msg.header.frame_id = "map";
    
    // 转换位置
    Eigen::Vector3d position = pose.block<3, 1>(0, 3).cast<double>();
    pose_msg.pose.position = tf2::toMsg(position);
    
    // 转换姿态
    Eigen::Quaterniond quat(pose.block<3, 3>(0, 0).cast<double>());
    pose_msg.pose.orientation = tf2::toMsg(quat);
    
    pose_pub->publish(pose_msg);

    // 发布机器人轨迹
    path_msg.header.stamp = t_msg;
    path_msg.header.frame_id = "map";
    path_msg.poses.push_back(pose_msg);
    path_pub->publish(path_msg);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr get_local_map(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_map, pcl::PointXYZ &center_point, float thr) {
    // 创建一个新的点云，用于存储剪裁后的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_map(new pcl::PointCloud<pcl::PointXYZ>);

    // 遍历所有点，将距离小于阈值的点添加到剪裁后的点云中
    for (const auto &point : *pcl_map) {
        double distance = pcl::euclideanDistance(center_point, point);
        if (distance <= thr) {
            local_map->push_back(point);
        }
    }
    return local_map;
}

void registration(pcl::PointCloud<pcl::PointXYZ>::Ptr &points, pcl::PointCloud<pcl::PointXYZ>::Ptr &local_map) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
    
    if (!inited) {
        inited = true;

        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

        // 设置要匹配的目标点云
        ndt.setInputSource(points);
        ndt.setInputTarget(local_map);

        ndt.setTransformationEpsilon(1e-8);
        ndt.setStepSize(0.1);
        ndt.setMaximumIterations(30);
        ndt.setResolution(1.0);

        ndt.align(*aligned, veh_pose);

        Eigen::Matrix4f candidate_pose = ndt.getFinalTransformation();
        
        // 评估NDT初始化质量
        RegistrationQuality quality = evaluateRegistrationQuality(
            points, local_map, candidate_pose,
            ndt.getFitnessScore(),
            ndt.getFinalNumIteration()
        );
        
        RCLCPP_INFO(g_node->get_logger(),
            "NDT初始化完成: 迭代次数=%d, 适应度=%.4f, 置信度=%.2f, x=%.2f, y=%.2f", 
            quality.num_iterations, quality.fitness_score, quality.getConfidence(),
            candidate_pose(0, 3), candidate_pose(1, 3));
        
        // NDT初始化通常接受，但记录质量
        veh_pose = candidate_pose;
        prev_veh_pose = veh_pose;
        
        // 发布置信度
        if (confidence_pub) {
            std_msgs::msg::Float32 conf_msg;
            conf_msg.data = quality.getConfidence();
            confidence_pub->publish(conf_msg);
        }
        
    } else {
        // ====================================================================
        // FastGICP配准（适用于G1人形机器人的运行时跟踪）
        // ====================================================================
        fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setInputSource(points);
        gicp.setInputTarget(local_map);
        gicp.setNumThreads(8);
        gicp.setMaxCorrespondenceDistance(0.5);
        gicp.align(*aligned, veh_pose);
        
        Eigen::Matrix4f candidate_pose = gicp.getFinalTransformation();
        
        // ====================================================================
        // 配准质量评估（关键改进：防止G1行走时的误匹配）
        // ====================================================================
        RegistrationQuality quality = evaluateRegistrationQuality(
            points, local_map, candidate_pose,
            gicp.getFitnessScore(),
            30  // GICP没有提供迭代次数，使用默认值
        );
        
        float confidence = quality.getConfidence();
        
        // ====================================================================
        // G1人形机器人特殊处理：
        // 行走时会有周期性晃动，需要更宽松的置信度阈值
        // 但同时要防止大幅度的位姿跳变
        // ====================================================================
        const float CONFIDENCE_THRESHOLD = 0.15f;  // G1行走时适当降低阈值
        const float MAX_POSITION_JUMP = 4.0f;      // 最大位置跳变4米
        const float MAX_ROTATION_JUMP = 2.0f;      // 最大旋转跳变（弧度）
        
        // 计算位置变化
        Eigen::Vector3f pos_change = candidate_pose.block<3, 1>(0, 3) - prev_veh_pose.block<3, 1>(0, 3);
        float position_jump = pos_change.norm();
        
        // 计算旋转变化
        Eigen::Matrix3f rot_change = prev_veh_pose.block<3, 3>(0, 0).transpose() * 
                                     candidate_pose.block<3, 3>(0, 0);
        Eigen::AngleAxisf angle_axis(rot_change);
        float rotation_jump = std::abs(angle_axis.angle());
        
        bool accept_result = true;
        std::string reject_reason = "";
        
        // 判断1：置信度过低
        if (confidence < CONFIDENCE_THRESHOLD) {
            accept_result = false;
            reject_reason = "低置信度";
        }
        
        // 判断2：位置跳变过大（除非是初次定位后）
        if (quality_history.size() > 5 && position_jump > MAX_POSITION_JUMP) {
            accept_result = false;
            reject_reason = "位置跳变过大";
        }
        
        // 判断3：旋转跳变过大
        if (quality_history.size() > 5 && rotation_jump > MAX_ROTATION_JUMP) {
            accept_result = false;
            reject_reason = "旋转跳变过大";
        }
        
        if (accept_result) {
            // 接受配准结果
            veh_pose = candidate_pose;
            prev_veh_pose = veh_pose;
            
            RCLCPP_INFO_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 1000,
                "配准成功: 置信度=%.2f, 内点率=%.2f, 适应度=%.4f, 位移=%.2fm, 旋转=%.2frad",
                confidence, quality.inlier_ratio, quality.fitness_score,
                position_jump, rotation_jump);
        } else {
            // 拒绝配准结果，保持上一次位姿
            // RCLCPP_WARN(g_node->get_logger(),
            RCLCPP_WARN_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 1000,
                "配准被拒绝 [%s]: 置信度=%.2f, 位移=%.2fm, 旋转=%.2frad - 保持上次位姿",
                reject_reason.c_str(), confidence, position_jump, rotation_jump);
            
            // 对于G1机器人，可以使用简单的运动模型预测
            // 这里暂时保持位姿不变，实际部署时可集成IMU数据
            candidate_pose = prev_veh_pose;
        }
        
        // 保存质量历史（最多保留最近50次）
        quality_history.push_back(quality);
        if (quality_history.size() > 50) {
            quality_history.erase(quality_history.begin());
        }
        
        // 发布置信度信息
        if (confidence_pub) {
            std_msgs::msg::Float32 conf_msg;
            conf_msg.data = confidence;
            confidence_pub->publish(conf_msg);
        }
    }

    // 发布配准后的点云
    sensor_msgs::msg::PointCloud2 points_msg; 
    pcl::toROSMsg(*aligned, points_msg);
    points_msg.header.stamp = t_msg;
    points_msg.header.frame_id = "map";

    points_pub->publish(points_msg);

    pub_path(veh_pose);
    // RCLCPP_INFO_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 1000,
    //     "发布配准点云，点数: %zu", aligned->size());
    // RCLCPP_INFO(g_node->get_logger(), "对齐点云大小: %zu", aligned->size());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr normalSpaceSampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int target_points) {
    // 计算法向量
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(20);
    ne.compute(*normals);

    // 创建带法向量的点云
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    // 法向量空间采样
    pcl::PointCloud<pcl::PointNormal>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalSpaceSampling<pcl::PointNormal, pcl::PointNormal> nss;
    nss.setInputCloud(cloud_with_normals);
    nss.setNormals(cloud_with_normals);
    nss.setBins(8, 8, 8);
    nss.setSample(target_points);
    nss.filter(*sampled_cloud);

    // 转换回普通点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*sampled_cloud, *result);
    
    return result;
}

void pointCloudProcess(pcl::PointCloud<pcl::PointXYZ>::Ptr &points_filtered) {
    auto start = std::chrono::high_resolution_clock::now();

    points_filtered = normalSpaceSampling(points_filtered, 4000);

    pcl::PointCloud<pcl::PointXYZ>::Ptr local_map(new pcl::PointCloud<pcl::PointXYZ>);
    auto center = pcl::PointXYZ(veh_pose(0, 3), veh_pose(1, 3), veh_pose(2, 3));

    // ========================================================================
    // 改进：使用自适应局部地图半径（专为G1人形机器人优化）
    // ========================================================================
    static auto last_time = std::chrono::high_resolution_clock::now();
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> time_delta = current_time - last_time;
    float dt = time_delta.count();
    
    // 计算自适应半径
    float adaptive_radius = computeAdaptiveLocalMapRadius(veh_pose, prev_veh_pose, dt);
    
    // 使用自适应半径获取局部地图
    local_map = get_local_map(points_map, center, adaptive_radius);
    
    last_time = current_time;

    auto end0 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed0 = end0 - start;

    registration(points_filtered, local_map);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_total = end - start;
    // RCLCPP_INFO_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 1000,
    //     "定位耗时: %.2f[ms], 预处理: %.2f[ms], 点数: %zu, 局部地图: %zu (半径: %.1fm)", 
    //     elapsed_total.count() * 1e3, elapsed0.count() * 1e3, 
    //     points_filtered->size(), local_map->points.size(), adaptive_radius);
}

void livox_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for (uint i = 1; i < msg->point_num; i++) {
        auto &pt_i = msg->points.at(i);
        double distance2 = pt_i.x * pt_i.x + pt_i.y * pt_i.y;
        
        if (distance2 < 0.25 || distance2 > 2500.0) {
            continue;
        }
        pcl::PointXYZ pt(pt_i.x, -pt_i.y, -pt_i.z);
        points_filtered->emplace_back(pt);
    }

    RCLCPP_INFO_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 1000,
        "Received Livox pointcloud: point_num=%u, filtered=%zu", 
        msg->point_num, points_filtered->size());

    t_msg = msg->header.stamp;
    pointCloudProcess(points_filtered);
    // RCLCPP_INFO_T(g_node->get_logger(), "Received Livox pointcloud, point_num=%u", msg->point_num);
    // RCLCPP_INFO_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 1000,
    //     "Received Livox pointcloud: filtered=%zu", points_filtered->size());
}

void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *points);

    pcl::PointCloud<pcl::PointXYZ>::Ptr points_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto &pt : *points) {
        double distance2 = pt.x * pt.x + pt.y * pt.y;
        if (distance2 < distance2_min_thr || distance2 > distance2_local_map_thr) {
            continue;
        }
        pcl::PointXYZ pt1(pt.x, -pt.y, -pt.z); 
        points_filtered->push_back(pt1);
    }
    t_msg = msg->header.stamp;
    pointCloudProcess(points_filtered);
}

void pointCloudPublisherThread(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub) {
    rclcpp::Rate loop_rate(1);
    while (rclcpp::ok()) {
        pub->publish(cloud_msg);
        loop_rate.sleep();
    }
}

void initializeMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& points_map, 
                   const std::string& map_file_path, 
                   float map_yaw, float map_pitch, float map_roll) {
    RCLCPP_INFO(g_node->get_logger(), "Reading map file: %s", map_file_path.c_str());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_file_path, *points_map) == -1) {
        RCLCPP_ERROR(g_node->get_logger(), "Couldn't read file.");
        return;
    }

    RCLCPP_INFO(g_node->get_logger(), "Original point cloud size: %zu points.", points_map->size());
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(points_map);
    sor.setLeafSize(0.2f, 0.2f, 0.2f);
    sor.filter(*points_map);
    
    RCLCPP_INFO(g_node->get_logger(), "Filtered point cloud size: %zu points.", points_map->size());

    RCLCPP_INFO(g_node->get_logger(), "Find the plane...");
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.1);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setInputCloud(points_map); 
    seg.segment(inliers, coefficients);

    RCLCPP_INFO(g_node->get_logger(),
        "Plane coefficients: %f %f %f %f",
        coefficients.values[0],
        coefficients.values[1],
        coefficients.values[2],
        coefficients.values[3]);

    RCLCPP_INFO(g_node->get_logger(), "Rotate map to z plane.");
    Eigen::Vector3f plane_normal(
        coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    plane_normal.normalize();
    Eigen::Vector3f horizontal_normal(0.0, 0.0, 1.0);
    Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(plane_normal, horizontal_normal);
    Eigen::Matrix3f rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix4f T_to_plane = Eigen::Matrix4f::Identity();
    T_to_plane(2, 3) = coefficients.values[3];
    T_to_plane.topLeftCorner<3, 3>() = rotation_matrix;
    int target_points = points_map->points.size();
    points_map = normalSpaceSampling(points_map, target_points/2);
    pcl::transformPointCloud(*points_map, *points_map, T_to_plane);

    RCLCPP_INFO(g_node->get_logger(), "Rotate map's yaw.");
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(
        Eigen::AngleAxisf(map_yaw, Eigen::Vector3f::UnitZ()) *
        Eigen::AngleAxisf(map_pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(map_roll, Eigen::Vector3f::UnitX()));

    pcl::transformPointCloud(*points_map, *points_map, transform);

    RCLCPP_INFO(g_node->get_logger(), "Map init finish.");
}
