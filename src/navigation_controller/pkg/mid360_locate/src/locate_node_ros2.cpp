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
// ROS parameters (defaults can be overridden via launch)
// ============================================================================
bool g_enable_deskew = true;
std::string g_imu_topic = "/livox/imu";
double g_imu_buffer_sec = 5.0;
int64_t g_imu_time_offset_ns = 0;
int g_deskew_reference_mode = 0;  // 0=start, 1=mid, 2=end

float g_confidence_threshold = 0.20f;
float g_max_position_jump_m = 4.0f;
float g_max_rotation_jump_rad = 2.0f;
int g_reject_check_warmup_frames = 5;

// ============================================================================
// IMU buffer + helper functions for rotation-only deskew
// ============================================================================
namespace {
struct ImuSample {
  rclcpp::Time stamp;
  Eigen::Vector3d gyro_rad_s;  // angular velocity (rad/s)
};

std::mutex g_imu_mutex;
std::deque<ImuSample> g_imu_buffer;

inline double toSec(const rclcpp::Time& t) {
  return static_cast<double>(t.nanoseconds()) * 1e-9;
}

void trimImuBufferLocked(const rclcpp::Time& newest_stamp) {
  const int64_t keep_ns = static_cast<int64_t>(g_imu_buffer_sec * 1e9);
  const int64_t min_ns = newest_stamp.nanoseconds() - keep_ns;
  while (!g_imu_buffer.empty() && g_imu_buffer.front().stamp.nanoseconds() < min_ns) {
    g_imu_buffer.pop_front();
  }
}

bool buildGyroTrajectoryForScan(
    const rclcpp::Time& scan_start,
    const rclcpp::Time& scan_end,
    std::vector<double>& imu_times,
    std::vector<Eigen::Quaterniond>& imu_quats,
    std::string& fail_reason) {
  std::vector<ImuSample> imu;
  {
    std::lock_guard<std::mutex> lk(g_imu_mutex);
    if (g_imu_buffer.size() < 2) {
      fail_reason = "IMU buffer too small (<2 samples)";
      return false;
    }
    // Copy relevant IMU samples to avoid holding the mutex while integrating.
    for (const auto& s : g_imu_buffer) {
      const int64_t ns = s.stamp.nanoseconds();
      if (ns >= scan_start.nanoseconds() - 50'000'000LL && ns <= scan_end.nanoseconds() + 50'000'000LL) {
        imu.push_back(s);
      }
    }
  }

  if (imu.size() < 2) {
    fail_reason = "Not enough IMU samples covering scan time window";
    return false;
  }

  std::sort(imu.begin(), imu.end(), [](const ImuSample& a, const ImuSample& b) {
    return a.stamp.nanoseconds() < b.stamp.nanoseconds();
  });

  const double t0 = toSec(scan_start);
  imu_times.clear();
  imu_quats.clear();
  imu_times.reserve(imu.size());
  imu_quats.reserve(imu.size());

  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  imu_times.push_back(std::max(0.0, toSec(imu.front().stamp) - t0));
  imu_quats.push_back(q);

  for (size_t i = 1; i < imu.size(); ++i) {
    const double t_prev = toSec(imu[i - 1].stamp);
    const double t_curr = toSec(imu[i].stamp);
    double dt = t_curr - t_prev;
    if (dt <= 0.0 || dt > 0.2) {
      // Non-monotonic or too large dt: skip integration to keep stable.
      dt = 0.0;
    }

    const Eigen::Vector3d w_prev = imu[i - 1].gyro_rad_s;
    const Eigen::Vector3d w_curr = imu[i].gyro_rad_s;
    const Eigen::Vector3d w_avg = 0.5 * (w_prev + w_curr);
    const Eigen::Vector3d dtheta = w_avg * dt;

    const double angle = dtheta.norm();
    if (angle > 1e-12) {
      const Eigen::Vector3d axis = dtheta / angle;
      const Eigen::AngleAxisd aa(angle, axis);
      q = (q * Eigen::Quaterniond(aa)).normalized();
    }

    imu_times.push_back(std::max(0.0, t_curr - t0));
    imu_quats.push_back(q);
  }

  return imu_times.size() >= 2;
}

Eigen::Quaterniond interpolateQuat(
    const std::vector<double>& imu_times,
    const std::vector<Eigen::Quaterniond>& imu_quats,
    double t,
    size_t& idx) {
  if (imu_times.empty()) {
    return Eigen::Quaterniond::Identity();
  }
  if (t <= imu_times.front()) {
    return imu_quats.front();
  }
  if (t >= imu_times.back()) {
    return imu_quats.back();
  }
  while (idx + 1 < imu_times.size() && imu_times[idx + 1] < t) {
    idx++;
  }
  const size_t i0 = idx;
  const size_t i1 = std::min(idx + 1, imu_times.size() - 1);
  const double t0 = imu_times[i0];
  const double t1 = imu_times[i1];
  const double alpha = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;
  return imu_quats[i0].slerp(alpha, imu_quats[i1]).normalized();
}

bool deskewLivoxCustomMsg(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud,
    std::string& fail_reason) {
  if (!g_enable_deskew) {
    fail_reason = "deskew disabled by parameter";
    return false;
  }
  if (!msg || msg->point_num < 2) {
    fail_reason = "empty scan";
    return false;
  }

  const int64_t timebase_ns = static_cast<int64_t>(msg->timebase);
  // msg->header.stamp is builtin_interfaces::msg::Time, convert it explicitly.
  const rclcpp::Time scan_start = (timebase_ns > 0) ? rclcpp::Time(timebase_ns) : rclcpp::Time(msg->header.stamp);

  uint32_t max_offset = 0;
  for (uint32_t i = 0; i < msg->point_num; ++i) {
    max_offset = std::max(max_offset, msg->points[i].offset_time);
  }
  const rclcpp::Time scan_end = scan_start + rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(max_offset));

  std::vector<double> imu_times;
  std::vector<Eigen::Quaterniond> imu_quats;
  const rclcpp::Time scan_start_imu = scan_start + rclcpp::Duration::from_nanoseconds(g_imu_time_offset_ns);
  const rclcpp::Time scan_end_imu = scan_end + rclcpp::Duration::from_nanoseconds(g_imu_time_offset_ns);
  if (!buildGyroTrajectoryForScan(scan_start_imu, scan_end_imu, imu_times, imu_quats, fail_reason)) {
    return false;
  }

  const double scan_duration = std::max(1e-6, static_cast<double>(max_offset) * 1e-9);
  double tref = 0.0;
  if (g_deskew_reference_mode == 1) {
    tref = 0.5 * scan_duration;
  } else if (g_deskew_reference_mode == 2) {
    tref = scan_duration;
  }

  size_t imu_idx_ref = 0;
  const Eigen::Quaterniond q_ref = interpolateQuat(imu_times, imu_quats, tref, imu_idx_ref);

  size_t imu_idx = 0;
  out_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
  out_cloud->reserve(msg->point_num);

  for (uint32_t i = 1; i < msg->point_num; i++) {
    const auto& pt_i = msg->points.at(i);
    const double distance2 = static_cast<double>(pt_i.x) * pt_i.x + static_cast<double>(pt_i.y) * pt_i.y;
    if (distance2 < 0.25 || distance2 > 2500.0) {
      continue;
    }

    // Keep the same axis convention as the original code path.
    const Eigen::Vector3d p(pt_i.x, -pt_i.y, -pt_i.z);
    const double ti = static_cast<double>(pt_i.offset_time) * 1e-9;  // assume ns
    const Eigen::Quaterniond q_t = interpolateQuat(imu_times, imu_quats, ti, imu_idx);

    // Rotation from reference time to point time:
    //   q_ref_to_t = q_ref^{-1} * q_t
    // Deskew point to reference frame:
    //   p_ref = q_ref_to_t * p_t
    const Eigen::Vector3d p_ref = (q_ref.conjugate() * q_t) * p;
    out_cloud->emplace_back(static_cast<float>(p_ref.x()),
                            static_cast<float>(p_ref.y()),
                            static_cast<float>(p_ref.z()));
  }

  return !out_cloud->empty();
}
}  // namespace

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
        // 阈值来自 ROS 参数（见 main_ros2.cpp），便于针对不同场景/速度调参。
        const float CONFIDENCE_THRESHOLD = g_confidence_threshold;
        const float MAX_POSITION_JUMP = g_max_position_jump_m;
        const float MAX_ROTATION_JUMP = g_max_rotation_jump_rad;
        
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
        if (static_cast<int>(quality_history.size()) > g_reject_check_warmup_frames && position_jump > MAX_POSITION_JUMP) {
            accept_result = false;
            reject_reason = "位置跳变过大";
        }
        
        // 判断3：旋转跳变过大
        if (static_cast<int>(quality_history.size()) > g_reject_check_warmup_frames && rotation_jump > MAX_ROTATION_JUMP) {
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

    // 自适应采样数量：不超过实际点云大小，避免 NormalSpaceSampling 警告
    // 当点云较少时直接全量使用，当点云较多时采样到 4000 以控制计算量
    int sample_target = std::min(static_cast<int>(points_filtered->size()), 4000);
    if (sample_target > 50) {  // 点数太少时跳过采样，直接用原始点云
        points_filtered = normalSpaceSampling(points_filtered, sample_target);
    }

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
    // =========================================================================
    // Rotation-only deskew (IMU gyro + per-point offset_time)
    //
    // 实机问题背景（G1 大幅转向时常见）：
    // - MID360 的一帧点云不是“瞬时”采样，而是一个时间扫掠；
    // - 机器人转向时，同一帧内前后点的姿态不同，墙面会在点云里呈现扇形/弯曲；
    // - 直接将畸变点云送入 GICP/NDT，会导致配准适应度变差、置信度下降，严重时发生跳变/丢定位。
    //
    // 这里的策略：
    // - 若开启 deskew 且 IMU 数据充足：对每个点按采样时间做旋转补偿，把点云拉回同一参考时刻；
    // - 若 deskew 失败：自动回退到原来的不补偿模式（保证系统可用，但转向稳定性会下降）。
    // =========================================================================
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_filtered;
    std::string deskew_fail_reason;
    const bool deskew_ok = deskewLivoxCustomMsg(msg, points_filtered, deskew_fail_reason);

    if (!deskew_ok) {
        // Fallback: keep original behavior.
        points_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>);
        points_filtered->reserve(msg->point_num);
        for (uint32_t i = 1; i < msg->point_num; i++) {
            const auto& pt_i = msg->points.at(i);
            const double distance2 = static_cast<double>(pt_i.x) * pt_i.x + static_cast<double>(pt_i.y) * pt_i.y;
            if (distance2 < 0.25 || distance2 > 2500.0) {
                continue;
            }
            points_filtered->emplace_back(pt_i.x, -pt_i.y, -pt_i.z);
        }

        RCLCPP_WARN_THROTTLE(
            g_node->get_logger(), *g_node->get_clock(), 2000,
            "Deskew disabled/failed (%s). Fallback to raw scan. Turning stability may degrade.",
            deskew_fail_reason.c_str());
    }

    RCLCPP_INFO_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 1000,
        "Livox scan: point_num=%u, used=%zu, deskew=%s",
        msg->point_num, points_filtered->size(), deskew_ok ? "ON" : "OFF");

    t_msg = msg->header.stamp;
    pointCloudProcess(points_filtered);
}

void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!msg) {
        return;
    }

    // Deskew uses gyro only:
    // - Gyro integration is reliable for short-term rotation within a scan.
    // - IMU absolute orientation may drift (no magnetometer), so we don't require it here.
    ImuSample s;
    s.stamp = msg->header.stamp;
    s.gyro_rad_s = Eigen::Vector3d(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z);

    std::lock_guard<std::mutex> lk(g_imu_mutex);
    if (!g_imu_buffer.empty() && s.stamp.nanoseconds() <= g_imu_buffer.back().stamp.nanoseconds()) {
        // Drop out-of-order IMU samples to keep integration stable.
        return;
    }
    g_imu_buffer.push_back(s);
    trimImuBufferLocked(s.stamp);
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
