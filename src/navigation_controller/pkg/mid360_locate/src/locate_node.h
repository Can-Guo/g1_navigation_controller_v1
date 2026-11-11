#ifndef LOCATE_NODE_H
#define LOCATE_NODE_H

#include <string>
#include <thread>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <pcl/registration/ndt.h>

#include <Eigen/Core>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "fast_gicp/gicp/fast_gicp.hpp" 
#include <pcl/filters/normal_space.h>
#include <pcl/features/normal_3d.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// 全局常量
constexpr float distance2_min_thr = 0.25f;
constexpr float distance2_local_map_thr = 2500.f; 

// ============================================================================
// G1人形机器人配准质量评估结构体
// 用于评估LiDAR配准的可靠性，适配人形机器人行走时的动态特性
// ============================================================================
struct RegistrationQuality {
    float fitness_score;           // 配准适应度分数 (越小越好)
    float inlier_ratio;           // 内点比率 (0-1, 越大越好)
    float transformation_change;   // 相对上次的位姿变化量
    float point_density;          // 点云密度
    int num_iterations;           // 迭代次数
    
    // 计算综合置信度 (0-1, 越高越可靠)
    float getConfidence() const {
        // 适应度分数归一化 (假设好的适应度 < 0.5)
        float fitness_norm = std::max(0.0f, 1.0f - fitness_score / 0.5f);
        
        // 迭代次数惩罚 (迭代太多说明收敛困难)
        float iteration_penalty = num_iterations < 20 ? 1.0f : 0.7f;
        
        // 位姿变化惩罚 (对于G1行走，过大的突变不合理)
        float change_penalty = transformation_change < 0.5f ? 1.0f : 
                              (transformation_change < 1.0f ? 0.8f : 0.5f);
        
        // 点云密度加权 (密度低时降低置信度)
        float density_weight = std::min(1.0f, point_density / 1000.0f);
        
        // 综合评分：各项加权求和
        return (0.35f * fitness_norm +           // 35% 适应度
                0.30f * inlier_ratio +           // 30% 内点率
                0.20f * iteration_penalty +      // 20% 迭代效率
                0.15f * density_weight) *        // 15% 点云质量
               change_penalty;                   // 乘以变化惩罚
    }
    
    // 判断是否应该接受此配准结果
    bool isAcceptable(float threshold = 0.5f) const {
        return getConfidence() >= threshold;
    }
};

// 全局变量
extern bool inited;
extern Eigen::Matrix4f veh_pose;
extern Eigen::Matrix4f prev_veh_pose;  // 新增：上一次的位姿，用于计算变化量
extern pcl::PointCloud<pcl::PointXYZ>::Ptr points_map;
extern sensor_msgs::msg::PointCloud2 cloud_msg;
extern nav_msgs::msg::Path path_msg;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub;
extern rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
extern rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
extern rclcpp::Time t_msg;
extern bool received_initial_pose;
extern std::shared_ptr<rclcpp::Node> g_node;

// 新增：配准置信度发布者和历史记录
extern rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr confidence_pub;
extern std::vector<RegistrationQuality> quality_history;  // 保存最近的配准质量历史

// 函数声明
pcl::PointCloud<pcl::PointXYZ>::Ptr normalSpaceSampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int target_points);
void initializeMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& points_map, const std::string& map_file_path, float map_yaw, float map_pitch, float map_roll);
void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
void pub_path(Eigen::Matrix4f &pose);
pcl::PointCloud<pcl::PointXYZ>::Ptr get_local_map(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_map, pcl::PointXYZ &center_point, float thr);
void registration(pcl::PointCloud<pcl::PointXYZ>::Ptr &points, pcl::PointCloud<pcl::PointXYZ>::Ptr &local_map);
void pointCloudProcess(pcl::PointCloud<pcl::PointXYZ>::Ptr &points_filtered);
void livox_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
void pointCloudPublisherThread(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub);

// 新增：配准质量评估函数
RegistrationQuality evaluateRegistrationQuality(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    const Eigen::Matrix4f& transformation,
    float fitness_score,
    int num_iterations
);

// 新增：计算内点比率
float computeInlierRatio(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    const Eigen::Matrix4f& transformation,
    float max_correspondence_distance = 0.5f
);

// 新增：自适应局部地图半径计算（专为G1人形机器人优化）
float computeAdaptiveLocalMapRadius(
    const Eigen::Matrix4f& current_pose,
    const Eigen::Matrix4f& previous_pose,
    float dt = 0.1f  // 时间间隔（秒）
);

#endif // LOCATE_NODE_H 