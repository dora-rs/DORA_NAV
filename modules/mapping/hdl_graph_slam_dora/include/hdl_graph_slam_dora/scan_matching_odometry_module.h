#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <boost/optional.hpp>

namespace g2o {
class VertexSE3;
}

namespace hdl_graph_slam_dora {

struct KeyFrame {
    using Ptr = std::shared_ptr<KeyFrame>;

    double timestamp;
    Eigen::Isometry3d pose;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    double accum_distance;
    g2o::VertexSE3* node = nullptr;  // g2o图节点指针

    // IMU 数据（可选）
    boost::optional<Eigen::Vector3d> acceleration;    // IMU 加速度
    boost::optional<Eigen::Quaterniond> orientation;  // IMU 姿态
};

class ScanMatchingOdometryModule {
public:
    ScanMatchingOdometryModule();
    void loadConfig(const YAML::Node& config);

    bool processFrame(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        double timestamp,
        KeyFrame::Ptr& keyframe_out
    );

    Eigen::Isometry3d getCurrentPose() const { return current_pose_; }

private:
    // 配置参数
    double keyframe_delta_trans_;
    double keyframe_delta_angle_;
    double keyframe_delta_time_;
    std::string registration_method_;

    // 配准质量与失败自愈参数
    double max_fitness_score_;         // fitness超过此值判为配准失败(原硬编码1.0,现可配置)
    int max_consecutive_failures_;     // 连续失败达到此数则强制用当前帧重锚定target

    // 状态:连续失败计数
    int consecutive_failures_;

    // 配准算法
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration_;

    // 状态
    pcl::PointCloud<pcl::PointXYZI>::Ptr prev_cloud_;
    Eigen::Isometry3d current_pose_;
    Eigen::Isometry3d prev_keyframe_pose_;
    double prev_keyframe_time_;
    bool is_first_frame_;
    Eigen::Isometry3d last_delta_pose_;
    double accum_distance_;

    void initializeRegistration(const YAML::Node& config);
    bool isKeyFrame(const Eigen::Isometry3d& relative_pose, double delta_time);
    Eigen::Isometry3d performMatching(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& target,
        const Eigen::Isometry3d& initial_guess,
        bool& success
    );
};

} // namespace hdl_graph_slam_dora
