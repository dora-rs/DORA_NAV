#include "hdl_graph_slam_dora/scan_matching_odometry_module.h"
#include <hdl_graph_slam/registrations.hpp>
#include <iostream>

namespace hdl_graph_slam_dora {

ScanMatchingOdometryModule::ScanMatchingOdometryModule()
    : keyframe_delta_trans_(1.0)
    , keyframe_delta_angle_(1.0)
    , keyframe_delta_time_(10000.0)
    , registration_method_("FAST_GICP")
    , max_fitness_score_(2.5)
    , max_consecutive_failures_(5)
    , consecutive_failures_(0)
    , prev_cloud_(nullptr)
    , current_pose_(Eigen::Isometry3d::Identity())
    , prev_keyframe_pose_(Eigen::Isometry3d::Identity())
    , prev_keyframe_time_(0.0)
    , is_first_frame_(true)
    , last_delta_pose_(Eigen::Isometry3d::Identity())
    , accum_distance_(0.0)
{
}

void ScanMatchingOdometryModule::loadConfig(const YAML::Node& config) {
    keyframe_delta_trans_ = config["keyframe_delta_trans"].as<double>(1.0);
    keyframe_delta_angle_ = config["keyframe_delta_angle"].as<double>(1.0);
    keyframe_delta_time_ = config["keyframe_delta_time"].as<double>(10000.0);
    registration_method_ = config["registration_method"].as<std::string>("FAST_GICP");

    // 配准质量与失败自愈参数
    max_fitness_score_ = config["max_fitness_score"].as<double>(2.5);
    max_consecutive_failures_ = config["max_consecutive_failures"].as<int>(5);

    // 转换角度为弧度
    keyframe_delta_angle_ = keyframe_delta_angle_ * M_PI / 180.0;

    std::cout << "[ScanMatchingOdometry] About to initialize registration..." << std::endl;
    initializeRegistration(config);
    std::cout << "[ScanMatchingOdometry] Registration initialized" << std::endl;

    std::cout << "[ScanMatchingOdometry] About to print config..." << std::endl;

    std::cout << "[ScanMatchingOdometry] Loaded config: method=" << registration_method_
              << ", trans_thresh=" << keyframe_delta_trans_
              << "m, angle_thresh=" << (keyframe_delta_angle_ * 180.0 / M_PI) << "deg"
              << ", max_fitness=" << max_fitness_score_
              << ", max_consec_fail=" << max_consecutive_failures_ << std::endl;
}

void ScanMatchingOdometryModule::initializeRegistration(const YAML::Node& config) {
    registration_ = hdl_graph_slam::select_registration_method(config);
    if (!registration_) {
        std::cerr << "[ScanMatchingOdometry] Failed to initialize registration method!" << std::endl;
    }
}

bool ScanMatchingOdometryModule::processFrame(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    double timestamp,
    KeyFrame::Ptr& keyframe_out
) {
    if (!cloud || cloud->empty()) {
        std::cerr << "[ScanMatchingOdometry] Empty cloud received" << std::endl;
        return false;
    }

    if (is_first_frame_) {
        // 第一帧初始化（参考ROS版本第168-174行）
        is_first_frame_ = false;
        prev_keyframe_time_ = timestamp;
        prev_keyframe_pose_ = Eigen::Isometry3d::Identity();  // keyframe_pose
        last_delta_pose_ = Eigen::Isometry3d::Identity();     // prev_trans
        current_pose_ = Eigen::Isometry3d::Identity();

        // 第一帧作为关键帧，并设为配准目标
        prev_cloud_ = cloud;
        registration_->setInputTarget(cloud);

        keyframe_out = std::make_shared<KeyFrame>();
        keyframe_out->timestamp = timestamp;
        keyframe_out->pose = prev_keyframe_pose_;
        keyframe_out->cloud = cloud;
        keyframe_out->accum_distance = 0.0;

        std::cout << "[ScanMatchingOdometry] First frame initialized as keyframe" << std::endl;
        return true;
    }

    // 记录上一帧位姿用于计算帧间距离
    Eigen::Isometry3d prev_pose = current_pose_;

    // 配准：当前帧 -> 关键帧（参考ROS版本第210行）
    // 使用 last_delta_pose_ (即prev_trans) 作为初始猜测
    bool success = false;
    Eigen::Isometry3d trans = performMatching(cloud, prev_cloud_, last_delta_pose_, success);

    if (!success) {
        // 配准失败：用上次变换dead-reckon（参考ROS版本第214-218行）
        consecutive_failures_++;
        trans = last_delta_pose_;

        // 更新dead-reckon全局位姿并累加距离
        current_pose_ = prev_keyframe_pose_ * trans;
        Eigen::Isometry3d fd = prev_pose.inverse() * current_pose_;
        accum_distance_ += fd.translation().norm();

        // 连续失败过多：强制用当前帧重锚定target，打破"目标帧过期→永久冻结"死循环
        if (consecutive_failures_ >= max_consecutive_failures_) {
            std::cerr << "[ScanMatchingOdometry] Re-anchoring target to current frame after "
                      << consecutive_failures_ << " consecutive failures" << std::endl;

            prev_cloud_ = cloud;
            registration_->setInputTarget(cloud);
            prev_keyframe_pose_ = current_pose_;
            prev_keyframe_time_ = timestamp;
            last_delta_pose_ = Eigen::Isometry3d::Identity();
            consecutive_failures_ = 0;

            // 生成重锚定关键帧，让图从新锚点继续（位姿为dead-reckon估计，但target已刷新可恢复配准）
            keyframe_out = std::make_shared<KeyFrame>();
            keyframe_out->timestamp = timestamp;
            keyframe_out->pose = current_pose_;
            keyframe_out->cloud = cloud;
            keyframe_out->accum_distance = accum_distance_;
            return true;
        }

        // 未达重锚定阈值：保留dead-reckon位姿，但失败帧绝不生成关键帧（避免用错误位姿灌爆位姿图）
        std::cerr << "[ScanMatchingOdometry] Registration failed ("
                  << consecutive_failures_ << "/" << max_consecutive_failures_
                  << "), dead-reckon, no keyframe" << std::endl;
        last_delta_pose_ = trans;
        return false;
    }

    // 配准成功：重置连续失败计数
    consecutive_failures_ = 0;

    // 计算全局位姿（参考ROS版本第220-221行）
    // odom = keyframe_pose * trans
    current_pose_ = prev_keyframe_pose_ * trans;

    // 计算帧间距离并累加
    Eigen::Isometry3d frame_delta = prev_pose.inverse() * current_pose_;
    accum_distance_ += frame_delta.translation().norm();

    // 保存相对于关键帧的变换（参考ROS版本第236行）
    last_delta_pose_ = trans;

    // 计算相对于关键帧的位移和角度变化（参考ROS版本第241-243行）
    double delta_trans = trans.translation().norm();
    Eigen::AngleAxisd angle_axis(trans.rotation());
    double delta_angle = std::abs(angle_axis.angle());
    double delta_time = timestamp - prev_keyframe_time_;

    // 判断是否为关键帧（参考ROS版本第244-252行）
    if (delta_trans > keyframe_delta_trans_ ||
        delta_angle > keyframe_delta_angle_ ||
        delta_time > keyframe_delta_time_) {

        // 创建新关键帧
        keyframe_out = std::make_shared<KeyFrame>();
        keyframe_out->timestamp = timestamp;
        keyframe_out->pose = current_pose_;
        keyframe_out->cloud = cloud;
        keyframe_out->accum_distance = accum_distance_;

        // 更新关键帧（参考ROS版本第245-251行）
        prev_cloud_ = cloud;
        registration_->setInputTarget(cloud);
        prev_keyframe_pose_ = current_pose_;
        prev_keyframe_time_ = timestamp;
        last_delta_pose_ = Eigen::Isometry3d::Identity();  // 重置相对变换

        return true;
    }

    return false;
}

Eigen::Isometry3d ScanMatchingOdometryModule::performMatching(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& target,
    const Eigen::Isometry3d& initial_guess,
    bool& success
) {
    if (!registration_) {
        std::cerr << "[ScanMatchingOdometry] Registration not initialized!" << std::endl;
        success = false;
        return initial_guess;
    }

    registration_->setInputSource(source);
    // Target已经在关键帧更新时通过setInputTarget设置了

    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
    registration_->align(*aligned, initial_guess.matrix().cast<float>());

    success = registration_->hasConverged();

    if (!success) {
        double fitness = registration_->getFitnessScore();
        std::cerr << "[ScanMatchingOdometry] Registration failed to converge! Fitness: " << fitness << std::endl;
        return initial_guess;
    }

    // 获取配准结果：从关键帧到当前帧的变换（参考ROS版本第220行）
    Eigen::Matrix4f result_matrix = registration_->getFinalTransformation();
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.matrix() = result_matrix.cast<double>();

    double fitness_score = registration_->getFitnessScore();

    // 验证配准质量(阈值可配置,原硬编码1.0对室外稀疏场景过严)
    if (fitness_score > max_fitness_score_) {
        std::cerr << "[ScanMatchingOdometry] Poor fitness score: " << fitness_score
                  << " > " << max_fitness_score_ << std::endl;
        success = false;
        return initial_guess;
    }

    return result;
}

} // namespace hdl_graph_slam_dora
