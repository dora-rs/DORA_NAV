#include "rerun_mapping/rerun_mapper.hpp"
#include <cmath>

namespace rerun_mapping {

Eigen::Matrix4f Pose::toMatrix() const {
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();

    float cy = std::cos(yaw), sy = std::sin(yaw);
    float cp = std::cos(pitch), sp = std::sin(pitch);
    float cr = std::cos(roll), sr = std::sin(roll);

    matrix(0, 0) = cy * cp;
    matrix(0, 1) = cy * sp * sr - sy * cr;
    matrix(0, 2) = cy * sp * cr + sy * sr;
    matrix(1, 0) = sy * cp;
    matrix(1, 1) = sy * sp * sr + cy * cr;
    matrix(1, 2) = sy * sp * cr - cy * sr;
    matrix(2, 0) = -sp;
    matrix(2, 1) = cp * sr;
    matrix(2, 2) = cp * cr;
    matrix(0, 3) = x;
    matrix(1, 3) = y;
    matrix(2, 3) = z;

    return matrix;
}

double Pose::distanceTo(const Pose& other) const {
    double dx = x - other.x, dy = y - other.y, dz = z - other.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

RerunMapper::RerunMapper() {
    rec_ = std::make_shared<rerun::RecordingStream>("ndt_mapping_viz");
    rec_->spawn().exit_on_failure();
    rec_->set_time_sequence("frame", 0);
    std::cout << "[RerunMapper] Initialized" << std::endl;
}

void RerunMapper::onMappingData(const float* data, size_t len) {
    size_t num_points = len / 3;
    for (size_t i = 0; i < num_points; ++i) {
        pcl::PointXYZI point;
        point.x = data[i * 3];
        point.y = data[i * 3 + 1];
        point.z = data[i * 3 + 2];
        point.intensity = 100.0f;
        global_map_.points.push_back(point);
    }
    global_map_.width = global_map_.points.size();
    global_map_.height = 1;
    visualizeGlobalMap();
}

void RerunMapper::onPoseData(const Pose& pose) {
    current_pose_ = pose;
    if (trajectory_.empty() ||
        current_pose_.distanceTo(last_trajectory_pose_) >= TRAJECTORY_SAMPLE_DISTANCE) {
        trajectory_.push_back(current_pose_);
        last_trajectory_pose_ = current_pose_;
    }
    visualizePose(pose);
    visualizeTrajectory();
}

void RerunMapper::onPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    rec_->set_time_sequence("frame", ++frame_count_);
    visualizeCurrentFrame(cloud);
}

void RerunMapper::onFullMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    if (!cloud || cloud->points.empty()) return;

    std::cout << "[RerunMapper] Received full map with " << cloud->points.size() << " points" << std::endl;

    // 替换整个全局地图（而不是累积）
    global_map_.clear();
    global_map_ = *cloud;
    global_map_.width = global_map_.points.size();
    global_map_.height = 1;

    visualizeGlobalMap();
}

void RerunMapper::visualizeGlobalMap() {
    if (global_map_.points.empty()) return;

    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(global_map_.makeShared());
    voxel.setLeafSize(GLOBAL_MAP_VOXEL_SIZE, GLOBAL_MAP_VOXEL_SIZE, GLOBAL_MAP_VOXEL_SIZE);
    pcl::PointCloud<pcl::PointXYZI> filtered;
    voxel.filter(filtered);

    std::vector<rerun::Position3D> positions;
    positions.reserve(filtered.points.size());
    for (const auto& p : filtered.points) {
        positions.emplace_back(p.x, p.y, p.z);
    }

    rec_->log("map/global", rerun::Points3D(positions)
        .with_colors(rerun::Color(200, 200, 200))
        .with_radii(0.05f));
}

void RerunMapper::visualizeCurrentFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    if (!cloud || cloud->points.empty()) return;

    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>);
    for (size_t i = 0; i < cloud->points.size(); i += CURRENT_FRAME_DOWNSAMPLE) {
        downsampled->points.push_back(cloud->points[i]);
    }

    auto transformed = transformPointCloud(downsampled, current_pose_.toMatrix());

    std::vector<rerun::Position3D> positions;
    positions.reserve(transformed->points.size());
    for (const auto& p : transformed->points) {
        positions.emplace_back(p.x, p.y, p.z);
    }

    rec_->log("map/current_frame", rerun::Points3D(positions)
        .with_colors(rerun::Color(255, 100, 100))
        .with_radii(0.08f));
}

void RerunMapper::visualizePose(const Pose& pose) {
    Eigen::Matrix4f mat = pose.toMatrix();

    // 从旋转矩阵提取四元数（简化方法，假设roll/pitch较小）
    float qw = std::sqrt(1.0f + mat(0,0) + mat(1,1) + mat(2,2)) / 2.0f;
    float qx = (mat(2,1) - mat(1,2)) / (4.0f * qw);
    float qy = (mat(0,2) - mat(2,0)) / (4.0f * qw);
    float qz = (mat(1,0) - mat(0,1)) / (4.0f * qw);

    rec_->log("map/pose", rerun::Transform3D::from_translation_rotation(
        {mat(0,3), mat(1,3), mat(2,3)},
        rerun::Rotation3D{rerun::datatypes::Quaternion::from_wxyz(qw, qx, qy, qz)}
    ));

    float s = 2.0f;
    std::vector<rerun::components::LineStrip3D> axes = {
        rerun::components::LineStrip3D({{0, 0, 0}, {s, 0, 0}}),
        rerun::components::LineStrip3D({{0, 0, 0}, {0, s, 0}}),
        rerun::components::LineStrip3D({{0, 0, 0}, {0, 0, s}})
    };
    rec_->log("map/pose/axes", rerun::LineStrips3D(axes)
        .with_colors({
            rerun::Color(255, 0, 0),
            rerun::Color(0, 255, 0),
            rerun::Color(0, 0, 255)
        })
        .with_radii({0.05f}));
}

void RerunMapper::visualizeTrajectory() {
    if (trajectory_.size() < 2) return;

    std::vector<rerun::Position3D> positions;
    positions.reserve(trajectory_.size());
    for (const auto& pose : trajectory_) {
        positions.emplace_back(pose.x, pose.y, pose.z);
    }

    std::vector<rerun::components::LineStrip3D> strips = {rerun::components::LineStrip3D(positions)};
    rec_->log("map/trajectory", rerun::LineStrips3D(strips)
        .with_colors(rerun::Color(100, 255, 100))
        .with_radii({0.1f}));
}

pcl::PointCloud<pcl::PointXYZI>::Ptr RerunMapper::transformPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const Eigen::Matrix4f& transform) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>);
    transformed->points.reserve(cloud->points.size());

    for (const auto& point : cloud->points) {
        Eigen::Vector4f p(point.x, point.y, point.z, 1.0f);
        Eigen::Vector4f pt = transform * p;
        pcl::PointXYZI new_point;
        new_point.x = pt[0];
        new_point.y = pt[1];
        new_point.z = pt[2];
        new_point.intensity = point.intensity;
        transformed->points.push_back(new_point);
    }

    transformed->width = transformed->points.size();
    transformed->height = 1;
    return transformed;
}

} // namespace rerun_mapping
