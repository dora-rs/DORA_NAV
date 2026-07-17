#ifndef RERUN_MAPPER_HPP
#define RERUN_MAPPER_HPP

#include <iostream>
#include <vector>
#include <deque>
#include <memory>

#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <rerun.hpp>

namespace rerun_mapping {

struct Pose {
    double x, y, z;
    double roll, pitch, yaw;

    Pose() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {}
    Pose(double x_, double y_, double z_, double roll_, double pitch_, double yaw_)
        : x(x_), y(y_), z(z_), roll(roll_), pitch(pitch_), yaw(yaw_) {}

    Eigen::Matrix4f toMatrix() const;
    double distanceTo(const Pose& other) const;
};

class RerunMapper {
public:
    static constexpr float GLOBAL_MAP_VOXEL_SIZE = 0.5f;
    static constexpr int CURRENT_FRAME_DOWNSAMPLE = 5;
    static constexpr float TRAJECTORY_SAMPLE_DISTANCE = 0.5f;

    RerunMapper();
    ~RerunMapper() = default;

    void onMappingData(const float* data, size_t len);
    void onPoseData(const Pose& pose);
    void onPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void onFullMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);  // 新增：完整地图替换

private:
    void visualizeGlobalMap();
    void visualizeCurrentFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void visualizePose(const Pose& pose);
    void visualizeTrajectory();

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformPointCloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const Eigen::Matrix4f& transform);

    std::shared_ptr<rerun::RecordingStream> rec_;

    pcl::PointCloud<pcl::PointXYZI> global_map_;
    std::deque<Pose> trajectory_;
    Pose current_pose_;
    Pose last_trajectory_pose_;

    int frame_count_ = 0;
};

} // namespace rerun_mapping

#endif // RERUN_MAPPER_HPP
