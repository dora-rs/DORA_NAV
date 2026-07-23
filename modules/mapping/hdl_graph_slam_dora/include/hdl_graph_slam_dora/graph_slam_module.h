#pragma once
#include <vector>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>
#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>
#include <g2o/types/slam3d/types_slam3d.h>

namespace g2o {
class VertexSE3;
}

namespace hdl_graph_slam_dora {

struct KeyFrame;

class HdlGraphSlamModule {
public:
    HdlGraphSlamModule();
    void loadConfig(const YAML::Node& graph_slam_config,
                    const YAML::Node& loop_closure_config,
                    const YAML::Node& constraints_config);
    void addKeyFrame(const std::shared_ptr<KeyFrame>& keyframe);
    void optimize();
    std::vector<std::shared_ptr<KeyFrame>> getKeyFrames() const;

private:
    // 内部方法
    void initializeGraph();
    void addOdometryEdge(int from_idx, int to_idx);
    void addLoopClosureEdge(int from_idx, int to_idx, const Eigen::Isometry3d& relative_pose);
    void performLoopDetection(int new_keyframe_idx);
    Eigen::MatrixXd calculateInformationMatrix(const Eigen::Isometry3d& relative_pose);

    // IMU约束边
    void addImuAccelerationEdge(int keyframe_idx);
    void addImuOrientationEdge(int keyframe_idx);

    // 关键帧管理
    std::vector<std::shared_ptr<KeyFrame>> keyframes_;

    // g2o图优化
    std::unique_ptr<hdl_graph_slam::GraphSLAM> graph_slam_;
    std::unique_ptr<hdl_graph_slam::InformationMatrixCalculator> inf_calculator_;

    // 配置参数 - g2o
    std::string g2o_solver_type_;
    int g2o_solver_num_iterations_;

    // 配置参数 - 图优化
    double graph_update_interval_;
    double last_optimization_time_;
    bool fix_first_node_;

    // 配置参数 - 回环检测
    double loop_distance_thresh_;
    double loop_accum_distance_thresh_;     // 候选帧与当前帧的最小累积路程差(过滤近邻假回环)
    double loop_check_interval_;            // 回环检测的触发间隔(每走这么远检测一次),与上者解耦
    int loop_min_edge_interval_;
    double loop_fitness_score_thresh_;
    double last_loop_check_distance_;
    int last_loop_edge_idx_;               // 上次成功闭环的关键帧索引,用于抑制连续多帧重复闭环

    // 配置参数 - 回环配准
    std::string loop_registration_method_;
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr loop_registration_;
    YAML::Node loop_closure_config_;  // 保存loop_closure配置用于延迟初始化

    // 配置参数 - IMU约束
    bool enable_imu_acceleration_;
    bool enable_imu_orientation_;
    double imu_acceleration_stddev_;
    double imu_orientation_stddev_;
    std::string imu_acceleration_robust_kernel_;
    double imu_acceleration_robust_kernel_size_;
    std::string imu_orientation_robust_kernel_;
    double imu_orientation_robust_kernel_size_;

    // 统计
    int loop_closure_count_;
};

} // namespace hdl_graph_slam_dora
