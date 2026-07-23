#include "hdl_graph_slam_dora/graph_slam_module.h"
#include "hdl_graph_slam_dora/scan_matching_odometry_module.h"
#include <hdl_graph_slam/registrations.hpp>
#include <iostream>
#include <chrono>
#include <cmath>

namespace hdl_graph_slam_dora {

HdlGraphSlamModule::HdlGraphSlamModule()
    : g2o_solver_type_("lm_var_cholmod")
    , g2o_solver_num_iterations_(512)
    , graph_update_interval_(3.0)
    , last_optimization_time_(0.0)
    , fix_first_node_(true)
    , loop_distance_thresh_(20.0)
    , loop_accum_distance_thresh_(35.0)
    , loop_check_interval_(10.0)
    , loop_min_edge_interval_(5)
    , loop_fitness_score_thresh_(0.5)
    , last_loop_check_distance_(0.0)
    , last_loop_edge_idx_(-1000000)
    , enable_imu_acceleration_(false)
    , enable_imu_orientation_(false)
    , imu_acceleration_stddev_(3.0)
    , imu_orientation_stddev_(0.1)
    , imu_acceleration_robust_kernel_("NONE")
    , imu_acceleration_robust_kernel_size_(1.0)
    , imu_orientation_robust_kernel_("NONE")
    , imu_orientation_robust_kernel_size_(1.0)
    , loop_closure_count_(0)
{
}

void HdlGraphSlamModule::loadConfig(const YAML::Node& graph_slam_config,
                                     const YAML::Node& loop_closure_config,
                                     const YAML::Node& constraints_config) {
    // ============================================================
    // 1. 加载 graph_slam 节点参数
    // ============================================================

    // g2o求解器参数
    g2o_solver_type_ = graph_slam_config["g2o_solver_type"].as<std::string>("lm_var_cholmod");
    g2o_solver_num_iterations_ = graph_slam_config["g2o_solver_num_iterations"].as<int>(512);

    // 图优化参数
    graph_update_interval_ = graph_slam_config["graph_update_interval"].as<double>(3.0);
    fix_first_node_ = graph_slam_config["fix_first_node"].as<bool>(true);

    std::cout << "[HdlGraphSlamModule] Loaded graph_slam config: solver=" << g2o_solver_type_
              << ", iterations=" << g2o_solver_num_iterations_
              << ", update_interval=" << graph_update_interval_ << "s" << std::endl;

    // ============================================================
    // 2. 加载 loop_closure 节点参数
    // ============================================================

    if (loop_closure_config) {
        loop_distance_thresh_ = loop_closure_config["distance_thresh"].as<double>(20.0);
        loop_accum_distance_thresh_ = loop_closure_config["accum_distance_thresh"].as<double>(35.0);
        // 触发间隔独立于候选路程下限:默认取accum_thresh的1/3,保证走回起点段也能触发检测
        loop_check_interval_ = loop_closure_config["check_interval"].as<double>(loop_accum_distance_thresh_ / 3.0);
        loop_min_edge_interval_ = loop_closure_config["min_edge_interval"].as<int>(5);
        loop_fitness_score_thresh_ = loop_closure_config["fitness_score_thresh"].as<double>(0.5);

        loop_registration_method_ = loop_closure_config["registration_method"].as<std::string>("FAST_GICP");

        // 保存配置用于延迟初始化回环配准
        loop_closure_config_ = loop_closure_config;

        std::cout << "[HdlGraphSlamModule] Loaded loop_closure config: distance_thresh="
                  << loop_distance_thresh_ << "m, accum_thresh=" << loop_accum_distance_thresh_
                  << "m, check_interval=" << loop_check_interval_
                  << "m, min_interval=" << loop_min_edge_interval_
                  << ", fitness_thresh=" << loop_fitness_score_thresh_
                  << ", registration=" << loop_registration_method_ << std::endl;
    } else {
        std::cout << "[HdlGraphSlamModule] Warning: loop_closure config not found, using defaults" << std::endl;
    }

    // ============================================================
    // 3. 加载 constraints 节点参数
    // ============================================================

    if (constraints_config && constraints_config["imu"]) {
        const auto& imu_config = constraints_config["imu"];

        // IMU加速度约束
        if (imu_config["acceleration"]) {
            const auto& acc_config = imu_config["acceleration"];
            enable_imu_acceleration_ = acc_config["enabled"].as<bool>(false);
            imu_acceleration_stddev_ = acc_config["stddev"].as<double>(3.0);
            imu_acceleration_robust_kernel_ = acc_config["robust_kernel"].as<std::string>("NONE");
            imu_acceleration_robust_kernel_size_ = acc_config["robust_kernel_size"].as<double>(1.0);
        }

        // IMU方向约束
        if (imu_config["orientation"]) {
            const auto& ori_config = imu_config["orientation"];
            enable_imu_orientation_ = ori_config["enabled"].as<bool>(false);
            imu_orientation_stddev_ = ori_config["stddev"].as<double>(0.1);
            imu_orientation_robust_kernel_ = ori_config["robust_kernel"].as<std::string>("NONE");
            imu_orientation_robust_kernel_size_ = ori_config["robust_kernel_size"].as<double>(1.0);
        }

        std::cout << "[HdlGraphSlamModule] Loaded constraints.imu config: acceleration="
                  << (enable_imu_acceleration_ ? "ON" : "OFF")
                  << ", orientation=" << (enable_imu_orientation_ ? "ON" : "OFF") << std::endl;
    } else {
        std::cout << "[HdlGraphSlamModule] Warning: constraints.imu config not found, IMU disabled" << std::endl;
    }
}

void HdlGraphSlamModule::initializeGraph() {
    if (!graph_slam_) {
        std::cout << "[HdlGraphSlamModule] Initializing g2o graph with solver: "
                  << g2o_solver_type_ << std::endl;
        graph_slam_ = std::make_unique<hdl_graph_slam::GraphSLAM>(g2o_solver_type_);

        // 初始化信息矩阵计算器（使用默认参数）
        inf_calculator_ = std::make_unique<hdl_graph_slam::InformationMatrixCalculator>();

        std::cout << "[HdlGraphSlamModule] Graph initialized successfully" << std::endl;
    }
}

void HdlGraphSlamModule::addKeyFrame(const std::shared_ptr<KeyFrame>& keyframe) {
    // 初始化图（延迟初始化）
    if (!graph_slam_) {
        initializeGraph();
    }

    // 添加关键帧到列表
    keyframes_.push_back(keyframe);
    int current_idx = keyframes_.size() - 1;

    // 添加节点到g2o图
    keyframe->node = graph_slam_->add_se3_node(keyframe->pose);

    std::cout << "[HdlGraphSlamModule] Added keyframe #" << keyframes_.size()
              << " at timestamp " << keyframe->timestamp
              << " | Pose: [" << keyframe->pose.translation().transpose() << "]" << std::endl;

    // 固定第一个节点
    if (current_idx == 0 && fix_first_node_) {
        keyframe->node->setFixed(true);
        std::cout << "[HdlGraphSlamModule] First node fixed" << std::endl;
    }

    // 添加里程计边（连接到前一个关键帧）
    if (current_idx > 0) {
        addOdometryEdge(current_idx - 1, current_idx);
    }

    // 添加 IMU 约束边
    if (keyframe->acceleration && enable_imu_acceleration_) {
        addImuAccelerationEdge(current_idx);
    }
    if (keyframe->orientation && enable_imu_orientation_) {
        addImuOrientationEdge(current_idx);
    }

    // 回环检测：每个关键帧都检查。
    // "定期触发"会漏掉走回起点的关键帧恰好落在两次触发空档的情况(实测始终不闭合)。
    // 候选筛选(accum路程差 + 水平距离)很廉价,真正贵的align只对通过筛选的少数候选执行,故每帧检查开销可控。
    performLoopDetection(current_idx);

    // 定时优化
    double current_time = keyframe->timestamp;
    if (keyframes_.size() > 2 &&
        (current_time - last_optimization_time_) > graph_update_interval_) {
        optimize();
        last_optimization_time_ = current_time;
    }
}

void HdlGraphSlamModule::addOdometryEdge(int from_idx, int to_idx) {
    auto& kf_from = keyframes_[from_idx];
    auto& kf_to = keyframes_[to_idx];

    // 计算相对位姿
    Eigen::Isometry3d relative_pose = kf_from->pose.inverse() * kf_to->pose;

    // 计算信息矩阵（简化版，使用常量矩阵）
    Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6);
    information.topLeftCorner(3, 3) *= 1.0 / (0.5 * 0.5);  // 平移方差
    information.bottomRightCorner(3, 3) *= 1.0 / (0.1 * 0.1);  // 旋转方差

    // 添加边到图
    graph_slam_->add_se3_edge(kf_from->node, kf_to->node, relative_pose, information);
}

void HdlGraphSlamModule::addLoopClosureEdge(int from_idx, int to_idx,
                                             const Eigen::Isometry3d& relative_pose) {
    auto& kf_from = keyframes_[from_idx];
    auto& kf_to = keyframes_[to_idx];

    // 回环边使用更高的信息矩阵（更可信）
    Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6);
    information.topLeftCorner(3, 3) *= 1.0 / (0.3 * 0.3);  // 平移方差
    information.bottomRightCorner(3, 3) *= 1.0 / (0.05 * 0.05);  // 旋转方差

    // 添加回环边
    graph_slam_->add_se3_edge(kf_from->node, kf_to->node, relative_pose, information);

    loop_closure_count_++;
    std::cout << "[HdlGraphSlamModule] Added loop closure edge: " << from_idx
              << " <-> " << to_idx << " (total loops: " << loop_closure_count_ << ")" << std::endl;
}

void HdlGraphSlamModule::performLoopDetection(int new_keyframe_idx) {
    if (new_keyframe_idx < loop_min_edge_interval_ + 1) {
        return;  // 太少关键帧，不检测
    }

    // 抑制连续多帧重复闭环:刚闭环后的min_edge_interval帧内不再检测,
    // 避免走到起点附近时每帧都加一条冗余回环边并触发全图优化。
    if (new_keyframe_idx - last_loop_edge_idx_ < loop_min_edge_interval_) {
        return;
    }

    auto& new_kf = keyframes_[new_keyframe_idx];
    Eigen::Vector3d new_pos = new_kf->pose.translation();

    // 初始化回环配准算法（延迟初始化）
    if (!loop_registration_) {
        if (loop_closure_config_) {
            // 使用配置文件中的参数
            loop_registration_ = hdl_graph_slam::select_registration_method(loop_closure_config_);
            std::cout << "[HdlGraphSlamModule] Loop registration initialized from config: "
                      << loop_registration_method_ << std::endl;
        } else {
            // 降级到硬编码默认值（不应该发生）
            YAML::Node fallback_config;
            fallback_config["registration_method"] = "FAST_GICP";
            fallback_config["reg_num_threads"] = 4;
            fallback_config["reg_transformation_epsilon"] = 0.01;
            fallback_config["reg_maximum_iterations"] = 64;
            fallback_config["reg_max_correspondence_distance"] = 2.5;
            fallback_config["reg_correspondence_randomness"] = 20;

            loop_registration_ = hdl_graph_slam::select_registration_method(fallback_config);
            std::cout << "[HdlGraphSlamModule] Warning: Loop registration initialized with fallback defaults" << std::endl;
        }
    }

    // 搜索回环候选
    int candidates_checked = 0;
    for (int i = 0; i < new_keyframe_idx - loop_min_edge_interval_; ++i) {
        auto& candidate_kf = keyframes_[i];
        Eigen::Vector3d candidate_pos = candidate_kf->pose.translation();

        // 真回环必须"走过的路程足够远、但空间上又靠近"。
        // 仅按空间距离筛选会把轨迹上前后紧挨的相邻帧误判为回环,产生冲突约束把后半程轨迹拧偏。
        // 因此先用累积路程差做下限过滤(原版hdl-graph-slam的核心筛选条件)。
        double accum_diff = new_kf->accum_distance - candidate_kf->accum_distance;
        if (accum_diff < loop_accum_distance_thresh_) {
            continue;  // 路程不够远,是近邻帧而非回环,跳过
        }

        // 检查空间距离(只用水平x,y):Z受里程计漂移污染(平地也漂到-5m),
        // 用被污染的Z当门槛会误杀真回环。回环本就是要纠正含Z在内的漂移。
        double dx = new_pos.x() - candidate_pos.x();
        double dy = new_pos.y() - candidate_pos.y();
        double distance = std::sqrt(dx * dx + dy * dy);
        if (distance > loop_distance_thresh_) {
            continue;
        }

        candidates_checked++;

        // 使用配准验证回环
        loop_registration_->setInputSource(new_kf->cloud);
        loop_registration_->setInputTarget(candidate_kf->cloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
        Eigen::Isometry3d initial_guess = candidate_kf->pose.inverse() * new_kf->pose;
        loop_registration_->align(*aligned, initial_guess.matrix().cast<float>());

        if (!loop_registration_->hasConverged()) {
            continue;
        }

        double fitness_score = loop_registration_->getFitnessScore();

        // 检查适应度得分
        if (fitness_score < loop_fitness_score_thresh_) {
            // 找到有效回环！
            Eigen::Matrix4f result_matrix = loop_registration_->getFinalTransformation();
            Eigen::Isometry3d relative_pose = Eigen::Isometry3d::Identity();
            relative_pose.matrix() = result_matrix.cast<double>();

            std::cout << "[LoopDetector] Loop detected! KF#" << new_keyframe_idx
                      << " <-> KF#" << i
                      << " | Distance: " << distance << "m"
                      << " | Fitness: " << fitness_score << std::endl;

            addLoopClosureEdge(i, new_keyframe_idx, relative_pose);
            last_loop_edge_idx_ = new_keyframe_idx;  // 记录闭环点,抑制后续几帧重复闭环

            // 找到一个回环后立即优化
            optimize();
            break;  // 每次只添加一个回环边
        }
    }

    if (candidates_checked > 0) {
        std::cout << "[LoopDetector] Checked " << candidates_checked << " candidates, "
                  << "no valid loop found" << std::endl;
    }
}

void HdlGraphSlamModule::optimize() {
    if (!graph_slam_ || keyframes_.size() < 2) {
        return;
    }

    std::cout << "[HdlGraphSlamModule] Optimizing graph ("
              << graph_slam_->num_vertices() << " nodes, "
              << graph_slam_->num_edges() << " edges)..." << std::endl;

    auto start = std::chrono::steady_clock::now();

    // 执行g2o优化
    int iterations = graph_slam_->optimize(g2o_solver_num_iterations_);

    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::cout << "[HdlGraphSlamModule] Optimization completed in " << elapsed << "ms"
              << " (" << iterations << " iterations)" << std::endl;

    // 更新所有关键帧的优化后位姿
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        if (keyframes_[i]->node) {
            keyframes_[i]->pose = keyframes_[i]->node->estimate();
        }
    }
}

std::vector<std::shared_ptr<KeyFrame>> HdlGraphSlamModule::getKeyFrames() const {
    return keyframes_;
}

void HdlGraphSlamModule::addImuAccelerationEdge(int keyframe_idx) {
    auto& keyframe = keyframes_[keyframe_idx];

    if (!keyframe->acceleration) {
        return;  // 没有加速度数据
    }

    // 加速度约束：重力向量应该指向 -Z 方向
    // direction: -Z 轴（世界坐标系中的重力方向）
    // measurement: IMU 测量的加速度（已转换到 base_link 坐标系）
    Eigen::Vector3d direction = -Eigen::Vector3d::UnitZ();
    Eigen::Vector3d measurement = keyframe->acceleration.get();

    // 信息矩阵（基于标准差）
    Eigen::MatrixXd information = Eigen::MatrixXd::Identity(3, 3) / (imu_acceleration_stddev_ * imu_acceleration_stddev_);

    // 添加先验向量边，使用 auto 接收返回值
    auto edge = graph_slam_->add_se3_prior_vec_edge(
        keyframe->node,
        direction,
        measurement,
        information
    );

    // 添加鲁棒核（如果配置了）
    if (imu_acceleration_robust_kernel_ != "NONE" && edge) {
        graph_slam_->add_robust_kernel(
            reinterpret_cast<g2o::HyperGraph::Edge*>(edge),
            imu_acceleration_robust_kernel_,
            imu_acceleration_robust_kernel_size_
        );
    }

    std::cout << "[HdlGraphSlamModule] Added IMU acceleration edge to keyframe #" << (keyframe_idx + 1)
              << " | acc: [" << measurement.transpose() << "]" << std::endl;
}

void HdlGraphSlamModule::addImuOrientationEdge(int keyframe_idx) {
    auto& keyframe = keyframes_[keyframe_idx];

    if (!keyframe->orientation) {
        return;  // 没有方向数据
    }

    Eigen::Quaterniond quat = keyframe->orientation.get();

    // 确保四元数 w 为正（标准化）
    if (quat.w() < 0.0) {
        quat.coeffs() = -quat.coeffs();
    }

    // 信息矩阵（基于标准差）
    Eigen::MatrixXd information = Eigen::MatrixXd::Identity(3, 3) / (imu_orientation_stddev_ * imu_orientation_stddev_);

    // 添加先验四元数边，使用 auto 接收返回值
    auto edge = graph_slam_->add_se3_prior_quat_edge(
        keyframe->node,
        quat,
        information
    );

    // 添加鲁棒核（如果配置了）
    if (imu_orientation_robust_kernel_ != "NONE" && edge) {
        graph_slam_->add_robust_kernel(
            reinterpret_cast<g2o::HyperGraph::Edge*>(edge),
            imu_orientation_robust_kernel_,
            imu_orientation_robust_kernel_size_
        );
    }

    std::cout << "[HdlGraphSlamModule] Added IMU orientation edge to keyframe #" << (keyframe_idx + 1)
              << " | quat: [" << quat.w() << ", " << quat.x() << ", " << quat.y() << ", " << quat.z() << "]" << std::endl;
}

} // namespace hdl_graph_slam_dora
