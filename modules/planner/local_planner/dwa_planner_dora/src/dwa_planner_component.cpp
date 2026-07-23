#include "dwa_planner/dwa_planner_component.hpp"
#include <limits>
#include <algorithm>
#include <iostream>

namespace dwa_planner {

// =====================================================================
//  主计算接口（简化版：固定线速度，只采样角速度）
// =====================================================================
std::pair<double, double> DWAPlannerComponent::compute(
    const RobotState& current_state,
    const std::vector<double>& local_path_x,
    const std::vector<double>& local_path_y,
    const CostmapBuilder& costmap)
{
    // 清空上一帧的轨迹
    all_trajectories_.clear();

    // 边界条件：路径为空
    if (local_path_x.empty() || local_path_y.empty()) {
        std::cerr << "[DWA] empty path, stopping" << std::endl;
        return {0.0, 0.0};
    }

    // 计算到路径终点的距离
    double dist_to_goal = distance(0.0, 0.0, local_path_x.back(), local_path_y.back());

    // 终点停止判断：距离终点很近时直接停止
    if (dist_to_goal < 0.15) {
        std::cout << "[DWA] Reached goal (dist=" << dist_to_goal << "m), stopping" << std::endl;
        return {0.0, 0.0};
    }

    // 固定线速度
    double v = config_.fixed_v;

    // 1. 计算角速度动态窗口
    double w_min, w_max;
    calculateDynamicWindow(current_state.omega, w_min, w_max);

    // std::cout << "[DWA] Fixed v=" << v << " m/s, Dynamic Window: w=["
    //           << w_min << ", " << w_max << "]" << std::endl;

    // 2. 在动态窗口内采样角速度并评估
    double best_v = v;
    double best_w = 0.0;
    double best_score = -std::numeric_limits<double>::infinity();

    // 记录最优轨迹的各项得分
    double best_score_obstacle = 0.0;
    double best_score_heading = 0.0;

    int valid_trajectories = 0;
    int collision_trajectories = 0;

    // 角速度采样
    for (int j = 0; j < config_.w_samples; ++j) {
        double w = w_min + (w_max - w_min) * j / std::max(1, config_.w_samples - 1);

        // 预测轨迹（接近终点时限制预测距离）
        auto trajectory = predictTrajectory(current_state, v, w, dist_to_goal);

        // 保存轨迹用于可视化
        all_trajectories_.push_back(trajectory);

        // 评估得分（得分越高越好）
        double score = evaluateTrajectory(trajectory, local_path_x, local_path_y, costmap);

        // 统计
        if (std::isinf(score) && score < 0) {
            collision_trajectories++;
        } else {
            valid_trajectories++;
        }

        // 更新最优解（选择得分最高的）
        if (score > best_score) {
            best_score = score;
            best_w = w;

            // 记录最优轨迹的各项得分
            best_score_obstacle = calculateObstacleScore(trajectory, costmap);
            best_score_heading = calculateHeadingScore(trajectory, local_path_x, local_path_y);
        }
    }

    // std::cout << "[DWA] Sampled " << config_.w_samples
    //           << " trajectories: " << valid_trajectories << " valid, "
    //           << collision_trajectories << " collision" << std::endl;

    // 3. 速度平滑
    smoothVelocity(best_w);

    // 4. 可视化（调试用）
    if (enable_visualization_) {
        visualize(costmap);
    }

    // 无可行解时停车
    if (std::isinf(best_score) && best_score < 0) {
        std::cerr << "[DWA] no feasible trajectory, stopping" << std::endl;
        best_v = 0.0;
        best_w = 0.0;
    } else {
        // // 打印最优解的详细信息
        // std::cout << "[DWA] Best trajectory: v=" << best_v << " m/s, w=" << best_w << " rad/s" << std::endl;
        // std::cout << "      Total score=" << best_score << " (higher is better)" << std::endl;
        // std::cout << "      - Obstacle score: " << best_score_obstacle
        //           << " (weight=" << config_.weight_obstacle << ")" << std::endl;
        // std::cout << "      - Heading score:  " << best_score_heading
        //           << " (weight=" << config_.weight_heading << ")" << std::endl;
    }

    return {best_v, best_w};
}

// =====================================================================
//  1. 计算角速度动态窗口
// =====================================================================
void DWAPlannerComponent::calculateDynamicWindow(
    double current_w,
    double& w_min, double& w_max)
{
    // 约束1：机器人物理限制
    w_min = -config_.max_w;
    w_max = config_.max_w;

    // 约束2：加速度限制（使用实际控制周期）
    // 控制周期：timer触发频率 = 50ms = 0.05s
    double control_dt = 0.05;
    w_min = std::max(w_min, current_w - config_.max_accel_w * control_dt);
    w_max = std::min(w_max, current_w + config_.max_accel_w * control_dt);

    // 确保窗口有效
    if (w_min > w_max) w_min = w_max;
}

// =====================================================================
//  2. 预测轨迹（差分驱动运动学模型）
// =====================================================================
std::vector<std::pair<float, float>> DWAPlannerComponent::predictTrajectory(
    const RobotState& state, double v, double w, double goal_distance)
{
    std::vector<std::pair<float, float>> trajectory;

    // 初始状态（机器人坐标系原点）
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;

    // 预测时间步数
    int steps = static_cast<int>(config_.predict_time / config_.dt);

    for (int i = 0; i <= steps; ++i) {
        // 检查是否超过终点距离
        if (goal_distance > 0) {
            double dist_from_origin = std::sqrt(x * x + y * y);
            if (dist_from_origin > goal_distance) {
                // 超过终点，停止生成后续点
                break;
            }
        }

        trajectory.emplace_back(static_cast<float>(x), static_cast<float>(y));

        // 差速模型运动学更新（欧拉积分）
        x += v * std::cos(yaw) * config_.dt;
        y += v * std::sin(yaw) * config_.dt;
        yaw += w * config_.dt;
    }

    return trajectory;
}

// =====================================================================
//  3. 评估轨迹总得分（得分越高越好）
// =====================================================================
double DWAPlannerComponent::evaluateTrajectory(
    const std::vector<std::pair<float, float>>& trajectory,
    const std::vector<double>& path_x,
    const std::vector<double>& path_y,
    const CostmapBuilder& costmap)
{
    // 1. 避障得分（碰撞则返回负无穷）
    double score_obstacle = calculateObstacleScore(trajectory, costmap);
    if (std::isinf(score_obstacle) && score_obstacle < 0) {
        return -std::numeric_limits<double>::infinity();
    }

    // 2. 朝向得分
    double score_heading = calculateHeadingScore(trajectory, path_x, path_y);

    // 加权总得分
    double total_score = config_.weight_obstacle * score_obstacle +
                        config_.weight_heading * score_heading;

    return total_score;
}

// =====================================================================
//  3.1 避障得分（距离障碍物越远得分越高）
// =====================================================================
double DWAPlannerComponent::calculateObstacleScore(
    const std::vector<std::pair<float, float>>& trajectory,
    const CostmapBuilder& costmap)
{
    // 检查轨迹是否碰撞
    if (!costmap.isTrajectoryCollisionFree(trajectory)) {
        return -std::numeric_limits<double>::infinity();
    }

    // 计算到最近障碍物的距离
    double min_dist = costmap.getMinDistanceToObstacle(trajectory);

    // 局部代价地图的最大感知距离（对角线距离）
    // 地图为6m×6m（以机器人为中心，半径3m的正方形）
    // 最远距离 = 3 * sqrt(2) ≈ 4.24m
    const double max_perception_dist = 3.0 * std::sqrt(2.0);

    // 如果没有找到障碍物（距离为无穷大），得分最高
    if (std::isinf(min_dist)) {
        return 1.0;
    }

    // 距离越远得分越高，使用感知范围归一化 [0, 1]
    // min_dist = 0 → score = 0.0
    // min_dist >= max_perception_dist → score = 1.0
    double score = std::min(min_dist / max_perception_dist, 1.0);

    return score;
}

// =====================================================================
//  3.2 朝向得分（轨迹终点朝向与局部目标方向的夹角越小得分越高）
// =====================================================================
double DWAPlannerComponent::calculateHeadingScore(
    const std::vector<std::pair<float, float>>& trajectory,
    const std::vector<double>& path_x,
    const std::vector<double>& path_y)
{
    if (trajectory.empty() || path_x.empty()) {
        return 0.0;
    }

    // 轨迹终点
    auto end_point = trajectory.back();
    float end_x = end_point.first;
    float end_y = end_point.second;

    // 找到距离机器人当前位置（原点）最近的路径点索引
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path_x.size(); ++i) {
        double dist = distance(0.0, 0.0, path_x[i], path_y[i]);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    // 从最近点向前取5个点作为局部终点
    // 如果剩余点数不足5个，则取路径的最后一个点
    size_t local_goal_idx = std::min(closest_idx + 5, path_x.size() - 1);
    double local_goal_x = path_x[local_goal_idx];
    double local_goal_y = path_y[local_goal_idx];

    // 计算从轨迹终点指向局部终点的向量
    double dx = local_goal_x - end_x;
    double dy = local_goal_y - end_y;

    // 计算轨迹终点的朝向（使用轨迹最后两个点）
    double traj_yaw = 0.0;
    if (trajectory.size() >= 2) {
        auto prev_point = trajectory[trajectory.size() - 2];
        double traj_dx = end_x - prev_point.first;
        double traj_dy = end_y - prev_point.second;
        traj_yaw = std::atan2(traj_dy, traj_dx);
    }

    // 局部目标方向的角度
    double goal_yaw = std::atan2(dy, dx);

    // 计算夹角（归一化到[-π, π]）
    double angle_diff = goal_yaw - traj_yaw;
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

    // 夹角越小得分越高（纯夹角评分）
    // angle_diff = 0° → score = 1.0
    // angle_diff = ±180° → score = 0.0
    double angle_score = (M_PI - std::abs(angle_diff)) / M_PI;

    return angle_score;
}

// =====================================================================
//  4. 速度平滑（只对角速度进行平滑）
// =====================================================================
void DWAPlannerComponent::smoothVelocity(double& w)
{
    double alpha = config_.velocity_smooth_alpha;

    // 指数移动平均
    w = alpha * w + (1.0 - alpha) * last_w_;

    // 更新历史
    last_w_ = w;
}

// =====================================================================
//  5. 可视化（用于调试）
// =====================================================================
void DWAPlannerComponent::visualize(const CostmapBuilder& costmap)
{
    // 获取代价地图信息
    int width_cells = costmap.getWidthCells();
    int height_cells = costmap.getHeightCells();
    const auto& costmap_data = costmap.getCostmap();

    // 代价地图分辨率（从CostmapConfig获取，默认0.1m）
    float resolution = 0.1f;

    // 创建可视化图像（使用真实比例）
    float map_width_m = width_cells * resolution;
    float map_height_m = height_cells * resolution;
    int img_width = static_cast<int>(map_width_m * viz_scale_);
    int img_height = static_cast<int>(map_height_m * viz_scale_);
    cv::Mat viz_img(img_height, img_width, CV_8UC3, cv::Scalar(255, 255, 255));

    // 1. 绘制代价地图（黑色表示障碍物）
    // 坐标系转换：
    // 机器人坐标系：x向前（上），y向左
    // 图像坐标系：x向右，y向下
    // 转换规则：img_x = center_x - local_y * scale, img_y = center_y - local_x * scale
    for (int grid_y = 0; grid_y < height_cells; ++grid_y) {
        for (int grid_x = 0; grid_x < width_cells; ++grid_x) {
            int idx = grid_y * width_cells + grid_x;
            uint8_t cost = costmap_data[idx];

            // 代价地图栅格坐标 -> 机器人坐标系（米）
            // 地图中心为机器人位置
            float local_x = (grid_x - width_cells / 2.0f) * resolution;
            float local_y = (grid_y - height_cells / 2.0f) * resolution;

            // 机器人坐标系 -> 图像坐标
            // 上方为x轴（向前），左方为y轴（向左）
            int img_x = img_width / 2 - static_cast<int>(local_y * viz_scale_);
            int img_y = img_height / 2 - static_cast<int>(local_x * viz_scale_);

            if (img_x >= 0 && img_x < img_width && img_y >= 0 && img_y < img_height) {
                if (cost > 200) {  // 障碍物
                    viz_img.at<cv::Vec3b>(img_y, img_x) = cv::Vec3b(0, 0, 0);  // 黑色
                }
            }
        }
    }

    // 2. 绘制机器人（空心圆圈在中心）
    cv::Point robot_center(img_width / 2, img_height / 2);
    int robot_radius_px = static_cast<int>(0.10 * viz_scale_);  // 机器人半径0.10m（缩小显示）
    cv::circle(viz_img, robot_center, robot_radius_px, cv::Scalar(0, 128, 0), 2);  // 绿色空心圆，2像素线宽

    // 3. 绘制所有预测轨迹（红色）
    // 跳过前3个点，与碰撞检测逻辑保持一致
    for (const auto& trajectory : all_trajectories_) {
        size_t start_idx = std::min(size_t(3), trajectory.size());

        for (size_t i = start_idx; i < trajectory.size(); ++i) {
            const auto& point = trajectory[i];
            float local_x = point.first;
            float local_y = point.second;

            // 机器人坐标系 -> 图像坐标
            int img_x = img_width / 2 - static_cast<int>(local_y * viz_scale_);
            int img_y = img_height / 2 - static_cast<int>(local_x * viz_scale_);

            if (img_x >= 0 && img_x < img_width && img_y >= 0 && img_y < img_height) {
                // 红色轨迹点（覆盖障碍物）
                int point_size = (i == trajectory.size() - 1) ? 3 : 2;  // 终点更大
                cv::circle(viz_img, cv::Point(img_x, img_y), point_size, cv::Scalar(0, 0, 255), -1);
            }
        }
    }

    // 4. 添加坐标轴标注和比例信息
    cv::putText(viz_img, "X (forward)", cv::Point(img_width / 2 - 50, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    cv::putText(viz_img, "Y (left)", cv::Point(10, img_height / 2),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

    std::string scale_info = "Scale: " + std::to_string(viz_scale_) + " px/m, " +
                            "Predict: " + std::to_string(config_.predict_time) + "s";
    cv::putText(viz_img, scale_info, cv::Point(10, img_height - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

    // 5. 显示图像
    cv::imshow("DWA Trajectory Visualization", viz_img);
    cv::waitKey(1);  // 非阻塞，允许实时更新
}

} // namespace dwa_planner
