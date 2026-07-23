//
// Created by xiang on 25-4-21.
//

#ifndef LIGHTNING_LOOP_CLOSING_H
#define LIGHTNING_LOOP_CLOSING_H

#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "utils/async_message_process.h"

#include "core/graph/optimizer.h"
#include "core/types/edge_se3.h"

namespace lightning {

/**
 * 基于grid ndt的回环检测
 */
class LoopClosing {
   public:
    struct Options {
        Options() {}

        bool verbose_ = true;       // 输出调试信息
        bool online_mode_ = false;  // 切换离线-在线模式

        int loop_kf_gap_ = 20;       // 每隔多少个关键帧检查一次
        int min_id_interval_ = 20;   // 被检查的关键帧ID间隔
        int closest_id_th_ = 50;     // 历史关键帧与当前帧的ID间隔
        double max_range_ = 30.0;    // 候选帧的最大距离
        double ndt_score_th_ = 1.0;  // ndt位姿分值

        /// 图优化权重
        double motion_trans_noise_ = 0.1;               // 位移权重
        double motion_rot_noise_ = 3.0 * M_PI / 180.0;  // 旋转权重

        double loop_trans_noise_ = 0.2;               // 位移权重
        double loop_rot_noise_ = 3.0 * M_PI / 180.0;  // 旋转权重

        double rk_loop_th_ = 5.2 / 5;  // 回环的RK阈值

        bool with_height_ = true;
        double height_noise_ = 0.1;
    };

    LoopClosing(Options options = Options()) { options_ = options; }
    ~LoopClosing();

    void Init(const std::string yaml_path);

    /// 向回环中添加一个关键帧
    void AddKF(Keyframe::Ptr kf);

    /// 如果检测到新地回环并发生了优化，则调用回调
    using LoopClosedCallback = std::function<void()>;
    void SetLoopClosedCB(LoopClosedCallback cb) { loop_cb_ = cb; }

   protected:
    void HandleKF(Keyframe::Ptr kf);

    void DetectLoopCandidates();

    /// 计算回环候选位姿
    void ComputeLoopCandidates();

    /// 计算单个回环候选
    void ComputeForCandidate(LoopCandidate& c);

    /// 优化位姿
    void PoseOptimization();

    Options options_;

    Keyframe::Ptr last_kf_ = nullptr;
    Keyframe::Ptr last_loop_kf_ = nullptr;
    Keyframe::Ptr cur_kf_ = nullptr;
    std::vector<Keyframe::Ptr> all_keyframes_;
    std::vector<LoopCandidate> candidates_;

    AsyncMessageProcess<Keyframe::Ptr> kf_thread_;

    std::shared_ptr<miao::Optimizer> optimizer_ = nullptr;

    Mat6d info_motion_ = Mat6d::Identity();  // 关键帧间的运动信息阵
    Mat6d info_loops_ = Mat6d::Identity();   // 回环帧的信息矩阵

    std::vector<std::shared_ptr<miao::VertexSE3>> kf_vert_;
    std::vector<std::shared_ptr<miao::EdgeSE3>> edge_loops_;

    LoopClosedCallback loop_cb_;
};

}  // namespace lightning

#endif  // LIGHTNING_LOOP_CLOSING_H
