//
// Created by xiang on 25-7-14.
//

#ifndef MIAO_EDGE_SE3_PRIOR_H
#define MIAO_EDGE_SE3_PRIOR_H

#include "core/common/math.h"
#include "core/graph/base_unary_edge.h"
#include "core/types/vertex_se3.h"

namespace lightning::miao {

/**
 * 先验的SE3观测
 *
 * 误差平移在前（与g2o保持一致，否则pose graph的info就不对）
 * 误差(旋转) = T.trans - obs.trans
 * 误差(平移) = T.R^T * obs.R
 *
 */
class EdgeSE3Prior : public BaseUnaryEdge<6, SE3, VertexSE3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// 为了读g2o文件，这里的算法必须与g2o一致。

    void ComputeError() override {
        SE3 pose = ((VertexSE3*)(vertices_[0]))->Estimate();
        error_.head<3>() = pose.translation() - measurement_.translation();
        error_.tail<3>() = (pose.so3().inverse() * measurement_.so3()).log();
    }

    // TODO: jacobian计算
};

}  // namespace lightning::miao
#endif  // LIGHTNING_EDGE_SE3_PRIOR_H
