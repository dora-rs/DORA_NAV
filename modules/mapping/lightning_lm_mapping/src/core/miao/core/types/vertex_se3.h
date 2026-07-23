//
// Created by xiang on 24-5-20.
//

#ifndef MIAO_VERTEX_SE3_H
#define MIAO_VERTEX_SE3_H

#include "core/graph/base_vertex.h"

namespace lightning::miao {

/**
 * \brief 3D pose Vertex, represented as an SE3
 *
 * 更新量为6维：t, w，平移在前
 * 更新方程：
 * t = t + Delta t
 * R = R Exp(w)
 */
class VertexSE3 : public BaseVertex<6, SE3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// 覆盖oplus
    void OplusImpl(const double *update) override {
        Eigen::Map<const Vector6> v(update);

        estimate_.translation() += Vec3d(v[0], v[1], v[2]);
        estimate_.so3() = estimate_.so3() * SO3::exp(Vec3d(v[3], v[4], v[5]));
    }
};

}  // namespace lightning::miao

#endif  // MIAO_VERTEX_SE3_H
