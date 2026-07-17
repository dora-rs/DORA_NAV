//
// Created by xiang on 24-6-17.
//

#ifndef MIAO_VERTEX_SE2_H
#define MIAO_VERTEX_SE2_H

#include "core/graph/base_vertex.h"

namespace lightning::miao {

class VertexSE2 : public BaseVertex<3, SE2> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// 覆盖oplus
    void OplusImpl(const double *update) override {
        Eigen::Map<const Vector6> v(update);

        estimate_.translation() += Vec2d(v[0], v[1]);
        estimate_.so2() = SO2(estimate_.so2().log() + v[2]);
    }
};

}  // namespace lightning::miao

#endif  // MIAO_VERTEX_SE2_H
