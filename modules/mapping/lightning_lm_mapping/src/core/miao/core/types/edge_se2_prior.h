//
// Created by xiang on 24-7-1.
//

#ifndef MIAO_EDGE_SE2_PRIOR_H
#define MIAO_EDGE_SE2_PRIOR_H

#include "core/graph/base_unary_edge.h"
#include "core/types/vertex_se2.h"

namespace lightning::miao {

class EdgeSE2Prior : public BaseUnaryEdge<3, SE2, VertexSE2> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void ComputeError() override {
        const VertexSE2* v1 = static_cast<const VertexSE2*>(vertices_[0]);
        SE2 delta = measurement_.inverse() * v1->Estimate();
        error_ = Vec3d(delta.translation().x(), delta.translation().y(), delta.so2().log());
    }

    void LinearizeOplus() override {
        jacobian_oplus_xi_.setZero();
        jacobian_oplus_xi_.block<2, 2>(0, 0) = measurement_.inverse().so2().matrix();
        jacobian_oplus_xi_(2, 2) = 1;
    }
};

}  // namespace lightning::miao

#endif  // MIAO_EDGE_SE2_PRIOR_H
