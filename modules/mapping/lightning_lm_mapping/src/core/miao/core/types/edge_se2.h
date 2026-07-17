//
// Created by xiang on 24-6-17.
//

#ifndef MIAO_EDGE_SE2_H
#define MIAO_EDGE_SE2_H

#include "core/graph/base_binary_edge.h"
#include "core/types/vertex_se2.h"

namespace lightning::miao {

class EdgeSE2 : public BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2> {
   public:
    void ComputeError() override {
        auto* v1 = (VertexSE2*)(vertices_[0]);
        auto* v2 = (VertexSE2*)(vertices_[1]);

        SE2 delta = measurement_.inverse() * (v1->Estimate().inverse() * v2->Estimate());
        error_ = Vec3d(delta.translation().x(), delta.translation().y(), delta.so2().log());
    }

    void LinearizeOplus() override {
        auto* vi = (VertexSE2*)(vertices_[0]);
        auto* vj = (VertexSE2*)(vertices_[1]);
        double thetai = vi->Estimate().so2().log();

        Vec2d dt = vj->Estimate().translation() - vi->Estimate().translation();
        double si = std::sin(thetai), ci = std::cos(thetai);

        jacobian_oplus_xi_ << -ci, -si, -si * dt.x() + ci * dt.y(), si, -ci, -ci * dt.x() - si * dt.y(), 0, 0, -1;

        jacobian_oplus_xj_ << ci, si, 0, -si, ci, 0, 0, 0, 1;

        const SE2& rmean = measurement_.inverse();
        Mat3d z;
        z.block<2, 2>(0, 0) = rmean.so2().matrix();
        z.col(2) << cst(0.), cst(0.), cst(1.);
        z.row(2).head<2>() << cst(0.), cst(0.);

        jacobian_oplus_xi_ = z * jacobian_oplus_xi_;
        jacobian_oplus_xj_ = z * jacobian_oplus_xj_;
    }
};

}  // namespace lightning::miao

#endif  // MIAO_EDGE_SE2_H
