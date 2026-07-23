//
// Created by xiang on 24-6-4.
//

#ifndef MIAO_BASE_VEC_VERTEX_H
#define MIAO_BASE_VEC_VERTEX_H

#include "core/graph/base_vertex.h"

namespace lightning::miao {

/**
 * 内部以向量形式存储的顶点
 * 外边只需定义维度即可
 */
template <int D>
class BaseVecVertex : public BaseVertex<D, typename Eigen::Matrix<double, D, 1>> {
   public:
    void OplusImpl(const double *d) override {
        typename Eigen::Matrix<double, D, 1>::ConstMapType v(
            d, BaseVertex<D, typename Eigen::Matrix<double, D, 1>>::Dimension);
        BaseVertex<D, typename Eigen::Matrix<double, D, 1>>::estimate_ += v;
    }
};
}  // namespace lightning::miao

#endif  // MIAO_BASE_VEC_VERTEX_H
