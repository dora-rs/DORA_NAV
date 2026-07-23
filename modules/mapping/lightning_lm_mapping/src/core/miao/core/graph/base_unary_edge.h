//
// Created by xiang on 24-4-23.
//

#ifndef MIAO_BASE_UNARY_EDGE_H
#define MIAO_BASE_UNARY_EDGE_H

#include "base_fixed_sized_edge.h"

namespace lightning::miao {

template <int D, typename E, typename VertexXi>
class BaseUnaryEdge : public BaseFixedSizedEdge<D, E, VertexXi> {
   public:
    using VertexXiType = VertexXi;

    BaseUnaryEdge() : BaseFixedSizedEdge<D, E, VertexXi>(){};

   protected:
    typename BaseFixedSizedEdge<D, E, VertexXi>::template JacobianType<D, VertexXi::Dimension> &jacobian_oplus_xi_ =
        std::get<0>(this->jacobian_oplus_);
};

}  // namespace lightning::miao

#endif  // MIAO_BASE_UNARY_EDGE_H
