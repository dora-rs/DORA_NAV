//
// Created by xiang on 24-5-8.
//

#ifndef BASE_FIXED_SIZED_EDGE_HPP
#define BASE_FIXED_SIZED_EDGE_HPP

#include "base_vertex.h"
#include "core/math/misc.h"
#include "core/robust_kernel/robust_kernel.h"
#include "utils/tuple_tools.h"
#include "vertex.h"

#include <glog/logging.h>

namespace lightning::miao {

template <int D, typename E, typename... VertexTypes>
template <int N>
void BaseFixedSizedEdge<D, E, VertexTypes...>::LinearizeOplusN() {
    auto vertex = VertexXn<N>();
    if (vertex->Fixed()) {
        return;
    }

    // 为了并发，现在需要拷贝这个顶点的估计值
    // auto vertex_copy = vertex;
    auto vertex_copy = std::make_shared<VertexXnType<N>>(*vertex);
    vertices_[N] = vertex_copy.get();

    auto &jacobianOplus = std::get<N>(jacobian_oplus_);

    const double delta = cst(1e-9);
    const double scalar = 1 / (2 * delta);

    // estimate the jacobian numerically

    double add_vertex[VertexDimension<N>()] = {};

    // add small step along the unit vector in each dimension
    for (int d = 0; d < VertexDimension<N>(); ++d) {
        vertex_copy->Push();
        add_vertex[d] = delta;
        vertex_copy->Oplus(add_vertex);

        ComputeError();
        auto errorBak = this->error_;
        vertex_copy->Pop();

        vertex_copy->Push();
        add_vertex[d] = -delta;
        vertex_copy->Oplus(add_vertex);
        ComputeError();

        errorBak -= this->error_;
        vertex_copy->Pop();
        add_vertex[d] = 0.0;

        jacobianOplus.col(d) = scalar * errorBak;
    }

    /// recover the copied vertex
    vertices_[N] = vertex;
}

template <int D, typename E, typename... VertexTypes>
template <std::size_t... Ints>
void BaseFixedSizedEdge<D, E, VertexTypes...>::LinearizeOplusNs(std::index_sequence<Ints...>) {
    (void(LinearizeOplusN<Ints>()), ...);
}

template <int D, typename E, typename... VertexTypes>
void BaseFixedSizedEdge<D, E, VertexTypes...>::LinearizeOplus() {
    if (AllVerticesFixed()) {
        return;
    }

    ErrorVector errorBeforeNumeric = error_;
    LinearizeOplusNs(std::make_index_sequence<nr_of_vertices_>());
    error_ = errorBeforeNumeric;
}

template <int D, typename E, typename... VertexTypes>
template <std::size_t... Ints>
bool BaseFixedSizedEdge<D, E, VertexTypes...>::AllVerticesFixedNs(std::index_sequence<Ints...>) const {
    return (... && VertexXn<Ints>()->Fixed());
}

template <int D, typename E, typename... VertexTypes>
bool BaseFixedSizedEdge<D, E, VertexTypes...>::AllVerticesFixed() const {
    return AllVerticesFixedNs(std::make_index_sequence<nr_of_vertices_>());
}

template <int D, typename E, typename... VertexTypes>
void BaseFixedSizedEdge<D, E, VertexTypes...>::CopyHessianToSolver() {
    CopyHessianToSolverNs(std::make_index_sequence<nr_of_vertices_>());
}

template <int D, typename E, typename... VertexTypes>
template <std::size_t... Ints>
void BaseFixedSizedEdge<D, E, VertexTypes...>::CopyHessianToSolverNs(std::index_sequence<Ints...>) {
    (void(CopyHessianToSolverN<Ints>()), ...);
}

template <int D, typename E, typename... VertexTypes>
template <int N>
void BaseFixedSizedEdge<D, E, VertexTypes...>::CopyHessianToSolverN() {
    CopyEdgeHessianMs<N>(std::make_index_sequence<nr_of_vertices_ - N - 1>());
}

template <int D, typename E, typename... VertexTypes>
template <int N, std::size_t... Ints>
void BaseFixedSizedEdge<D, E, VertexTypes...>::CopyEdgeHessianMs(std::index_sequence<Ints...>) {
    (void(CopyEdgeHessianM<N, Ints>()), ...);
}

template <int D, typename E, typename... VertexTypes>
template <int N, int M>
void BaseFixedSizedEdge<D, E, VertexTypes...>::CopyEdgeHessianM() {
    constexpr auto fromId = N;
    constexpr auto toId = N + M + 1;
    assert(fromId < toId && "Index mixed up");
    constexpr auto K = internal::PairToIndex(fromId, toId);

    if (hessian_row_major_[K]) {
        auto &h = std::get<K>(hessian_tuple_trans_);
        if (h.hessian_in_solver_ == nullptr) {
            return;
        }
        memcpy(h.hessian_in_solver_, h.hessian_.data(), sizeof(double) * h.hessian_.rows() * h.hessian_.cols());
        h.hessian_.setZero();
    } else {
        auto &h = std::get<K>(hessian_tuple_);
        if (h.hessian_in_solver_ == nullptr) {
            return;
        }
        memcpy(h.hessian_in_solver_, h.hessian_.data(), sizeof(double) * h.hessian_.rows() * h.hessian_.cols());
        h.hessian_.setZero();
    }
}

template <int D, typename E, typename... VertexTypes>
void BaseFixedSizedEdge<D, E, VertexTypes...>::ConstructQuadraticForm() {
    if (this->GetRobustKernel()) {
        double error = this->Chi2();
        Vector3 rho;
        this->GetRobustKernel()->Robustify(error, rho);
        Eigen::Matrix<double, D, 1> omega_r = -information_ * error_;
        omega_r *= rho[1];
        ConstructQuadraticFormNs(this->RobustInformation(rho), omega_r, std::make_index_sequence<nr_of_vertices_>());
    } else {
        ConstructQuadraticFormNs(information_, -information_ * error_, std::make_index_sequence<nr_of_vertices_>());
    }
}

template <int D, typename E, typename... VertexTypes>
template <std::size_t... Ints>
void BaseFixedSizedEdge<D, E, VertexTypes...>::ConstructQuadraticFormNs(const InformationType &omega,
                                                                        const ErrorVector &weightedError,
                                                                        std::index_sequence<Ints...>) {
    (void(ConstructQuadraticFormN<Ints>(omega, weightedError)), ...);
}

template <int D, typename E, typename... VertexTypes>
template <int N>
void BaseFixedSizedEdge<D, E, VertexTypes...>::ConstructQuadraticFormN(const InformationType &omega,
                                                                       const ErrorVector &weightedError) {
    auto from = VertexXn<N>();
    const auto &A = std::get<N>(jacobian_oplus_);

    if (from->Fixed()) {
        return;
    }

    const auto AtO = A.transpose() * omega;

    // from->GetB().noalias() += A.transpose() * weightedError;
    // from->A().noalias() += AtO * A;

    from->UpdateHessianAndBias(AtO * A, A.transpose() * weightedError);

    ConstructOffDiagonalQuadraticFormMs<N>(AtO, std::make_index_sequence<nr_of_vertices_ - N - 1>());
};

template <int D, typename E, typename... VertexTypes>
template <int N, std::size_t... Ints, typename AtOType>
void BaseFixedSizedEdge<D, E, VertexTypes...>::ConstructOffDiagonalQuadraticFormMs(const AtOType &AtO,
                                                                                   std::index_sequence<Ints...>) {
    (void(ConstructOffDiagonalQuadraticFormM<N, Ints, AtOType>(AtO)), ...);
}

template <int D, typename E, typename... VertexTypes>
template <int N, int M, typename AtOType>
void BaseFixedSizedEdge<D, E, VertexTypes...>::ConstructOffDiagonalQuadraticFormM(const AtOType &AtO) {
    constexpr auto fromId = N;
    constexpr auto toId = N + M + 1;
    assert(fromId < toId && "Index mixed up");
    auto to = VertexXn<toId>();
    if (to->Fixed()) {
        return;
    }

    const auto &B = std::get<toId>(jacobian_oplus_);
    constexpr auto K = internal::PairToIndex(fromId, toId);

    if (hessian_row_major_[K]) {
        // we have to write to the block as transposed
        auto &hessianTransposed = std::get<K>(hessian_tuple_trans_);
        hessianTransposed.hessian_.noalias() += B.transpose() * AtO.transpose();
    } else {
        auto &hessian = std::get<K>(hessian_tuple_);
        hessian.hessian_.noalias() += AtO * B;
    }
}

/**
 * Helper functor class to construct the Hessian Eigen::Map object.
 * We have to pass the size at runtime to allow dynamically sized verices.
 */
struct MapHessianMemoryK {
    double *d;
    int rows;
    int cols;

    template <typename HessianT>
    void operator()(HessianT &hessian) {
        hessian.hessian_in_solver_ = d;
        hessian.hessian_.setZero();
        // new (&hessian) typename std::remove_reference<decltype(hessian)>::type(d, rows, cols);
    }
};

template <int D, typename E, typename... VertexTypes>
void BaseFixedSizedEdge<D, E, VertexTypes...>::MapHessianMemory(double *d, int i, int j, bool rowMajor) {
    assert(i < j && "index assumption violated");
    // get the size of the vertices
    int vi_dim = Edge::GetVertex(i)->Dimension();
    int vj_dim = Edge::GetVertex(j)->Dimension();

    int k = internal::PairToIndex(i, j);
    hessian_row_major_[k] = rowMajor;

    if (rowMajor) {
        tuple_apply_i(MapHessianMemoryK{d, vj_dim, vi_dim}, hessian_tuple_trans_, k);
    } else {
        tuple_apply_i(MapHessianMemoryK{d, vi_dim, vj_dim}, hessian_tuple_, k);
    }
}

}  // namespace lightning::miao

#endif  // BASE_FIXED_SIZED_EDGE_HPP
