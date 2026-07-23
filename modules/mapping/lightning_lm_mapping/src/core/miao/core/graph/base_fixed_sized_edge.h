//
// Created by xiang on 24-4-18.
//

#ifndef MIAO_BASE_FIXED_SIZED_EDGE_H
#define MIAO_BASE_FIXED_SIZED_EDGE_H

#include "base_edge.h"

namespace lightning::miao {

namespace internal {

// creating a bool array
template <int K>
std::array<bool, K> CreateBoolArray() {
    std::array<bool, K> aux = {false};
    return aux;
}
template <>
inline std::array<bool, 0> CreateBoolArray<0>() {
    return std::array<bool, 0>();
}

constexpr int PairToIndex(const int i, const int j) { return j * (j - 1) / 2 + i; }

/**
 * A trivial pair that has a constexpr c'tor.
 * std::pair in C++11 has not a constexpr c'tor.
 */
struct TrivialPair {
    int first_ = 0, second_ = 0;

    constexpr TrivialPair(int f, int s) : first_(f), second_(s) {}

    bool operator==(const TrivialPair &other) const { return first_ == other.first_ && second_ == other.second_; }
};

/**
 * If we would have a constexpr for sqrt (cst_sqrt) it would be sth like.
 * For now we implement as a recursive function at compile time.
 * constexpr TrivialPair IndexToPair(n) {
 *   constexpr int j = int(0.5 + cst_sqrt(0.25 + 2 * n));
 *   constexpr int base = PairToIndex(0, j);
 *   constexpr int i = n - base;
 *   return TrivialPair(i, j);
 * }
 */
constexpr TrivialPair IndexToPair(const int k, const int j = 0) {
    return k < j ? TrivialPair{k, j} : IndexToPair(k - j, j + 1);
}

//! helper function to call the c'tor of Eigen::Map
template <typename T>
T CreateHessianMapK() {
    return T();
}

//! helper function for creating a tuple of Eigen::Map
template <typename... Args>
std::tuple<Args...> CreateHessianMaps(const std::tuple<Args...> &) {
    return std::tuple<Args...>{CreateHessianMapK<Args>()...};
}

template <int I, typename EdgeType, typename... CtorArgs>
typename std::enable_if<I == -1, Vertex *>::type CreateNthVertexType(int, const EdgeType &, CtorArgs...) {
    return nullptr;
}

template <int I, typename EdgeType, typename... CtorArgs>
typename std::enable_if<I != -1, Vertex *>::type CreateNthVertexType(int i, const EdgeType &t, CtorArgs... args) {
    if (i == I) {
        using VertexType = typename EdgeType::template VertexXnType<I>;
        return new VertexType(args...);
    }
    return CreateNthVertexType<I - 1, EdgeType, CtorArgs...>(i, t, args...);
}
}  // namespace internal

/**
 * 编译期确定size的edge,unary, binary, multi edge都可以从这个类中定义
 * 各顶点类型以tuple形式存储，可在编译期获得
 * 真正实用化的edge
 *
 * 但由于VertexTypes是不定的，导致循环必须用tuple展开，写出一大堆魔法模板。。特别是针对jacobian和hessian的展开
 *
 * @tparam D    维度
 * @tparam E    measurement type
 * @tparam VertexTypes  关联的各种顶点
 */
template <int D, typename E, typename... VertexTypes>
class BaseFixedSizedEdge : public BaseEdge<D, E> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// tuple 中第N个元素的type
    template <int N, typename... Types>
    using NthType = typename std::tuple_element<N, std::tuple<Types...>>::type;

    //! The type of the N-th vertex
    /// 第N个顶点的type
    template <int VertexN>
    using VertexXnType = NthType<VertexN, VertexTypes...>;

    //! Size of the N-th vertex at compile time
    /// 第N个顶点的维度
    template <int VertexN>
    static constexpr int VertexDimension() {
        return VertexXnType<VertexN>::Dimension;
    }

    /**
     * Return a pointer to the N-th vertex, directly casted to the correct type
     * 第N个顶点的指针，转换到子类的指针（shared_ptr）
     */
    template <int VertexN>
    VertexXnType<VertexN> *VertexXn() const {
        return (VertexXnType<VertexN> *)(vertices_[VertexN]);
    }

    static const int Dimension = BaseEdge<D, E>::Dimension;
    using Measurement = typename BaseEdge<D, E>::Measurement;
    using ErrorVector = typename BaseEdge<D, E>::ErrorVector;
    using InformationType = typename BaseEdge<D, E>::InformationType;

    template <int EdgeDimension, int VertexDimension>
    using JacobianType = typename Eigen::Matrix<double, EdgeDimension, VertexDimension>;

    template <int DN, int DM>
    struct HessianType {
        double *hessian_in_solver_ = nullptr;    // 远程地址
        Eigen::Matrix<double, DN, DM> hessian_;  // 本地地址
    };

    //! it requires quite some ugly code to get the type of hessians...
    template <int DN, int DM>
    using HessianBlockType = HessianType<DN, DM>;

    template <int K>
    using HessianBlockTypeK = HessianBlockType<VertexXnType<internal::IndexToPair(K).first_>::Dimension,
                                               VertexXnType<internal::IndexToPair(K).second_>::Dimension>;
    template <int K>
    using HessianBlockTypeKTransposed = HessianBlockType<VertexXnType<internal::IndexToPair(K).second_>::Dimension,
                                                         VertexXnType<internal::IndexToPair(K).first_>::Dimension>;
    template <typename>
    struct HessianTupleType;

    template <std::size_t... Ints>
    struct HessianTupleType<std::index_sequence<Ints...>> {
        using type = std::tuple<HessianBlockTypeK<Ints>...>;
        using typeTransposed = std::tuple<HessianBlockTypeKTransposed<Ints>...>;
    };

    static const std::size_t nr_of_vertices_ = sizeof...(VertexTypes);
    static const std::size_t nr_of_vertex_pairs_ = internal::PairToIndex(0, nr_of_vertices_);

    using HessianTuple = typename HessianTupleType<std::make_index_sequence<nr_of_vertex_pairs_>>::type;

    using HessianTupleTransposed =
        typename HessianTupleType<std::make_index_sequence<nr_of_vertex_pairs_>>::typeTransposed;

    using HessianRowMajorStorage = std::array<bool, nr_of_vertex_pairs_>;

    BaseFixedSizedEdge()
        : BaseEdge<D, E>(),
          hessian_row_major_(internal::CreateBoolArray<nr_of_vertex_pairs_>()),
          hessian_tuple_(internal::CreateHessianMaps(hessian_tuple_)),
          hessian_tuple_trans_(internal::CreateHessianMaps(hessian_tuple_trans_)) {
        vertices_.resize(nr_of_vertices_);
    }
    ~BaseFixedSizedEdge() override = default;

    template <std::size_t... Ints>
    bool AllVerticesFixedNs(std::index_sequence<Ints...>) const;

    bool AllVerticesFixed() const override;

    /**
     * Linearizes the oplus operator in the vertex, and stores
     * the result in temporary variables _jacobianOplus
     *
     * 默认的linearize oplus，使用数值求导
     */
    void LinearizeOplus() override;

    template <std::size_t... Ints>
    void LinearizeOplusNs(std::index_sequence<Ints...>);

    template <int N>
    void LinearizeOplusN();

    /**
     * computes the (Block) elements of the Hessian matrix of the linearized least
     * squares.
     *
     * 计算二次型部分，包括hessian(AtA)和b
     */
    void ConstructQuadraticForm() override;

    /// 拷贝hessian至solver
    void CopyHessianToSolver() override;

    template <int N>
    void CopyHessianToSolverN();

    template <std::size_t... Ints>
    void CopyHessianToSolverNs(std::index_sequence<Ints...>);

    template <int N, std::size_t... Ints>
    void CopyEdgeHessianMs(std::index_sequence<Ints...>);

    template <int N, int M>
    void CopyEdgeHessianM();

    /// 主对角线部分
    template <std::size_t... Ints>
    void ConstructQuadraticFormNs(const InformationType &omega, const ErrorVector &weightedError,
                                  std::index_sequence<Ints...>);

    template <int N>
    void ConstructQuadraticFormN(const InformationType &omega, const ErrorVector &weightedError);

    /// 非主对角线部分
    template <int N, std::size_t... Ints, typename AtOType>
    void ConstructOffDiagonalQuadraticFormMs(const AtOType &AtO, std::index_sequence<Ints...>);

    template <int N, int M, typename AtOType>
    void ConstructOffDiagonalQuadraticFormM(const AtOType &AtO);

    void MapHessianMemory(double *d, int i, int j, bool rowMajor) override;

    using BaseEdge<D, E>::Resize;
    using BaseEdge<D, E>::ComputeError;

   protected:
    using BaseEdge<D, E>::measurement_;
    using BaseEdge<D, E>::information_;
    using BaseEdge<D, E>::error_;
    using BaseEdge<D, E>::vertices_;
    using BaseEdge<D, E>::dimension_;

    HessianRowMajorStorage hessian_row_major_;
    HessianTuple hessian_tuple_;
    HessianTupleTransposed hessian_tuple_trans_;

    std::tuple<JacobianType<D, VertexTypes::Dimension>...> jacobian_oplus_;  // 以tuple存储的jacobian，注意这是个map
};

}  // namespace lightning::miao

#include "base_fixed_sized_edge.hpp"

#endif  // MIAO_BASE_FIXED_SIZED_EDGE_H
