#include <cassert>

namespace lightning::miao {
namespace internal {

/**
 * y=Ax
 */
template <typename MatrixType>
inline void pcg_axy(const MatrixType& A, const VectorX& x, int xoff, VectorX& y, int yoff) {
    y.segment<MatrixType::RowsAtCompileTime>(yoff) = A * x.segment<MatrixType::ColsAtCompileTime>(xoff);
}

template <>
inline void pcg_axy(const MatrixX& A, const VectorX& x, int xoff, VectorX& y, int yoff) {
    y.segment(yoff, A.rows()) = A * x.segment(xoff, A.cols());
}

template <typename MatrixType>
inline void pcg_axpy(const MatrixType& A, const VectorX& x, int xoff, VectorX& y, int yoff) {
    y.segment<MatrixType::RowsAtCompileTime>(yoff) += A * x.segment<MatrixType::ColsAtCompileTime>(xoff);
}

template <>
inline void pcg_axpy(const MatrixX& A, const VectorX& x, int xoff, VectorX& y, int yoff) {
    y.segment(yoff, A.rows()) += A * x.segment(xoff, A.cols());
}

template <typename MatrixType>
inline void pcg_atxpy(const MatrixType& A, const VectorX& x, int xoff, VectorX& y, int yoff) {
    y.segment<MatrixType::ColsAtCompileTime>(yoff) += A.transpose() * x.segment<MatrixType::RowsAtCompileTime>(xoff);
}

template <>
inline void pcg_atxpy(const MatrixX& A, const VectorX& x, int xoff, VectorX& y, int yoff) {
    y.segment(yoff, A.cols()) += A.transpose() * x.segment(xoff, A.rows());
}
}  // namespace internal
// helpers end

template <typename MatrixType>
bool LinearSolverPCG<MatrixType>::Solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b) {
    const bool indexRequired = indices_by_row_.size() == 0;
    diag_.clear();
    J_.clear();

    // put the block matrix once in a linear structure, makes mult faster
    int colIdx = 0;
    for (size_t i = 0; i < A.BlockCols().size(); ++i) {
        auto& col = A.BlockCols()[i];
        if (col.size() > 0) {
            for (auto it = col.begin(); it != col.end(); ++it) {
                if (it->first == (int)i) {  // only the upper triangular block is needed
                    diag_.push_back(it->second);
                    J_.push_back(it->second->inverse());
                    break;
                }

                if (indexRequired) {
                    int row_idx = it->first > 0 ? A.RowBlockIndices()[it->first - 1] : 0;
                    MatAndIdx m;
                    m.idx_ = colIdx;
                    m.mat_ = it->second;

                    auto iter = indices_by_row_.find(row_idx);
                    if (iter == indices_by_row_.end()) {
                        std::vector<MatAndIdx> data{m};
                        indices_by_row_.emplace(row_idx, data);
                    } else {
                        iter->second.emplace_back(m);
                    }

                    iter = indices_by_col_.find(colIdx);
                    m.idx_ = row_idx;
                    if (iter == indices_by_col_.end()) {
                        std::vector<MatAndIdx> data{m};
                        indices_by_col_.emplace(colIdx, data);
                    } else {
                        iter->second.emplace_back(m);
                    }
                }
            }
        }
        colIdx = A.ColBlockIndices()[i];
    }

    int n = A.Rows();
    assert(n > 0 && "Hessian has 0 Rows/Cols");
    Eigen::Map<VectorX> xvec(x, A.Cols());
    const Eigen::Map<VectorX> bvec(b, n);
    xvec.setZero();

    VectorX r, pk, q, s;
    pk.setZero(n);
    q.setZero(n);
    s.setZero(n);
    r = bvec;

    MultDiag(A.ColBlockIndices(), J_, r, pk);
    double dn = r.dot(pk);
    double d0 = tolerance_ * dn;

    if (abs_tolerance_) {
        if (residual_ > 0.0 && residual_ > d0) {
            d0 = residual_;
        }
    }

    int maxIter = max_iter_ < 0 ? A.Rows() : max_iter_;

    int iteration;
    std::vector<double> dns;
    double last_dn = 0.0;

    for (iteration = 0; iteration < maxIter; ++iteration) {
        if (dn <= d0 || dn < 1e-8) {
            break;  // done
        }

        /// qk = A pk
        Mult(A.ColBlockIndices(), pk, q);

        /// alpha =  rk^T pk / (pk^T A pk)
        double alpha = dn / pk.dot(q);

        /// x_k+1 = x_k + alpha pk
        xvec += alpha * pk;

        // TODO: reset residual here every 50 iterations
        /// r_{k+1} = r_k + alpha p_k
        r -= alpha * q;

        /// s = Lambda pk
        MultDiag(A.ColBlockIndices(), J_, r, s);

        double dold = dn;

        /// dn = r_{k+1}^T A p_k
        dn = r.dot(s);

        if (iteration >= 0.1 * maxIter && dn >= 1.1 * last_dn) {
            break;
        }

        /// beta = r_{k+1}^T A p_k / (pk^T A p_k)
        double beta = dn / dold;

        if (iteration % 50 == 0) {
            beta = 0;
        }

        /// pk+1 = -r_k+1 + beta pk
        pk = s + beta * pk;

        dns.emplace_back(dn);

        last_dn = dn;
    }

    residual_ = 0.5 * dn;

    return true;
}

template <typename MatrixType>
void LinearSolverPCG<MatrixType>::MultDiag(const std::vector<int>& colBlockIndices, MatrixVector& A, const VectorX& src,
                                           VectorX& dest) {
    int row = 0;
    for (size_t i = 0; i < A.size(); ++i) {
        internal::pcg_axy(A[i], src, row, dest, row);
        row = colBlockIndices[i];
    }
}

template <typename MatrixType>
void LinearSolverPCG<MatrixType>::MultDiag(const std::vector<int>& colBlockIndices, MatrixPtrVector& A,
                                           const VectorX& src, VectorX& dest) {
    int row = 0;
    for (size_t i = 0; i < A.size(); ++i) {
        internal::pcg_axy(*A[i], src, row, dest, row);
        row = colBlockIndices[i];
    }
}

template <typename MatrixType>
void LinearSolverPCG<MatrixType>::Mult(const std::vector<int>& colBlockIndices, const VectorX& src, VectorX& dest) {
    // first multiply with the diagonal
    // A的主对角线部分已经乘完了
    MultDiag(colBlockIndices, diag_, src, dest);

    // now multiply with the upper triangular block
    // NOTE: 这个维度太小的时候，多线程反而不划算
    if (indices_by_row_.size() >= 200) {
        /// 上半角部分
        std::for_each(std::execution::par_unseq, indices_by_row_.begin(), indices_by_row_.end(), [&](const auto& d) {
            const int& destOffset = d.first;
            for (const MatAndIdx& m : d.second) {
                const int& srcOffset = m.idx_;
                const auto& a = m.mat_;
                internal::pcg_axpy(*a, src, srcOffset, dest, destOffset);
            }
        });

        /// 下半角部分
        std::for_each(std::execution::par_unseq, indices_by_col_.begin(), indices_by_col_.end(), [&](const auto& d) {
            const int& destOffsetT = d.first;
            for (const MatAndIdx& m : d.second) {
                const int& srcOffsetT = m.idx_;
                const auto& a = m.mat_;
                internal::pcg_atxpy(*a, src, srcOffsetT, dest, destOffsetT);
            }
        });

    } else {
        /// 串行计算时不需要再区分上下半区
        std::for_each(std::execution::seq, indices_by_row_.begin(), indices_by_row_.end(), [&](const auto& d) {
            for (const MatAndIdx& m : d.second) {
                const int& srcOffset = m.idx_;
                const int& destOffsetT = srcOffset;
                const int& destOffset = d.first;
                const int& srcOffsetT = destOffset;

                const auto& a = m.mat_;
                // destVec += *a * srcVec (according to the sub-vector parts)
                internal::pcg_axpy(*a, src, srcOffset, dest, destOffset);
                // destVec += *a.transpose() * srcVec (according to the sub-vector parts)
                internal::pcg_atxpy(*a, src, srcOffsetT, dest, destOffsetT);
            }
        });
    }
}
}  // namespace lightning::miao
