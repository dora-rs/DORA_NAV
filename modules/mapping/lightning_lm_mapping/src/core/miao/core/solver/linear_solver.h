//
// Created by xiang on 24-4-26.
//

#ifndef MIAO_LINEAR_SOLVER_H
#define MIAO_LINEAR_SOLVER_H

#include "core/sparse/sparse_block_matrix.h"

namespace lightning::miao {

/// 线性方程求解器的基类
/// 默认就是稀疏矩阵

/**
 * \brief basic Solver for Ax = b
 *
 * basic Solver for Ax = b which has to reimplemented for different linear
 * algebra libraries. A is assumed to be symmetric (only upper triangular Block
 * is stored) and positive-semi-definite.
 */

/// TODO 对于dense的问题，默认的linear solver多一步拷贝
template <typename MatrixType>
class LinearSolver {
   public:
    LinearSolver() = default;
    virtual ~LinearSolver() = default;

    /**
     * Init for operating on matrices with a different non-zero pattern like
     * before
     */
    virtual bool Init() = 0;

    /**
     * Assumes that A is the same matrix for several calls.
     * Among other assumptions, the non-zero pattern does not change!
     * If the matrix changes call Init() before.
     * Solve system Ax = b, x and b have to allocated beforehand!!
     */
    virtual bool Solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b) = 0;

    /**
     * Dense solve version
     * @param A Dense matrix A
     * @param x
     * @param b
     * @return
     */
    virtual bool Solve(const lightning::MatXd& A, double* x, double* b) { return true; }

    /**
     * Convert a Block permutation matrix to a scalar permutation
     */
    template <typename BlockDerived, typename ScalarDerived>
    static void BlockToScalarPermutation(const SparseBlockMatrix<MatrixType>& A,
                                         const Eigen::MatrixBase<BlockDerived>& p,
                                         const Eigen::MatrixBase<ScalarDerived>& scalar /* output */) {
        int n = A.Cols();
        auto& scalarPermutation = const_cast<Eigen::MatrixBase<ScalarDerived>&>(scalar);
        if (scalarPermutation.size() == 0) {
            scalarPermutation.derived().resize(n);
        }

        if (scalarPermutation.size() < n) {
            scalarPermutation.derived().resize(2 * n);
        }
        size_t scalarIdx = 0;

        for (size_t i = 0; i < A.ColBlockIndices().size(); ++i) {
            int base = A.ColBaseOfBlock(p(i));
            int nCols = A.ColsOfBlock(p(i));
            for (int j = 0; j < nCols; ++j) {
                scalarPermutation(scalarIdx++) = base++;
            }
        }
        assert((int)scalarIdx == n);
    }
};

}  // namespace lightning::miao

#endif  // MIAO_LINEAR_SOLVER_H
