//
// Created by xiang on 24-4-28.
//

#ifndef MIAO_LINEAR_SOLVER_DENSE_H
#define MIAO_LINEAR_SOLVER_DENSE_H

#include "linear_solver.h"

namespace lightning::miao {

/**
 * \brief linear Solver using dense cholesky decomposition
 */
template <typename MatrixType = MatrixX>
class LinearSolverDense : public LinearSolver<MatrixType> {
   public:
    LinearSolverDense() : LinearSolver<MatrixType>() {}

    virtual ~LinearSolverDense() = default;

    bool Init() override {
        reset_ = true;
        return true;
    }

    bool Solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b) override {
        int n = A.Cols();
        int m = A.Cols();

        MatrixX& H = H_;
        if (H.cols() != n) {
            H.resize(n, m);
            reset_ = true;
        }
        if (reset_) {
            reset_ = false;
            H.setZero();
        }

        // copy the sparse block matrix into a dense matrix
        int c_idx = 0;
        for (size_t i = 0; i < A.BlockCols().size(); ++i) {
            int c_size = A.ColsOfBlock(i);
            assert(c_idx == A.ColBaseOfBlock(i) && "mismatch in Block indices");

            const typename SparseBlockMatrix<MatrixType>::IntBlockMap& col = A.BlockCols()[i];
            if (col.size() > 0) {
                typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it;

                for (it = col.begin(); it != col.end(); ++it) {
                    int r_idx = A.RowBaseOfBlock(it->first);
                    // only the upper triangular block is processed
                    if (it->first <= (int)i) {
                        int r_size = A.RowsOfBlock(it->first);
                        H.block(r_idx, c_idx, r_size, c_size) = *(it->second);

                        if (r_idx != c_idx) {
                            // write the lower triangular block
                            H.block(c_idx, r_idx, c_size, r_size) = it->second->transpose();
                        }
                    }
                }
            }

            c_idx += c_size;
        }

        // solving via Cholesky decomposition
        VectorX::MapType xvec(x, m);
        VectorX::ConstMapType bvec(b, n);
        cholesky_.compute(H);
        if (cholesky_.isPositive()) {
            xvec = cholesky_.solve(bvec);
            return true;
        }
        return false;
    }

   protected:
    bool reset_ = true;
    MatrixX H_;
    Eigen::LDLT<MatrixX> cholesky_;
};

}  // namespace lightning::miao

#endif  // MIAO_LINEAR_SOLVER_DENSE_H
