//
// Created by xiang on 24-4-26.
//

#ifndef MIAO_LINEAR_SOLVER_CCS_H
#define MIAO_LINEAR_SOLVER_CCS_H

#include "core/math/marginal_covariance_cholesky.h"
#include "linear_solver.h"

#include <functional>

namespace lightning::miao {

/**
 * \brief Solver with faster iterating structure for the linear matrix
 */
template <typename MatrixType>
class LinearSolverCCS : public LinearSolver<MatrixType> {
   public:
    LinearSolverCCS() : LinearSolver<MatrixType>() {}
    ~LinearSolverCCS() {}

    //! do the AMD ordering on the blocks or on the scalar matrix
    bool BlockOrdering() const { return block_ordering_; }
    void SetBlockOrdering(bool blockOrdering) { block_ordering_ = blockOrdering; }

   protected:
    void InitMatrixStructure(const SparseBlockMatrix<MatrixType>& A) {
        ccs_matrix_ = std::make_shared<SparseBlockMatrixCCS<MatrixType>>(A.RowBlockIndices(), A.ColBlockIndices());
        A.FillSparseBlockMatrixCCS(*ccs_matrix_);
    }

    std::shared_ptr<SparseBlockMatrixCCS<MatrixType>> ccs_matrix_ = nullptr;
    bool block_ordering_ = true;
};

}  // namespace lightning::miao
#endif  // MIAO_LINEAR_SOLVER_CCS_H
