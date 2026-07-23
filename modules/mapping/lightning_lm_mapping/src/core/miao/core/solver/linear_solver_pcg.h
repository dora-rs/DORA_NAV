//
// Created by xiang on 24-5-13.
//

#ifndef MIAO_LINEAR_SOLVER_PCG_H
#define MIAO_LINEAR_SOLVER_PCG_H

#include "core/math/misc.h"
#include "linear_solver.h"

namespace lightning::miao {

/**
 * \brief linear Solver using PCG, pre-conditioner is Block Jacobi
 */
template <typename MatrixType>
class LinearSolverPCG : public LinearSolver<MatrixType> {
   public:
    LinearSolverPCG() : LinearSolver<MatrixType>() {}

    virtual ~LinearSolverPCG();

    virtual bool Init() {
        residual_ = -1.0;
        indices_by_row_.clear();
        indices_by_col_.clear();
        return true;
    }

    bool Solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b);

   protected:
    using MatrixVector = std::vector<MatrixType>;
    using MatrixPtrVector = std::vector<const MatrixType*>;

    double tolerance_ = cst(1e-6);
    double residual_ = -1.0;
    bool abs_tolerance_ = true;
    int max_iter_ = 100;

    /// A的对角线部分
    MatrixPtrVector diag_;  // A阵的对角线部分
    MatrixVector J_;        // A阵对角线部分之逆

    /// A的非对角线部分
    struct MatAndIdx {
        int idx_ = 0;  // 索引为行时，此处填列的id,索引为列时，此处填行
        MatrixType* mat_;
    };

    std::map<int, std::vector<MatAndIdx>> indices_by_row_;  // 以行为索引的稀疏矩阵块
    std::map<int, std::vector<MatAndIdx>> indices_by_col_;  // 以列为索引的稀疏矩阵块

    void MultDiag(const std::vector<int>& colBlockIndices, MatrixVector& A, const VectorX& src, VectorX& dest);
    void MultDiag(const std::vector<int>& colBlockIndices, MatrixPtrVector& A, const VectorX& src, VectorX& dest);
    void Mult(const std::vector<int>& colBlockIndices, const VectorX& src, VectorX& dest);
};
template <typename MatrixType>
LinearSolverPCG<MatrixType>::~LinearSolverPCG() {}

}  // namespace lightning::miao

#include "linear_solver_pcg.hpp"

#endif  // MIAO_LINEAR_SOLVER_PCG_H
