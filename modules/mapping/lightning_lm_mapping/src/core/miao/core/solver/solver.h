//
// Created by xiang on 24-4-25.
//

#ifndef MIAO_SOLVER_H
#define MIAO_SOLVER_H

#include "common/eigen_types.h"
#include "core/graph/graph.h"
#include "core/sparse/sparse_block_matrix.h"

namespace lightning::miao {

class Optimizer;

/**
 * \brief Generic interface for a sparse Solver operating on a graph which
 * solves one iteration of the linearized objective function
 *
 * Solver 接口类
 * Solver 的主要继承类是block solver,由block solver解出具体的H, b的矩阵块
 * 由optimization Algorithm 来调用各种成员函数
 *
 * Block Solver 矩阵块大小在编译期已知
 * Block Solver 进一步持有linear solver的指针，从而求解Hx = GetB
 */
class Solver {
   public:
    Solver();
    virtual ~Solver();

    DISALLOW_COPY(Solver)

   public:
    /**
     * initialize the Solver, called once before the first iteration
     */
    virtual bool Init(Optimizer* optimizer) = 0;

    /**
     * build the structure of the system
     */
    virtual bool BuildStructure(bool zeroBlocks = false) = 0;

    /**
     * build the current system
     */
    virtual bool BuildSystem() = 0;

    /**
     * Solve Ax = GetB
     * 实际由持有的linear solver执行
     */
    virtual bool Solve() = 0;

    /**
     * compute dest = H * src
     */
    virtual void MultiplyHessian(double* dest, const double* src) const = 0;

    /**
     * Update the system while performing Levenberg, i.e., modifying the Diagonal
     * components of A by doing += lambda along the main Diagonal of the Matrix.
     * Note that this function may be called with a positive and a negative
     * lambda. The latter is used to undo a former modification. If backup is
     * true, then the Solver should store a backup of the Diagonal, which can be
     * restored by RestoreDiagonal()
     */
    virtual bool SetLambda(double lambda, bool backup = false) = 0;

    /**
     * restore a previously made backup of the Diagonal
     */
    virtual void RestoreDiagonal() = 0;

    //! return x, the solution vector
    double* GetX() { return x_.data(); }
    lightning::VecXd GetXVec() { return x_; }
    const double* GetX() const { return x_.data(); }
    //! return GetB, the right hand side of the system
    double* GetB() { return b_.data(); }
    const double* GetB() const { return b_.data(); }

    //! return the size of the solution vector (GetX) and GetB
    size_t VectorSize() const { return x_size_; }

    /**
     * does this Solver support the Schur complement for solving a system
     * consisting of poses and landmarks. Re-implement in a derived Solver, if
     * your Solver supports it.
     */
    virtual bool SupportsSchur() { return false; }

    //! should the Solver perform the Schur complement or not
    virtual bool Schur() = 0;
    virtual void SetSchur(bool s) = 0;

   protected:
    Optimizer* optimizer_ = nullptr;
    lightning::VecXd x_;                  // 求解的x
    lightning::VecXd b_;                  // 求解的b
    size_t x_size_ = 0, max_x_size_ = 0;  // x的size
    bool is_levenberg_ = false;           ///< the system we gonna Solve is a Levenberg-Marquardt system
    size_t additional_vector_space_ = 0;

    void ResizeVector(size_t sx);
};

}  // namespace lightning::miao

#endif  // MIAO_SOLVER_H
