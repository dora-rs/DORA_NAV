//
// Created by xiang on 24-4-24.
//

#ifndef MIAO_OPTIMIZATION_ALGORITHM_H
#define MIAO_OPTIMIZATION_ALGORITHM_H

#include <iostream>
#include <set>
#include <vector>

#include "common/eigen_types.h"
#include "core/common/macros.h"
#include "core/sparse/sparse_block_matrix.h"

namespace lightning::miao {

class Optimizer;
class Vertex;
class Edge;
class Solver;

/**
 * \brief Generic interface for a non-linear Solver operating on a graph
 *
 * Algorithm 的操作界面
 */
class OptimizationAlgorithm {
   public:
    // 求解结果的状态
    enum class SolverResult { Terminate = 2, OK = 1, Fail = -1 };

    OptimizationAlgorithm(std::shared_ptr<Solver> solver) { solver_ = solver; }
    virtual ~OptimizationAlgorithm();

    DISALLOW_COPY(OptimizationAlgorithm)

    /**
     * initialize the Solver, called once before the first call to Solve()
     */
    virtual bool Init();

    /**
     * Solve one iteration. The SparseOptimizer running on-top will call this
     * for the given number of iterations.
     * @param iteration indicates the current iteration
     */
    virtual SolverResult Solve(int iteration) = 0;

    void SetOptimizer(Optimizer* optimizer);

   protected:
    Optimizer* optimizer_ = nullptr;  ///< the optimizer the Solver is working on
    std::shared_ptr<Solver> solver_;  ///< Solver
};

}  // namespace lightning::miao

#endif  // MIAO_OPTIMIZATION_ALGORITHM_H
