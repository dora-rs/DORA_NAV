//
// Created by xiang on 24-4-26.
//

#ifndef MIAO_OPTI_ALGO_GAUSS_NEWTON_H
#define MIAO_OPTI_ALGO_GAUSS_NEWTON_H

#include "optimization_algorithm.h"

namespace lightning::miao {

/**
 * \brief Implementation of the Gauss Newton Algorithm
 *
 * 最基本的algothrm 基本啥也没做，只是调solve函数
 */
class OptimizationAlgorithmGaussNewton : public OptimizationAlgorithm {
   public:
    /**
     * construct the Gauss Newton Algorithm, which use the given Solver for
     * solving the linearized system.
     */
    explicit OptimizationAlgorithmGaussNewton(std::shared_ptr<Solver> solver);
    ~OptimizationAlgorithmGaussNewton() override = default;

    SolverResult Solve(int iteration) override;
};

}  // namespace lightning::miao

#endif  // MIAO_OPTI_ALGO_GAUSS_NEWTON_H
