//
// Created by xiang on 24-5-14.
//

#ifndef MIAO_ALGO_SELECT_H
#define MIAO_ALGO_SELECT_H

#include "optimization_algorithm.h"

#include "core/common/config.h"
#include "core/solver/block_solver.h"
#include "core/solver/linear_solver_dense.h"
#include "core/solver/linear_solver_eigen.h"
#include "core/solver/linear_solver_pcg.h"
#include "core/solver/solver.h"

#include "core/opti_algo/opti_algo_dogleg.h"
#include "core/opti_algo/opti_algo_gauss_newton.h"
#include "core/opti_algo/opti_algo_lm.h"

#include "core/graph/optimizer.h"

namespace lightning::miao {

template <int pose_dim = -1, int landmark_dim = -1>
void SetupOptimizer(Optimizer& optimizer, OptimizerConfig options) {
    std::shared_ptr<Solver> solver = nullptr;

    using BlockSolverType = BlockSolver<BlockSolverTraits<pose_dim, landmark_dim>>;

    if (options.is_dense_) {
        if (options.linear_solver_type_ != LinearSolverType::LINEAR_SOLVER_DENSE) {
            LOG(FATAL) << "不能为稠密的问题设置稀疏求解器";
        }

        /// TODO: dense but not Block Solver
        solver = std::make_shared<BlockSolverType>(
            std::make_unique<LinearSolverDense<typename BlockSolverType::PoseMatrixType>>());

    } else {
        if (options.linear_solver_type_ == LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN) {
            solver = std::make_shared<BlockSolverType>(
                std::make_unique<LinearSolverEigen<typename BlockSolverType::PoseMatrixType>>());
        } else if (options.linear_solver_type_ == LinearSolverType::LINEAR_SOLVER_PCG) {
            solver = std::make_shared<BlockSolverType>(
                std::make_unique<LinearSolverPCG<typename BlockSolverType::PoseMatrixType>>());
        }
    }

    std::shared_ptr<OptimizationAlgorithm> algo = nullptr;
    switch (options.algo_type_) {
        case AlgorithmType::GAUSS_NEWTON:
            algo = std::make_shared<OptimizationAlgorithmGaussNewton>(solver);
            break;
        case AlgorithmType::LEVENBERG_MARQUARDT:
            algo = std::make_shared<OptimizationAlgorithmLevenberg>(solver);
            break;
        case AlgorithmType::DOGLEG:
            algo = std::make_shared<OptimizationAlgorithmDogleg>(solver);
            break;
        default:
            LOG(ERROR) << "unknown optimization type";
            break;
    }

    optimizer.SetAlgorithm(algo);
    algo->Init();
    optimizer.SetConfig(options);
}

/// creator func
template <int pose_dim = -1, int landmark_dim = -1>
std::shared_ptr<Optimizer> SetupOptimizer(OptimizerConfig options) {
    auto opti = std::make_shared<Optimizer>();
    SetupOptimizer<pose_dim, landmark_dim>(*opti, options);
    return opti;
}

}  // namespace lightning::miao

#endif  // MIAO_ALGO_SELECT_H
