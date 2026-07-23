//
// Created by xiang on 24-5-13.
//

#ifndef MIAO_OPTI_ALGO_DOGLEG_H
#define MIAO_OPTI_ALGO_DOGLEG_H

#include "optimization_algorithm.h"

namespace lightning::miao {

/**
 * \brief Implementation of Powell's Dogleg Algorithm
 */
class OptimizationAlgorithmDogleg : public OptimizationAlgorithm {
   public:
    /**
     * construct the Dogleg Algorithm, which will use the given Solver for solving
     * the linearized system.
     */
    explicit OptimizationAlgorithmDogleg(std::shared_ptr<Solver> solver);
    ~OptimizationAlgorithmDogleg() override;

    virtual SolverResult Solve(int iteration) override;

   protected:
    // parameters
    int max_trail_after_failure_ = 100;
    double user_delta_init_ = 1e4;
    // damping to enforce positive definite matrix
    double init_lambda_ = 1e-7;
    double lambda_factor_ = 10.0;

    lightning::VectorX hsd_;         ///< steepest decent step
    lightning::VectorX hdl_;         ///< final dogleg step
    lightning::VectorX aux_vector_;  ///< auxiliary vector used to perform multiplications or other stuff

    double current_lambda_ = 0.0;  ///< the damping factor to force positive definite matrix
    double delta_ = 1e4;           ///< trust region
    bool was_pd_in_all_iterations_ =
        true;  ///< the matrix we Solve was positive definite in all iterations -> if not apply damping
    int last_num_tries_ = 0;
};
}  // namespace lightning::miao

#endif  // MIAO_OPTI_ALGO_DOGLEG_H
