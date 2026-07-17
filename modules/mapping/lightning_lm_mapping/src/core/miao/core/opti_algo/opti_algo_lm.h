//
// Created by xiang on 24-5-13.
//

#ifndef MIAO_OPTI_ALGO_LM_H
#define MIAO_OPTI_ALGO_LM_H

#include "optimization_algorithm.h"

namespace lightning::miao {

/**
 * \brief Implementation of the Levenberg Algorithm
 */
class OptimizationAlgorithmLevenberg : public OptimizationAlgorithm {
   public:
    /**
     * construct the Levenberg Algorithm, which will use the given Solver for
     * solving the linearized system.
     */
    explicit OptimizationAlgorithmLevenberg(std::shared_ptr<Solver> solver);

    virtual ~OptimizationAlgorithmLevenberg() = default;

    virtual SolverResult Solve(int iteration);

   protected:
    /**
     * helper for Levenberg, this function computes the initial damping factor, if
     * the user did not specify an own value, see setUserLambdaInit()
     */
    double ComputeLambdaInit() const;

    double ComputeScale() const;

    // Levenberg parameters
    int max_trails_after_failure_ = 10;
    double user_lambda_limit_ = 0;
    double current_lambda_ = -1;
    double tau_ = 1e-5;
    double good_step_lower_ = 1.0 / 3.0;  ///< lower bound for lambda decrease if a good  LM step
    double good_step_upper_ = 2.0 / 3.0;  ///< upper bound for lambda decrease if a good  LM step
    double ni_ = 2.0;
    int lm_iter_ = 0;  ///< the number of levenberg iterations performed to accept the last step
};

}  // namespace lightning::miao

#endif  // MIAO_OPTI_ALGO_LM_H
