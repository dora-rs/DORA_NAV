//
// Created by xiang on 24-5-13.
//

#include "opti_algo_lm.h"
#include "core/graph/edge.h"
#include "core/graph/optimizer.h"
#include "core/graph/vertex.h"
#include "core/solver/solver.h"

#include <glog/logging.h>
#include <cassert>
#include <cmath>
#include <iostream>
#include <utility>

#include "utils/timer.h"

namespace lightning::miao {

OptimizationAlgorithmLevenberg::OptimizationAlgorithmLevenberg(std::shared_ptr<Solver> solver)
    : OptimizationAlgorithm(std::move(solver)), lm_iter_(0) {}

OptimizationAlgorithm::SolverResult OptimizationAlgorithmLevenberg::Solve(int iteration) {
    assert(optimizer_ && "_optimizer not set");

    if (iteration == 0) {  // built up the CCS structure, here due to easy time measure
        bool ok = false;
        ok = solver_->BuildStructure();

        if (!ok) {
            LOG(WARNING) << "Failure while building CCS structure";
            return SolverResult::Fail;
        }
    }

    optimizer_->ComputeActiveErrors();
    double currentChi = optimizer_->ActiveRobustChi2();

    solver_->BuildSystem();

    // core part of the Levenbarg algorithm
    if (iteration == 0) {
        current_lambda_ = ComputeLambdaInit();
        ni_ = 2;
    }

    double rho = 0;
    int &qmax = lm_iter_;
    qmax = 0;
    do {
        optimizer_->Push();

        // update the diagonal of the system matrix
        solver_->SetLambda(current_lambda_, true);
        bool ok2 = true;
        bool should_break = false;
        ok2 = solver_->Solve();

        optimizer_->Update(solver_->GetX());

        // restore the diagonal
        solver_->RestoreDiagonal();

        optimizer_->ComputeActiveErrors();
        double tempChi = optimizer_->ActiveRobustChi2();

        if (!ok2) {
            tempChi = std::numeric_limits<double>::max();
        }

        rho = (currentChi - tempChi);
        double scale = ok2 ? ComputeScale() + 1e-3 : 1;  // make sure it's non-zero :)
        rho /= scale;

        if (rho > 0 && std::isfinite(tempChi) && ok2) {  // last step was good
            double alpha = 1. - pow((2 * rho - 1), 3);
            // crop lambda between minimum and maximum factors
            alpha = (std::min)(alpha, good_step_upper_);
            double scaleFactor = (std::max)(good_step_lower_, alpha);
            current_lambda_ *= scaleFactor;
            ni_ = 2;
            currentChi = tempChi;
            optimizer_->DiscardTop();
        } else {
            current_lambda_ *= ni_;
            ni_ *= 2;
            optimizer_->Pop();  // restore the last state before trying to optimize
            if (!std::isfinite(current_lambda_)) {
                should_break = true;
            }
        }
        qmax++;

        // LOG(INFO) << "lm iter: " << qmax << ", temp chi: " << tempChi << ", current chi: " << currentChi
        //           << ", rho: " << rho;

        if (should_break) {
            break;
        }

    } while (rho < 0 && qmax < max_trails_after_failure_ && !optimizer_->Terminate());

    if (qmax == max_trails_after_failure_ || rho == 0 || !std::isfinite(current_lambda_)) {
        // LOG(WARNING) << "solver terminated, rho: " << rho << ", qmax: " << qmax << ", " << current_lambda_;
        return SolverResult::Terminate;
    }

    // LOG(INFO) << "lm iter: " << qmax << ", lambda: " << current_lambda_;

    return SolverResult::OK;
}

double OptimizationAlgorithmLevenberg::ComputeLambdaInit() const {
    if (user_lambda_limit_ > 0) {
        return user_lambda_limit_;
    }

    double maxDiagonal = 0;
    for (size_t k = 0; k < optimizer_->IndexMapping().size(); k++) {
        auto v = optimizer_->IndexMapping()[k];
        assert(v);
        int dim = v->Dimension();
        for (int j = 0; j < dim; ++j) {
            maxDiagonal = std::max(fabs(v->Hessian(j, j)), maxDiagonal);
        }
    }

    return tau_ * maxDiagonal;
}

double OptimizationAlgorithmLevenberg::ComputeScale() const {
    double scale = 0;
    for (size_t j = 0; j < solver_->VectorSize(); j++) {
        scale += solver_->GetX()[j] * (current_lambda_ * solver_->GetX()[j] + solver_->GetB()[j]);
    }
    return scale;
}

}  // namespace lightning::miao