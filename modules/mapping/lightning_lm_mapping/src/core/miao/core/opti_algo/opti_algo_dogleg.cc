//
// Created by xiang on 24-5-13.
//

#include "opti_algo_dogleg.h"
#include "core/graph/optimizer.h"
#include "core/graph/vertex.h"
#include "core/math/misc.h"
#include "core/solver/solver.h"

#include <glog/logging.h>
#include <cassert>
#include <cmath>
#include <iostream>

namespace lightning::miao {

OptimizationAlgorithmDogleg::OptimizationAlgorithmDogleg(std::shared_ptr<Solver> solver)
    : OptimizationAlgorithm(solver) {}

OptimizationAlgorithmDogleg::~OptimizationAlgorithmDogleg() {}

OptimizationAlgorithm::SolverResult OptimizationAlgorithmDogleg::Solve(int iteration) {
    assert(optimizer_ && "_optimizer not set");

    if (iteration == 0) {  // built up the CCS structure, here due to easy time measure
        bool ok = solver_->BuildStructure();
        if (!ok) {
            LOG(WARNING) << "Failure while building CCS structure";
            return SolverResult::Fail;
        }

        // init some members to the current size of the problem
        hsd_.resize(solver_->VectorSize());
        hdl_.resize(solver_->VectorSize());
        aux_vector_.resize(solver_->VectorSize());
        delta_ = user_delta_init_;
        current_lambda_ = init_lambda_;
        was_pd_in_all_iterations_ = true;
    }

    optimizer_->ComputeActiveErrors();
    double currentChi = optimizer_->ActiveRobustChi2();

    solver_->BuildSystem();
    lightning::VectorX::ConstMapType b(solver_->GetB(), solver_->VectorSize());

    // compute alpha
    aux_vector_.setZero();
    solver_->MultiplyHessian(aux_vector_.data(), solver_->GetB());
    double bNormSquared = b.squaredNorm();
    double alpha = bNormSquared / aux_vector_.dot(b);

    hsd_ = alpha * b;
    double hsdNorm = hsd_.norm();
    double hgnNorm = -1.;

    bool solvedGaussNewton = false;
    bool goodStep = false;
    int& numTries = last_num_tries_;
    numTries = 0;

    do {
        ++numTries;

        if (!solvedGaussNewton) {
            const double minLambda = cst(1e-12);
            const double maxLambda = cst(1e3);
            solvedGaussNewton = true;
            // apply a damping factor to enforce positive definite Hessian, if the
            // matrix appeared to be not positive definite in at least one iteration
            // before. We apply a damping factor to obtain a PD matrix.
            bool solverOk = false;
            while (!solverOk) {
                if (!was_pd_in_all_iterations_)
                    solver_->SetLambda(current_lambda_, true);  // add _currentLambda to the diagonal
                solverOk = solver_->Solve();
                if (!was_pd_in_all_iterations_) {
                    solver_->RestoreDiagonal();
                }

                was_pd_in_all_iterations_ = was_pd_in_all_iterations_ && solverOk;
                if (!was_pd_in_all_iterations_) {
                    // simple strategy to control the damping factor
                    if (solverOk) {
                        current_lambda_ = std::max(minLambda, current_lambda_ / (cst(0.5) * lambda_factor_));
                    } else {
                        current_lambda_ *= lambda_factor_;
                        if (current_lambda_ > maxLambda) {
                            current_lambda_ = maxLambda;
                            return SolverResult::Fail;
                        }
                    }
                }
            }

            hgnNorm = lightning::VectorX::ConstMapType(solver_->GetX(), solver_->VectorSize()).norm();
        }

        lightning::VectorX::ConstMapType hgn(solver_->GetX(), solver_->VectorSize());
        assert(hgnNorm >= 0. && "Norm of the GN step is not computed");

        if (hgnNorm < delta_) {
            hdl_ = hgn;
        } else if (hsdNorm > delta_) {
            hdl_ = delta_ / hsdNorm * hsd_;
        } else {
            aux_vector_ = hgn - hsd_;  // b - a
            double c = hsd_.dot(aux_vector_);
            double bmaSquaredNorm = aux_vector_.squaredNorm();
            double beta;
            if (c <= 0.)
                beta = (-c + sqrt(c * c + bmaSquaredNorm * (delta_ * delta_ - hsd_.squaredNorm()))) / bmaSquaredNorm;
            else {
                double hsdSqrNorm = hsd_.squaredNorm();
                beta = (delta_ * delta_ - hsdSqrNorm) /
                       (c + sqrt(c * c + bmaSquaredNorm * (delta_ * delta_ - hsdSqrNorm)));
            }

            assert(beta > 0. && beta < 1 && "Error while computing beta");
            hdl_ = hsd_ + beta * (hgn - hsd_);
            assert(hdl_.norm() < delta_ + 1e-5 && "Computed step does not correspond to the trust region");
        }

        // compute the linear gain
        aux_vector_.setZero();
        solver_->MultiplyHessian(aux_vector_.data(), hdl_.data());
        double linearGain = -1 * (aux_vector_.dot(hdl_)) + 2 * (b.dot(hdl_));

        // apply the update and see what happens
        optimizer_->Push();
        optimizer_->Update(hdl_.data());
        optimizer_->ComputeActiveErrors();
        double newChi = optimizer_->ActiveRobustChi2();
        double nonLinearGain = currentChi - newChi;
        if (fabs(linearGain) < 1e-12) {
            linearGain = cst(1e-12);
        }

        double rho = nonLinearGain / linearGain;

        if (rho > 0) {  // step is good and will be accepted
            optimizer_->DiscardTop();
            goodStep = true;
        } else {  // recover previous state
            optimizer_->Pop();
        }

        // update trust region based on the step quality
        if (rho > 0.75) {
            delta_ = std::max<double>(delta_, 3 * hdl_.norm());
        } else if (rho < 0.25) {
            delta_ *= 0.5;
        }
    } while (!goodStep && numTries < max_trail_after_failure_);

    if (numTries == max_trail_after_failure_ || !goodStep) {
        return SolverResult::Terminate;
    }
    return SolverResult::OK;
}

}  // namespace lightning::miao