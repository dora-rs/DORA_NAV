//
// Created by xiang on 24-5-13.
//

#ifndef MIAO_SATURATED_H
#define MIAO_SATURATED_H

#include "robust_kernel.h"

namespace lightning::miao {

/**
 * \brief Saturated cost function.
 *
 * The error is at most Delta^2
 */
class RobustKernelSaturated : public RobustKernel {
   public:
    virtual void Robustify(double e2, Vector3& rho) const override {
        double dsqr = delta_ * delta_;
        if (e2 <= dsqr) {  // inlier
            rho[0] = e2;
            rho[1] = 1.;
            rho[2] = 0.;
        } else {  // outlier
            rho[0] = dsqr;
            rho[1] = 0.;
            rho[2] = 0.;
        }
    }
};
}  // namespace lightning::miao

#endif  // MIAO_SATURATED_H
