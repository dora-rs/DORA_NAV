//
// Created by xiang on 24-5-13.
//

#ifndef MIAO_HUBER_H
#define MIAO_HUBER_H

#include "robust_kernel.h"

namespace lightning::miao {

/**
 * \brief Huber Cost Function
 *
 * Loss function as described by Huber
 * See http://en.wikipedia.org/wiki/Huber_loss_function
 *
 * If e^(1/2) < d
 * rho(e) = e
 *
 * else
 *
 *               1/2    2
 * rho(e) = 2 d e    - d
 */
class RobustKernelHuber : public RobustKernel {
   public:
    void Robustify(double e, Vector3& rho) const override {
        double dsqr = delta_ * delta_;
        if (e <= dsqr) {  // inlier
            rho[0] = e;
            rho[1] = 1.;
            rho[2] = 0.;
        } else {                                 // outlier
            double sqrte = sqrt(e);              // absolute value of the error
            rho[0] = 2 * sqrte * delta_ - dsqr;  // rho(e)   = 2 * delta * e^(1/2) - delta^2
            rho[1] = delta_ / sqrte;             // rho'(e)  = delta / sqrt(e)
            rho[2] = -0.5 * rho[1] / e;          // rho''(e) = -1 / (2*e^(3/2)) = -1/2 * (delta/e) / e
        }
    }
};

}  // namespace lightning::miao

#endif  // MIAO_HUBER_H
