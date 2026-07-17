//
// Created by xiang on 24-5-13.
//

#ifndef MIAO_TUKEY_H
#define MIAO_TUKEY_H

#include "robust_kernel.h"

namespace lightning::miao {

/**
 * \brief Tukey Cost Function
 *
 * See
 * http://research.microsoft.com/en-us/um/people/zhang/Papers/ZhangIVC-97-01.pdf
 *
 * If e2^(1/2) <= d
 * rho(e) = d^2 * (1 - ( 1 - e2 / d^2)^3) / 3
 *
 * else
 *
 * rho(e) = d^2 / 3
 */
class RobustKernelTukey : public RobustKernel {
   public:
    virtual void Robustify(double e2, Vector3& rho) const override {
        const double delta2 = delta_ * delta_;
        if (e2 <= delta2) {
            const double aux = e2 / delta2;
            rho[0] = delta2 * (1. - std::pow((1. - aux), 3)) / 3.;
            rho[1] = std::pow((1. - aux), 2);
            rho[2] = -2. * (1. - aux) / delta2;
        } else {
            rho[0] = delta2 / 3.;
            rho[1] = 0;
            rho[2] = 0;
        }
    }
};
}  // namespace lightning::miao

#endif  // MIAO_TUKEY_H
