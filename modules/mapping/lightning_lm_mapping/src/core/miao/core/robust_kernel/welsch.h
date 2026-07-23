//
// Created by xiang on 24-5-13.
//

#ifndef MIAO_WELSCH_H
#define MIAO_WELSCH_H

#include "robust_kernel.h"

namespace lightning::miao {

/**
 * \brief Welsch cost function
 *
 * See
 * http://research.microsoft.com/en-us/um/people/zhang/Papers/ZhangIVC-97-01.pdf
 *
 * d^2 [1 - exp(- e2/d^2)]
 *
 */
class RobustKernelWelsch : public RobustKernel {
   public:
    virtual void Robustify(double e2, Vector3& rho) const override {
        const double dsqr = delta_ * delta_;
        const double aux = e2 / dsqr;
        const double aux2 = exp(-aux);
        rho[0] = dsqr * (1. - aux2);
        rho[1] = aux2;
        rho[2] = -aux2 / dsqr;
    }
};

}  // namespace lightning::miao

#endif  // MIAO_WELSCH_H
