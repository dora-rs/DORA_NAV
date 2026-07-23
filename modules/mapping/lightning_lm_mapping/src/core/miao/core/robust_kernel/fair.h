//
// Created by xiang on 24-5-13.
//

#ifndef MIAO_FAIR_H
#define MIAO_FAIR_H

#include "robust_kernel.h"

namespace lightning::miao {

/**
 * \brief Fair cost function
 *
 * See
 * http://research.microsoft.com/en-us/um/people/zhang/Papers/ZhangIVC-97-01.pdf
 *
 * 2 * d^2 [e2 / d - log (1 + e2 / d)]
 *
 */
class RobustKernelFair : public RobustKernel {
   public:
    virtual void Robustify(double e2, Vector3& rho) const override {
        const double sqrte = sqrt(e2);
        const double dsqr = delta_ * delta_;
        const double aux = sqrte / delta_;
        rho[0] = 2. * dsqr * (aux - log1p(aux));
        rho[1] = 1. / (1. + aux);

        const double drec = 1. / delta_;
        const double e_3_2 = 1. / (sqrte * e2);
        const double aux2 = drec * sqrte + 1;

        rho[2] = 2 * dsqr * (1 / (4 * dsqr * aux2 * aux2 * e2) + (drec * e_3_2) / (4 * aux2) - (drec * e_3_2) / 4);
    }
};
}  // namespace lightning::miao

#endif  // MIAO_FAIR_H
