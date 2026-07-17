//
// Created by xiang on 24-5-13.
//

#ifndef MIAO_MC_CLURE_H
#define MIAO_MC_CLURE_H

#include "robust_kernel.h"

namespace lightning::miao {

/**
 * \brief Geman-McClure cost function
 *
 * See
 * http://research.microsoft.com/en-us/um/people/zhang/Papers/ZhangIVC-97-01.pdf
 * and
 * http://www2.informatik.uni-freiburg.de/~agarwal/resources/agarwal-thesis.pdf
 *    e2
 *  -----
 *  e2 + 1
 */
class RobustKernelGemanMcClure : public RobustKernel {
   public:
    virtual void Robustify(double e2, Vector3& rho) const override {
        const double aux = 1. / (delta_ + e2);
        rho[0] = delta_ * e2 * aux;
        rho[1] = delta_ * delta_ * aux * aux;
        rho[2] = -2. * rho[1] * aux;
    }
};

}  // namespace lightning::miao
#endif  // MIAO_MC_CLURE_H
