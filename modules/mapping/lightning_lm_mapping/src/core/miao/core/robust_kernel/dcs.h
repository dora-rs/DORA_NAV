//
// Created by xiang on 24-5-13.
//

#ifndef MIAO_DCS_H
#define MIAO_DCS_H

#include "robust_kernel.h"

namespace lightning::miao {

/**
 * \brief Dynamic covariance scaling - DCS
 *
 * See paper Robust Map Optimization from Agarwal et al.  ICRA 2013
 *
 * Delta is used as $phi$
 */
class RobustKernelDCS : public RobustKernel {
   public:
    virtual void Robustify(double e2, Vector3& rho) const override {
        const double& phi = delta_;
        double scale = (2.0 * phi) / (phi + e2);
        if (scale >= 1.0) {  // limit scale to max of 1 and return this
            rho[0] = e2;
            rho[1] = 1.;
            rho[2] = 0;
        } else {
            double phi_sqr = phi * phi;
            rho[0] = scale * e2 * scale;
            rho[1] = (4 * phi_sqr * (phi - e2)) / std::pow(phi + e2, 3);
            rho[2] = -(8 * phi_sqr * (2 * phi - e2)) / std::pow(phi + e2, 4);
        }
    };
};

}  // namespace lightning::miao

#endif  // MIAO_DCS_H
