//
// Created by xiang on 24-5-13.
//

#ifndef MIAO_PSEUDO_HUBER_H
#define MIAO_PSEUDO_HUBER_H

#include "robust_kernel.h"

namespace lightning::miao {

/**
 * \brief Pseudo Huber Cost Function
 *
 * The smooth pseudo huber cost function:
 * See http://en.wikipedia.org/wiki/Huber_loss_function
 *
 *    2       e
 * 2 d  (sqrt(-- + 1) - 1)
 *             2
 *            d
 */
class RobustKernelPseudoHuber : public RobustKernel {
   public:
    virtual void Robustify(double e2, Vector3& rho) const override {
        double dsqr = delta_ * delta_;
        double dsqrReci = 1. / dsqr;
        double aux1 = dsqrReci * e2 + 1.0;
        double aux2 = sqrt(aux1);
        rho[0] = 2 * dsqr * (aux2 - 1);
        rho[1] = 1. / aux2;
        rho[2] = -0.5 * dsqrReci * rho[1] / aux1;
    }
};

}  // namespace lightning::miao

#endif  // MIAO_PSEUDO_HUBER_H
