//
// Created by xiang on 24-4-25.
//

#ifndef MIAO_ROBUST_KERNEL_H
#define MIAO_ROBUST_KERNEL_H

#include "common/eigen_types.h"

#include <memory>

namespace lightning::miao {

/**
 * \brief base for all robust cost functions
 *
 * Note in all the implementations for the other cost functions the e in the
 * functions corresponds to the squared errors, i.e., the robustification
 * functions gets passed the squared error.
 *
 * e.g. the robustified least squares function is
 *
 * chi^2 = sum_{e} rho( e^T Omega e )
 */
class RobustKernel {
   public:
    RobustKernel() {}
    explicit RobustKernel(double delta) : delta_(delta) {}
    virtual ~RobustKernel() = default;

    /**
     * compute the scaling factor for a error:
     * The error is e^T Omega e
     * The output rho is
     * rho[0]: The actual scaled error value
     * rho[1]: First derivative of the scaling function
     * rho[2]: Second derivative of the scaling function
     */
    virtual void Robustify(double squaredError, lightning::Vec3d& rho) const = 0;

    /**
     * set the window size of the error. A squared error above Delta^2 is
     * considered as outlier in the data.
     */
    virtual void SetDelta(double delta) { delta_ = delta; }
    double Delta() const { return delta_; }

   protected:
    double delta_ = 1.0;
};

typedef std::shared_ptr<RobustKernel> RobustKernelPtr;

}  // namespace lightning::miao

#endif  // MIAO_ROBUST_KERNEL_H
