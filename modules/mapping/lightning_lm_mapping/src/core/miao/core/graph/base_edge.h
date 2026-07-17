//
// Created by xiang on 24-4-17.
//

#ifndef MIAO_BASE_EDGE_H
#define MIAO_BASE_EDGE_H

#include "common/eigen_types.h"
#include "edge.h"

namespace lightning::miao {

/**
 * 模板化的edge
 * @tparam D    边的维度
 * @tparam E    边的measurement type
 *
 * 由于只有edge本身的维度和measurement维度，没有关联的vertex维度，所以只能定义information矩阵维度，measurement维度，而没有jacobian维度
 */
template <int D, typename E>
class BaseEdge : public Edge {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr int Dimension = D;
    using Measurement = E;
    using ErrorVector = Eigen::Matrix<double, D, 1>;
    using InformationType = Eigen::Matrix<double, D, D>;

    BaseEdge() : Edge() { dimension_ = D; }

    // DISALLOW_COPY(BaseEdge);

    virtual ~BaseEdge() {}

    double Chi2() const override { return error_.dot(information_ * error_); }

    //! information matrix of the constraint
    EIGEN_STRONG_INLINE const InformationType& Information() const { return information_; }
    EIGEN_STRONG_INLINE InformationType& Information() { return information_; }
    void SetInformation(const InformationType& information) { information_ = information; }

    //! accessor functions for the measurement represented by the edge
    EIGEN_STRONG_INLINE const Measurement& GetMeasurement() const { return measurement_; }
    virtual void SetMeasurement(const Measurement& m) { measurement_ = m; }

    EIGEN_STRONG_INLINE const ErrorVector& GetError() const { return error_; }

   protected:
    /**
     * calculate the robust information matrix by updating the information matrix
     * of the error
     */
    InformationType RobustInformation(const Vec3d& rho) const { return rho[1] * information_; }

    Measurement measurement_;      ///< the measurement of the edge
    InformationType information_;  ///< information matrix of the edge.
    ErrorVector error_;            ///< error vector, stores the result after computeError() is called
};

}  // namespace lightning::miao

#endif  // MIAO_BASE_EDGE_H
