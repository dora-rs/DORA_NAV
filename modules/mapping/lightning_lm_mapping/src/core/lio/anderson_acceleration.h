#ifndef ANDERSONACCELERATION_H_
#define ANDERSONACCELERATION_H_

#include <omp.h>
#include <algorithm>
#include <cassert>
#include <fstream>
#include <vector>

#include "common/eigen_types.h"

namespace lightning {
/**
 * AA 加速器
 * @tparam S Scalar type
 * @tparam D dimension
 * @tparam m 允许前多少次迭代, 最多10次，不允许动态设置，只能编译期设置
 */
template <typename S, int D, int m>
class AndersonAcceleration {
   public:
    using Scalar = S;
    using Vec = Eigen::Matrix<S, D, 1>;
    using MatDM = Eigen::Matrix<S, D, m>;
    using MatDD = Eigen::Matrix<S, D, D>;

    /**
     * 为g计算加速之后的结果
     * @param g 输入的更新量
     * @return  加速之后的更新量
     */
    Vec compute(const Vec& g) {
        assert(iter_ >= 0);
        Vec G = g;
        current_F_ = G - current_u_;

        if (iter_ == 0) {
            prev_dF_.col(0) = -current_F_;
            prev_dG_.col(0) = -G;
            current_u_ = G;
        } else {
            prev_dF_.col(col_idx_) += current_F_;
            prev_dG_.col(col_idx_) += G;

            Scalar eps = 1e-14;
            Scalar scale = std::max(eps, prev_dF_.col(col_idx_).norm());
            dF_scale_(col_idx_) = scale;
            prev_dF_.col(col_idx_) /= scale;

            int m_k = std::min(m, iter_);

            if (m_k == 1) {
                theta_(0) = 0;
                Scalar dF_sqrnorm = prev_dF_.col(col_idx_).squaredNorm();
                M_(0, 0) = dF_sqrnorm;
                Scalar dF_norm = std::sqrt(dF_sqrnorm);

                if (dF_norm > eps) {
                    theta_(0) = (prev_dF_.col(col_idx_) / dF_norm).dot(current_F_ / dF_norm);
                }
            } else {
                // Update the normal equation matrix, for the column and row corresponding to the new dF column
                VecXd new_inner_prod = (prev_dF_.col(col_idx_).transpose() * prev_dF_.block(0, 0, D, m_k)).transpose();
                M_.block(col_idx_, 0, 1, m_k) = new_inner_prod.transpose();
                M_.block(0, col_idx_, m_k, 1) = new_inner_prod;

                // Solve normal equation
                cod_.compute(M_.block(0, 0, m_k, m_k));
                theta_.head(m_k) = cod_.solve(prev_dF_.block(0, 0, D, m_k).transpose() * current_F_);
            }

            // Use rescaled theta to compute new u
            current_u_ =
                G - prev_dG_.block(0, 0, D, m_k) * ((theta_.head(m_k).array() / dF_scale_.head(m_k).array()).matrix());
            col_idx_ = (col_idx_ + 1) % D;
            prev_dF_.col(col_idx_) = -current_F_;
            prev_dG_.col(col_idx_) = -G;
        }

        iter_++;
        return current_u_;
    }

    /**
     * 重置AA加速器
     * @param u
     */
    void reset(const Vec& u) {
        iter_ = 0;
        col_idx_ = 0;
        current_u_ = u;
    }

    /**
     * 初始化 anderson acceleration
     * @param u0: initial variable values
     */
    void init(const Vec& u0) {
        current_u_.setZero();
        current_F_.setZero();
        prev_dG_.setZero();
        prev_dF_.setZero();

        M_.setZero();
        theta_.setZero();
        dF_scale_.setZero();
        current_u_ = u0;

        iter_ = 0;
        col_idx_ = 0;
    }

   private:
    Vec current_u_ = Vec::Zero();  // 当前的更新
    Vec current_F_ = Vec::Zero();
    MatDM prev_dG_ = MatDM::Zero();
    MatDM prev_dF_ = MatDM::Zero();
    MatDD M_ = MatDD::Zero();                            // Normal equations matrix for the computing theta
    Vec theta_ = Vec::Zero();                            // theta value computed from normal equations
    Vec dF_scale_ = Vec::Zero();                         // The scaling factor for each column of prev_dF
    Eigen::CompleteOrthogonalDecomposition<MatXd> cod_;  // should use MatXd because iteration changes
    int iter_ = 0;                                       // Iteration count since initialization
    int col_idx_ = -1;                                   // Index for history matrix column to store the next value
};

}  // namespace lightning

#endif /* ANDERSONACCELERATION_H_ */
