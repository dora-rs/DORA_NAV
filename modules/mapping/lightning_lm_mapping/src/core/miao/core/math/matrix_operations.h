//
// Created by xiang on 24-4-24.
//

#ifndef MIAO_MATRIX_OPERATIONS_H
#define MIAO_MATRIX_OPERATIONS_H

#include "common/eigen_types.h"

namespace lightning::miao::internal {

template <typename MatrixType>
inline void axpy(const MatrixType& A, const Eigen::Map<const lightning::VectorX>& x, int xoff,
                 Eigen::Map<lightning::VectorX>& y, int yoff) {
    y.segment<MatrixType::RowsAtCompileTime>(yoff) += A * x.segment<MatrixType::ColsAtCompileTime>(xoff);
}

template <int t>
inline void axpy(const Eigen::Matrix<double, Eigen::Dynamic, t>& A, const Eigen::Map<const lightning::VectorX>& x,
                 int xoff, Eigen::Map<lightning::VectorX>& y, int yoff) {
    y.segment(yoff, A.rows()) += A * x.segment<Eigen::Matrix<double, Eigen::Dynamic, t>::ColsAtCompileTime>(xoff);
}

template <>
inline void axpy<lightning::MatrixX>(const lightning::MatrixX& A, const Eigen::Map<const lightning::VectorX>& x,
                                     int xoff, Eigen::Map<lightning::VectorX>& y, int yoff) {
    y.segment(yoff, A.rows()) += A * x.segment(xoff, A.cols());
}

template <typename MatrixType>
inline void atxpy(const MatrixType& A, const Eigen::Map<const lightning::VectorX>& x, int xoff,
                  Eigen::Map<lightning::VectorX>& y, int yoff) {
    y.segment<MatrixType::ColsAtCompileTime>(yoff) += A.transpose() * x.segment<MatrixType::RowsAtCompileTime>(xoff);
}

template <int t>
inline void atxpy(const Eigen::Matrix<double, Eigen::Dynamic, t>& A, const Eigen::Map<const lightning::VectorX>& x,
                  int xoff, Eigen::Map<lightning::VectorX>& y, int yoff) {
    y.segment<Eigen::Matrix<double, Eigen::Dynamic, t>::ColsAtCompileTime>(yoff) +=
        A.transpose() * x.segment(xoff, A.rows());
}

template <>
inline void atxpy<lightning::MatrixX>(const lightning::MatrixX& A, const Eigen::Map<const lightning::VectorX>& x,
                                      int xoff, Eigen::Map<lightning::VectorX>& y, int yoff) {
    y.segment(yoff, A.cols()) += A.transpose() * x.segment(xoff, A.rows());
}

}  // namespace lightning::miao::internal

#endif  // MIAO_MATRIX_OPERATIONS_H
