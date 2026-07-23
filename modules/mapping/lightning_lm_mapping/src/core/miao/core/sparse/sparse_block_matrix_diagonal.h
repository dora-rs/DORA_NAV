//
// Created by xiang on 24-4-26.
//

#ifndef MIAO_SPARSE_BLOCK_MATRIX_DIAGONAL_H
#define MIAO_SPARSE_BLOCK_MATRIX_DIAGONAL_H

#include "core/math/matrix_operations.h"

namespace lightning::miao {

/**
 * 对角线的稀疏矩阵块
 * @tparam MatrixType
 */
template <class MatrixType>
class SparseBlockMatrixDiagonal {
   public:
    //! this is the type of the elementary Block, it is an Eigen::Matrix.
    typedef MatrixType SparseMatrixBlock;

    //! columns of the matrix
    int Cols() const { return block_indices_.size() ? block_indices_.back() : 0; }
    //! Rows of the matrix
    int Rows() const { return block_indices_.size() ? block_indices_.back() : 0; }

    using DiagonalVector = std::vector<MatrixType>;

    explicit SparseBlockMatrixDiagonal(const std::vector<int>& blockIndices) : block_indices_(blockIndices) {}

    //! how many Rows/Cols does the block at Block-row / Block-column r has?
    inline int DimOfBlock(int r) const { return r ? block_indices_[r] - block_indices_[r - 1] : block_indices_[0]; }

    //! where does the row /col at Block-row / Block-column r starts?
    inline int BaseOfBlock(int r) const { return r ? block_indices_[r - 1] : 0; }

    //! the Block matrices per Block-column
    const DiagonalVector& Diagonal() const { return diagonal_; }
    DiagonalVector& Diagonal() { return diagonal_; }

    //! indices of the row blocks
    const std::vector<int>& BlockIndices() const { return block_indices_; }

    void Multiply(double*& dest, const double* src) const {
        int destSize = Cols();
        if (!dest) {
            dest = new double[destSize];
            memset(dest, 0, destSize * sizeof(double));
        }

        // map the memory by Eigen
        Eigen::Map<lightning::VectorX> destVec(dest, destSize);
        Eigen::Map<const lightning::VectorX> srcVec(src, Rows());

        for (int i = 0; i < static_cast<int>(diagonal_.size()); ++i) {
            int destOffset = BaseOfBlock(i);
            int srcOffset = destOffset;
            const SparseMatrixBlock& A = diagonal_[i];
            // destVec += *A.transpose() * srcVec (according to the sub-vector parts)
            internal::template axpy<SparseMatrixBlock>(A, srcVec, srcOffset, destVec, destOffset);
        }
    }

   protected:
    const std::vector<int>& block_indices_;  ///< vector of the indices of the
    ///< blocks along the Diagonal
    DiagonalVector diagonal_;
};

}  // namespace lightning::miao
#endif  // MIAO_SPARSE_BLOCK_MATRIX_DIAGONAL_H
