//
// Created by xiang on 24-4-24.
//

#ifndef MIAO_SPARSE_BLOCK_MATRIX_CCS_H
#define MIAO_SPARSE_BLOCK_MATRIX_CCS_H

#include <cstring>
#include <memory>
#include <vector>

#include "common/eigen_types.h"
#include "core/math/matrix_operations.h"

namespace lightning::miao {

/**
 * \brief Sparse matrix which uses blocks
 *
 * This class is used as a const view on a SparseBlockMatrix
 * which allows a faster iteration over the elements of the
 * matrix.
 */
template <class MatrixType>
class SparseBlockMatrixCCS {
   public:
    //! this is the type of the elementary Block, it is an Eigen::Matrix.
    typedef MatrixType SparseMatrixBlock;

    //! columns of the matrix
    int Cols() const { return col_block_indices_.size() ? col_block_indices_.back() : 0; }
    //! Rows of the matrix
    int Rows() const { return row_block_indices_.size() ? row_block_indices_.back() : 0; }

    /**
     * \brief A Block within a column
     */
    struct RowBlock {
        int row;                      ///< row of the Block
        MatrixType* block = nullptr;  ///< matrix pointer for the Block
        RowBlock() : row(-1), block(nullptr) {}
        RowBlock(int r, MatrixType* b) : row(r), block(b) {}
        bool operator<(const RowBlock& other) const { return row < other.row; }
    };
    typedef std::vector<RowBlock> SparseColumn;

    SparseBlockMatrixCCS(const std::vector<int>& rowIndices, const std::vector<int>& colIndices)
        : row_block_indices_(rowIndices), col_block_indices_(colIndices) {}

    //! how many Rows does the Block at Block-row r has?
    int RowsOfBlock(int r) const {
        return r ? row_block_indices_[r] - row_block_indices_[r - 1] : row_block_indices_[0];
    }

    //! how many Cols does the Block at Block-col c has?
    int ColsOfBlock(int c) const {
        return c ? col_block_indices_[c] - col_block_indices_[c - 1] : col_block_indices_[0];
    }

    //! where does the row at Block-row r start?
    int RowBaseOfBlock(int r) const { return r ? row_block_indices_[r - 1] : 0; }

    //! where does the col at Block-col r start?
    int ColBaseOfBlock(int c) const { return c ? col_block_indices_[c - 1] : 0; }

    //! the Block matrices per Block-column
    const std::vector<SparseColumn>& BlockCols() const { return block_cols_; }
    std::vector<SparseColumn>& BlockCols() { return block_cols_; }

    //! indices of the row blocks
    const std::vector<int>& RowBlockIndices() const { return row_block_indices_; }

    //! indices of the column blocks
    const std::vector<int>& ColBlockIndices() const { return col_block_indices_; }

    void RightMultiply(double*& dest, const double* src) const {
        int destSize = Cols();

        if (!dest) {
            dest = new double[destSize];
            memset(dest, 0, destSize * sizeof(double));
        }

        // map the memory by Eigen
        Eigen::Map<lightning::VectorX> destVec(dest, destSize);
        Eigen::Map<const lightning::VectorX> srcVec(src, Rows());

        for (int i = 0; i < static_cast<int>(block_cols_.size()); ++i) {
            int destOffset = ColBaseOfBlock(i);
            for (typename SparseColumn::const_iterator it = block_cols_[i].begin(); it != block_cols_[i].end(); ++it) {
                const SparseMatrixBlock* a = it->block;
                int srcOffset = RowBaseOfBlock(it->row);
                // destVec += *a.transpose() * srcVec (according to the sub-vector
                // parts)
                internal::template atxpy<SparseMatrixBlock>(*a, srcVec, srcOffset, destVec, destOffset);
            }
        }
    }

    /**
     * sort the blocks in each column
     */
    void SortColumns() {
        for (int i = 0; i < static_cast<int>(block_cols_.size()); ++i) {
            std::sort(block_cols_[i].begin(), block_cols_[i].end());
        }
    }

    /**
     * fill the CCS arrays of a matrix, arrays have to be allocated beforehand
     */
    int FillCCS(int* Cp, int* Ci, double* Cx, bool upperTriangle = false) const {
        assert(Cp && Ci && Cx && "Target destination is NULL");
        int nz = 0;
        for (size_t i = 0; i < block_cols_.size(); ++i) {
            int cstart = i ? col_block_indices_[i - 1] : 0;
            int csize = ColsOfBlock(i);
            for (int c = 0; c < csize; ++c) {
                *Cp = nz;
                for (auto it = block_cols_[i].begin(); it != block_cols_[i].end(); ++it) {
                    const SparseMatrixBlock* b = it->block;
                    int rstart = it->row ? row_block_indices_[it->row - 1] : 0;

                    int elemsToCopy = b->rows();
                    if (upperTriangle && rstart == cstart) elemsToCopy = c + 1;
                    for (int r = 0; r < elemsToCopy; ++r) {
                        *Cx++ = (*b)(r, c);
                        *Ci++ = rstart++;
                        ++nz;
                    }
                }
                ++Cp;
            }
        }
        *Cp = nz;
        return nz;
    }

    /**
     * fill the CCS arrays of a matrix, arrays have to be allocated beforehand.
     * This function only writes the values and assumes that column and row
     * structures have already been written.
     */
    int FillCCS(double* Cx, bool upperTriangle = false) const {
        assert(Cx && "Target destination is NULL");
        double* CxStart = Cx;
        int cstart = 0;
        for (size_t i = 0; i < block_cols_.size(); ++i) {
            int csize = col_block_indices_[i] - cstart;
            for (int c = 0; c < csize; ++c) {
                for (typename SparseColumn::const_iterator it = block_cols_[i].begin(); it != block_cols_[i].end();
                     ++it) {
                    const SparseMatrixBlock* b = it->block;
                    int rstart = it->row ? row_block_indices_[it->row - 1] : 0;

                    int elemsToCopy = b->rows();
                    if (upperTriangle && rstart == cstart) {
                        elemsToCopy = c + 1;
                    }

                    memcpy(Cx, b->data() + c * b->rows(), elemsToCopy * sizeof(double));
                    Cx += elemsToCopy;
                }
            }
            cstart = col_block_indices_[i];
        }
        return Cx - CxStart;
    }

   protected:
    const std::vector<int>& row_block_indices_;  ///< vector of the indices of the
    ///< blocks along the Rows.
    const std::vector<int>& col_block_indices_;  ///< vector of the indices of the blocks along the Cols
    std::vector<SparseColumn> block_cols_;       ///< the matrices stored in CCS order
};
}  // namespace lightning::miao

#endif  // MIAO_SPARSE_BLOCK_MATRIX_CCS_H
