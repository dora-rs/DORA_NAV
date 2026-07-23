//
// Created by xiang on 24-4-24.
//

#ifndef MIAO_SPARSE_BLOCK_HASHMAP_H
#define MIAO_SPARSE_BLOCK_HASHMAP_H

#include "common/eigen_types.h"

namespace lightning::miao {

/**
 * \brief Sparse matrix which uses blocks based on hash structures
 *
 * This class is used to construct the pattern of a sparse Block matrix
 *
 * hash map版本的sparse matrix
 */
template <class MatrixType>
class SparseBlockMatrixHashMap {
   public:
    //! this is the type of the elementary Block, it is an Eigen::Matrix.
    using SparseMatrixBlock = MatrixType;

    //! columns of the matrix
    int Cols() const { return col_block_indices_.size() ? col_block_indices_.back() : 0; }
    //! Rows of the matrix
    int Rows() const { return row_block_indices_.size() ? row_block_indices_.back() : 0; }

    using SparseColumn = std::unordered_map<int, MatrixType*>;

    SparseBlockMatrixHashMap(const std::vector<int>& rowIndices, const std::vector<int>& colIndices)
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

    /**
     * Add a Block to the pattern, return a pointer to the added Block
     */
    MatrixType* AddBlock(int r, int c, bool zeroBlock = false) {
        assert(c < (int)block_cols_.size() && "accessing column which is not available");
        SparseColumn& sparseColumn = block_cols_[c];
        typename SparseColumn::iterator foundIt = sparseColumn.find(r);
        if (foundIt == sparseColumn.end()) {
            int rb = RowsOfBlock(r);
            int cb = ColsOfBlock(c);
            MatrixType* m = new MatrixType(rb, cb);
            if (zeroBlock) m->setZero();
            sparseColumn[r] = m;
            return m;
        }
        return foundIt->second;
    }

   protected:
    const std::vector<int>& row_block_indices_;  ///< vector of the indices of the
    ///< blocks along the Rows.
    const std::vector<int>& col_block_indices_;  ///< vector of the indices of the blocks along the Cols
    std::vector<SparseColumn> block_cols_;       ///< the matrices stored in CCS order
};

}  // namespace lightning::miao
#endif  // MIAO_SPARSE_BLOCK_HASHMAP_H
