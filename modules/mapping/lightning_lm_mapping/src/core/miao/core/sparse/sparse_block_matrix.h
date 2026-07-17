//
// Created by xiang on 24-4-24.
//

#ifndef MIAO_SPARSE_BLOCK_MATRIX_H
#define MIAO_SPARSE_BLOCK_MATRIX_H

#include "common/eigen_types.h"
#include "sparse_block_matrix_ccs.h"
#include "sparse_block_matrix_hashmap.h"

#include <map>
#include <memory>

namespace lightning::miao {
/**
 * \brief Sparse matrix which uses blocks
 *
 * Template class that specifies a sparse Block matrix.  A Block
 * matrix is a sparse matrix made of dense blocks.  These blocks
 * cannot have a random pattern, but follow a (variable) grid
 * structure. This structure is specified by a partition of the Rows
 * and the columns of the matrix.  The blocks are represented by the
 * Eigen::Matrix structure, thus they can be statically or dynamically
 * allocated. For efficiency reasons it is convenient to allocate them
 * statically, when possible. A static Block matrix has all blocks of
 * the same size, and the size of the Block is specified by the
 * template argument.  If this is not the case, and you have different
 * Block sizes than you have to use a dynamic-Block matrix (default
 * template argument).
 */
template <class MatrixType = lightning::MatrixX>
class SparseBlockMatrix {
   public:
    //! this is the type of the elementary Block, it is an Eigen::Matrix.
    using SparseMatrixBlock = MatrixType;

    //! columns of the matrix
    inline int Cols() const { return col_block_indices_.size() ? col_block_indices_.back() : 0; }
    //! Rows of the matrix
    inline int Rows() const { return row_block_indices_.size() ? row_block_indices_.back() : 0; }

    using IntBlockMap = std::map<int, SparseMatrixBlock*>;

    /**
     * constructs a sparse Block matrix having a specific layout
     * @param rbi: array of int containing the row layout of the blocks.
     * the component i of the array should contain the index of the first row of
     * the Block i+1.
     * @param cbi: array of int containing the column layout of the blocks.
     *  the component i of the array should contain the index of the first col of
     * the Block i+1.
     * @param rb: number of row blocks
     * @param cb: number of col blocks
     * @param hasStorage: set it to true if the matrix "owns" the blocks, thus it
     * deletes it on destruction. if false the matrix is only a "view" over an
     * existing structure.
     */
    SparseBlockMatrix(const int* rbi, const int* cbi, int rb, int cb, bool hasStorage = true);

    SparseBlockMatrix(const std::vector<int>& row_block_index, const std::vector<int>& col_block_index,
                      bool hasStorage = true);

    SparseBlockMatrix();

    ~SparseBlockMatrix();

    /**
     * 设定新增的blocks
     */
    void SetBlockIndexInc(const std::vector<int>& row_block_index, const std::vector<int>& col_block_index);

    //! this zeroes all the blocks. If dealloc=true the blocks are removed from
    //! memory
    /**
     * 置零整个稀疏矩阵
     * @param dealloc 为true时，释放所有内存
     */
    void Clear(bool dealloc = false);

    //! returns the Block at location r,c. if alloc=true he Block is created if it
    //! does not exist
    SparseMatrixBlock* Block(int r, int c, bool alloc = false);
    //! returns the Block at location r,c
    const SparseMatrixBlock* Block(int r, int c) const;

    //! how many Rows does the Block at Block-row r has?
    inline int RowsOfBlock(int r) const {
        return r ? row_block_indices_[r] - row_block_indices_[r - 1] : row_block_indices_[0];
    }

    //! how many Cols does the Block at Block-col c has?
    inline int ColsOfBlock(int c) const {
        return c ? col_block_indices_[c] - col_block_indices_[c - 1] : col_block_indices_[0];
    }

    //! where does the row at Block-row r starts?
    inline int RowBaseOfBlock(int r) const { return r ? row_block_indices_[r - 1] : 0; }

    //! where does the col at Block-col r starts?
    inline int ColBaseOfBlock(int c) const { return c ? col_block_indices_[c - 1] : 0; }

    //! number of non-zero elements
    size_t NonZeros() const;
    //! number of allocated blocks
    size_t NonZeroBlocks() const;

    //! transposes a Block matrix, The transposed type should match the argument
    //! false on failure
    template <class MatrixTransposedType>
    bool transpose(SparseBlockMatrix<MatrixTransposedType>& dest) const;

    //! adds the current matrix to the destination
    bool Add(SparseBlockMatrix<MatrixType>& dest) const;

    /**
     * compute dest = (*this) *  src
     * However, assuming that this is a symmetric matrix where only the upper
     * triangle is stored
     */
    void MultiplySymmetricUpperTriangle(double*& dest, const double* src) const;

    //! exports the non zero blocks into a column compressed structure
    void FillBlockStructure(int* Cp, int* Ci) const;

    //! the Block matrices per Block-column
    const std::vector<IntBlockMap>& BlockCols() const { return block_cols_; }
    std::vector<IntBlockMap>& BlockCols() { return block_cols_; }

    //! indices of the row blocks
    const std::vector<int>& RowBlockIndices() const { return row_block_indices_; }
    std::vector<int>& RowBlockIndices() { return row_block_indices_; }

    //! indices of the column blocks
    const std::vector<int>& ColBlockIndices() const { return col_block_indices_; }
    std::vector<int>& ColBlockIndices() { return col_block_indices_; }

    /**
     * copy into CCS structure
     * @return number of processed blocks, -1 on error
     */
    int FillSparseBlockMatrixCCS(SparseBlockMatrixCCS<MatrixType>& blockCCS) const;

    /**
     * copy as transposed into a CCS structure
     * @return number of processed blocks, -1 on error
     */
    int FillSparseBlockMatrixCCSTransposed(SparseBlockMatrixCCS<MatrixType>& blockCCS) const;

    /**
     * take over the memory and matrix pattern from a hash matrix.
     * The structure of the hash matrix will be cleared.
     */
    void TakePatternFromHash(SparseBlockMatrixHashMap<MatrixType>& hashMatrix);

    bool WriteOctave(const std::string& filename, bool upperTriangle = true) const;

   protected:
    std::vector<int> row_block_indices_;  ///< vector of the indices of the blocks
    ///< along the Rows.
    std::vector<int> col_block_indices_;  ///< vector of the indices of the blocks
    ///< along the Cols
    //! array of maps of blocks. The index of the array represent a Block column
    //! of the matrix and the Block column is stored as a map row_block ->
    //! matrix_block_ptr.
    std::vector<IntBlockMap> block_cols_;  // vector 索引为列，内部map索引为行
    bool has_storage_;

   private:
    template <class MatrixTransposedType>
    void TransposeInternal(SparseBlockMatrix<MatrixTransposedType>& dest) const;

    void AddInternal(SparseBlockMatrix<MatrixType>& dest) const;
};

template <class MatrixType>
std::ostream& operator<<(std::ostream&, const SparseBlockMatrix<MatrixType>& m);

using SparseBlockMatrixX = SparseBlockMatrix<lightning::MatrixX>;

}  // namespace lightning::miao

#include "sparse_block_matrix.hpp"

#endif  // MIAO_SPARSE_BLOCK_MATRIX_H
