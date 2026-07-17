//
// Created by xiang on 24-4-24.
//

#ifndef MIAO_SPARSE_BLCOK_MATRIX_HPP
#define MIAO_SPARSE_BLCOK_MATRIX_HPP

#include "sparse_block_matrix.h"

#include <fstream>
#include <iomanip>

namespace lightning::miao {

template <class MatrixType>
SparseBlockMatrix<MatrixType>::SparseBlockMatrix(const int* rbi, const int* cbi, int rb, int cb, bool hasStorage)
    : row_block_indices_(rbi, rbi + rb), col_block_indices_(cbi, cbi + cb), block_cols_(cb), has_storage_(hasStorage) {}

template <class MatrixType>
SparseBlockMatrix<MatrixType>::SparseBlockMatrix(const std::vector<int>& row_block_index,
                                                 const std::vector<int>& col_block_index, bool hasStorage)
    : row_block_indices_(row_block_index),
      col_block_indices_(col_block_index),
      block_cols_(col_block_index.size()),
      has_storage_(hasStorage) {}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::SetBlockIndexInc(const std::vector<int>& row_block_index,
                                                     const std::vector<int>& col_block_index) {
    for (const auto& r : row_block_index) {
        row_block_indices_.emplace_back(r);
    }
    for (const auto& c : col_block_index) {
        col_block_indices_.emplace_back(c);
        block_cols_.emplace_back(std::map<int, SparseMatrixBlock*>());
    }
}

template <class MatrixType>
SparseBlockMatrix<MatrixType>::SparseBlockMatrix() : block_cols_(0), has_storage_(true) {}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::Clear(bool dealloc) {
    for (int i = 0; i < static_cast<int>(block_cols_.size()); ++i) {
        for (auto it = block_cols_[i].begin(); it != block_cols_[i].end(); ++it) {
            typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* b = it->second;
            if (has_storage_ && dealloc) {
                delete b;
            } else {
                b->setZero();
            }
        }

        if (has_storage_ && dealloc) {
            block_cols_[i].clear();
        }
    }
}

template <class MatrixType>
SparseBlockMatrix<MatrixType>::~SparseBlockMatrix() {
    if (has_storage_) {
        Clear(true);
    }
}

template <class MatrixType>
typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* SparseBlockMatrix<MatrixType>::Block(int r, int c,
                                                                                                bool alloc) {
    typename SparseBlockMatrix<MatrixType>::IntBlockMap::iterator it = block_cols_[c].find(r);
    typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* _block = 0;
    if (it == block_cols_[c].end()) {
        if (!has_storage_ && !alloc) {
            return nullptr;
        } else {
            int rb = RowsOfBlock(r);
            int cb = ColsOfBlock(c);
            _block = new typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock(rb, cb);
            _block->setZero();
            block_cols_[c].insert(std::make_pair(r, _block));
        }
    } else {
        _block = it->second;
    }
    return _block;
}

template <class MatrixType>
const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* SparseBlockMatrix<MatrixType>::Block(int r,
                                                                                                      int c) const {
    auto it = block_cols_[c].find(r);
    if (it == block_cols_[c].end()) {
        return nullptr;
    }
    return it->second_;
}

template <class MatrixType>
template <class MatrixTransposedType>
void SparseBlockMatrix<MatrixType>::TransposeInternal(SparseBlockMatrix<MatrixTransposedType>& dest) const {
    for (size_t i = 0; i < block_cols_.size(); ++i) {
        for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it = block_cols_[i].begin();
             it != block_cols_[i].end(); ++it) {
            typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* s = it->second;
            typename SparseBlockMatrix<MatrixTransposedType>::SparseMatrixBlock* d = dest.Block(i, it->first, true);
            *d = s->transpose();
        }
    }
}

template <class MatrixType>
template <class MatrixTransposedType>
bool SparseBlockMatrix<MatrixType>::transpose(SparseBlockMatrix<MatrixTransposedType>& dest) const {
    if (!dest.has_storage_) {
        return false;
    }

    if (row_block_indices_.size() != dest.col_block_indices_.size()) {
        return false;
    }

    if (col_block_indices_.size() != dest.row_block_indices_.size()) {
        return false;
    }

    for (size_t i = 0; i < row_block_indices_.size(); ++i) {
        if (row_block_indices_[i] != dest.col_block_indices_[i]) {
            return false;
        }
    }
    for (size_t i = 0; i < col_block_indices_.size(); ++i) {
        if (col_block_indices_[i] != dest.row_block_indices_[i]) {
            return false;
        }
    }

    TransposeInternal(dest);
    return true;
}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::AddInternal(SparseBlockMatrix<MatrixType>& dest) const {
    for (size_t i = 0; i < block_cols_.size(); ++i) {
        for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it = block_cols_[i].begin();
             it != block_cols_[i].end(); ++it) {
            typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* s = it->second;
            typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* d = dest.Block(it->first, i, true);
            (*d) += *s;
        }
    }
}

template <class MatrixType>
bool SparseBlockMatrix<MatrixType>::Add(SparseBlockMatrix<MatrixType>& dest) const {
    if (!dest.has_storage_) {
        return false;
    }

    if (row_block_indices_.size() != dest.row_block_indices_.size()) {
        return false;
    }

    if (col_block_indices_.size() != dest.col_block_indices_.size()) {
        return false;
    }

    for (size_t i = 0; i < row_block_indices_.size(); ++i) {
        if (row_block_indices_[i] != dest.row_block_indices_[i]) {
            return false;
        }
    }

    for (size_t i = 0; i < col_block_indices_.size(); ++i) {
        if (col_block_indices_[i] != dest.col_block_indices_[i]) {
            return false;
        }
    }

    AddInternal(dest);
    return true;
}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::MultiplySymmetricUpperTriangle(double*& dest, const double* src) const {
    if (!dest) {
        dest = new double[row_block_indices_[row_block_indices_.size() - 1]];
        memset(dest, 0, row_block_indices_[row_block_indices_.size() - 1] * sizeof(double));
    }

    // map the memory by Eigen
    Eigen::Map<lightning::VectorX> destVec(dest, Rows());
    const Eigen::Map<const lightning::VectorX> srcVec(src, Cols());

    for (size_t i = 0; i < block_cols_.size(); ++i) {
        int srcOffset = ColBaseOfBlock(i);
        for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it = block_cols_[i].begin();
             it != block_cols_[i].end(); ++it) {
            const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a = it->second;
            int destOffset = RowBaseOfBlock(it->first);
            if (destOffset > srcOffset)  // only upper triangle
                break;
            // destVec += *a * srcVec (according to the sub-vector parts)
            internal::template axpy<typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock>(*a, srcVec, srcOffset,
                                                                                               destVec, destOffset);
            if (destOffset < srcOffset)
                internal::template atxpy<typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock>(
                    *a, srcVec, destOffset, destVec, srcOffset);
        }
    }
}

template <class MatrixType>
size_t SparseBlockMatrix<MatrixType>::NonZeroBlocks() const {
    size_t count = 0;
    for (size_t i = 0; i < block_cols_.size(); ++i) {
        count += block_cols_[i].size();
    }
    return count;
}

template <class MatrixType>
size_t SparseBlockMatrix<MatrixType>::NonZeros() const {
    if (MatrixType::SizeAtCompileTime != Eigen::Dynamic) {
        size_t nnz = NonZeroBlocks() * MatrixType::SizeAtCompileTime;
        return nnz;
    } else {
        size_t count = 0;
        for (size_t i = 0; i < block_cols_.size(); ++i) {
            for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it = block_cols_[i].begin();
                 it != block_cols_[i].end(); ++it) {
                const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a = it->second;
                count += a->cols() * a->rows();
            }
        }
        return count;
    }
}

template <class MatrixType>
std::ostream& operator<<(std::ostream& os, const SparseBlockMatrix<MatrixType>& m) {
    os << "RBI: " << m.RowBlockIndices().size();
    for (size_t i = 0; i < m.RowBlockIndices().size(); ++i) os << " " << m.RowBlockIndices()[i];
    os << std::endl;
    os << "CBI: " << m.ColBlockIndices().size();
    for (size_t i = 0; i < m.ColBlockIndices().size(); ++i) os << " " << m.ColBlockIndices()[i];
    os << std::endl;

    for (size_t i = 0; i < m.BlockCols().size(); ++i) {
        for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it = m.BlockCols()[i].begin();
             it != m.BlockCols()[i].end(); ++it) {
            const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* b = it->second;
            os << "BLOCK: " << it->first << " " << i << std::endl;
            os << *b << std::endl;
        }
    }
    return os;
}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::FillBlockStructure(int* Cp, int* Ci) const {
    int nz = 0;
    for (int c = 0; c < static_cast<int>(block_cols_.size()); ++c) {
        *Cp = nz;
        for (auto it = block_cols_[c].begin(); it != block_cols_[c].end(); ++it) {
            const int& r = it->first;
            if (r <= c) {
                *Ci++ = r;
                ++nz;
            }
        }
        Cp++;
    }
    *Cp = nz;
    assert(nz <= static_cast<int>(NonZeroBlocks()));
}

template <class MatrixType>
int SparseBlockMatrix<MatrixType>::FillSparseBlockMatrixCCS(SparseBlockMatrixCCS<MatrixType>& blockCCS) const {
    auto& b = blockCCS.BlockCols();
    b.resize(block_cols_.size());

    int numblocks = 0;
    for (size_t i = 0; i < block_cols_.size(); ++i) {
        const IntBlockMap& row = block_cols_[i];
        auto& dest = b[i];

        dest.clear();
        dest.reserve(row.size());
        for (auto it = row.begin(); it != row.end(); ++it) {
            dest.push_back(typename SparseBlockMatrixCCS<MatrixType>::RowBlock(it->first, it->second));
            ++numblocks;
        }
    }
    return numblocks;
}

template <class MatrixType>
int SparseBlockMatrix<MatrixType>::FillSparseBlockMatrixCCSTransposed(
    SparseBlockMatrixCCS<MatrixType>& blockCCS) const {
    blockCCS.BlockCols().clear();
    blockCCS.BlockCols().resize(row_block_indices_.size());
    int numblocks = 0;
    for (size_t i = 0; i < BlockCols().size(); ++i) {
        const IntBlockMap& row = BlockCols()[i];
        for (typename IntBlockMap::const_iterator it = row.begin(); it != row.end(); ++it) {
            typename SparseBlockMatrixCCS<MatrixType>::SparseColumn& dest = blockCCS.BlockCols()[it->first];
            dest.push_back(typename SparseBlockMatrixCCS<MatrixType>::RowBlock(i, it->second));
            ++numblocks;
        }
    }
    return numblocks;
}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::TakePatternFromHash(SparseBlockMatrixHashMap<MatrixType>& hashMatrix) {
    // sort the sparse columns and add them to the map structures by
    // exploiting that we are inserting a sorted structure
    typedef std::pair<int, MatrixType*> SparseColumnPair;
    typedef typename SparseBlockMatrixHashMap<MatrixType>::SparseColumn HashSparseColumn;
    for (size_t i = 0; i < hashMatrix.BlockCols().size(); ++i) {
        // prepare a temporary vector for sorting
        HashSparseColumn& column = hashMatrix.BlockCols()[i];
        if (column.size() == 0) continue;
        std::vector<SparseColumnPair> sparseRowSorted;  // temporary structure
        sparseRowSorted.reserve(column.size());
        for (typename HashSparseColumn::const_iterator it = column.begin(); it != column.end(); ++it)
            sparseRowSorted.push_back(*it);
        std::sort(sparseRowSorted.begin(), sparseRowSorted.end(),
                  [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
        // try to free some memory early
        HashSparseColumn aux;
        std::swap(aux, column);
        // now insert sorted vector to the std::map structure
        IntBlockMap& destColumnMap = BlockCols()[i];
        destColumnMap.insert(sparseRowSorted[0]);
        for (size_t j = 1; j < sparseRowSorted.size(); ++j) {
            typename SparseBlockMatrix<MatrixType>::IntBlockMap::iterator hint = destColumnMap.end();
            --hint;  // cppreference says the element goes after the hint (until
            // C++11)
            destColumnMap.insert(hint, sparseRowSorted[j]);
        }
    }
}

template <class MatrixType>
bool SparseBlockMatrix<MatrixType>::WriteOctave(const std::string& filename, bool upperTriangle) const {
    std::string name = filename;
    std::string::size_type lastDot = name.find_last_of('.');
    if (lastDot != std::string::npos) {
        name.resize(lastDot);
    }

    struct TripletEntry {
        int r, c;
        double x;
        TripletEntry(int r_, int c_, double x_) : r(r_), c(c_), x(x_) {}
    };
    struct TripletColSort {
        bool operator()(const TripletEntry& e1, const TripletEntry& e2) const {
            return e1.c < e2.c || (e1.c == e2.c && e1.r < e2.r);
        }
    };

    std::vector<TripletEntry> entries;
    for (size_t i = 0; i < block_cols_.size(); ++i) {
        const int& c = i;
        for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it = block_cols_[i].begin();
             it != block_cols_[i].end(); ++it) {
            const int& r = it->first;
            const MatrixType& m = *(it->second);
            for (int cc = 0; cc < m.cols(); ++cc)
                for (int rr = 0; rr < m.rows(); ++rr) {
                    int aux_r = RowBaseOfBlock(r) + rr;
                    int aux_c = ColBaseOfBlock(c) + cc;
                    entries.push_back(TripletEntry(aux_r, aux_c, m(rr, cc)));
                    if (upperTriangle && r != c) {
                        entries.push_back(TripletEntry(aux_c, aux_r, m(rr, cc)));
                    }
                }
        }
    }

    int nz = entries.size();
    std::sort(entries.begin(), entries.end(), TripletColSort());

    std::ofstream fout(filename);
    fout << "# name: " << name << std::endl;
    fout << "# type: sparse matrix" << std::endl;
    fout << "# nnz: " << nz << std::endl;
    fout << "# rows: " << Rows() << std::endl;
    fout << "# columns: " << Cols() << std::endl;
    fout << std::setprecision(9) << std::fixed << std::endl;

    for (auto it = entries.begin(); it != entries.end(); ++it) {
        const TripletEntry& entry = *it;
        fout << entry.r + 1 << " " << entry.c + 1 << " " << entry.x << std::endl;
    }
    return fout.good();
}

}  // namespace lightning::miao

#endif  // MIAO_SPARSE_BLCOK_MATRIX_HPP
