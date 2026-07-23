//
// Created by xiang on 24-4-26.
//

#include "marginal_covariance_cholesky.h"

namespace lightning::miao {

struct MatrixElem {
    int r, c;
    MatrixElem(int r_, int c_) : r(r_), c(c_) {}
    bool operator<(const MatrixElem& other) const { return c > other.c || (c == other.c && r > other.r); }
};

MarginalCovarianceCholesky::MarginalCovarianceCholesky() = default;

MarginalCovarianceCholesky::~MarginalCovarianceCholesky() = default;

void MarginalCovarianceCholesky::SetCholeskyFactor(int n, int* Lp, int* Li, double* Lx, int* permInv) {
    n_ = n;
    Ap_ = Lp;
    Ai_ = Li;
    Ax_ = Lx;
    perm_ = permInv;

    // pre-compute reciprocal values of the diagonal of L
    diag_.resize(n);
    for (int r = 0; r < n; ++r) {
        const int& sc = Ap_[r];  // L is lower triangular, thus the first elem in
        // the column is the diagonal entry
        assert(r == Ai_[sc] && "Error in CCS storage of L");
        diag_[r] = 1.0 / Ax_[sc];
    }
}

double MarginalCovarianceCholesky::ComputeEntry(int r, int c) {
    assert(r <= c);
    int idx = ComputeIndex(r, c);

    auto foundIt = map_.find(idx);
    if (foundIt != map_.end()) {
        return foundIt->second;
    }

    // compute the summation over column r
    double s = 0.;
    const int& sc = Ap_[r];
    const int& ec = Ap_[r + 1];
    for (int j = sc + 1; j < ec; ++j) {  // sum over row r while skipping the element on the diagonal
        const int& rr = Ai_[j];
        double val = rr < c ? ComputeEntry(rr, c) : ComputeEntry(c, rr);
        s += val * Ax_[j];
    }

    double result;
    if (r == c) {
        const double& diagElem = diag_[r];
        result = diagElem * (diagElem - s);
    } else {
        result = -s * diag_[r];
    }

    map_[idx] = result;
    return result;
}

void MarginalCovarianceCholesky::ComputeCovariance(double** covBlocks, const std::vector<int>& blockIndices) {
    map_.clear();
    int base = 0;
    std::vector<MatrixElem> elemsToCompute;
    for (size_t i = 0; i < blockIndices.size(); ++i) {
        int nbase = blockIndices[i];
        int vdim = nbase - base;
        for (int rr = 0; rr < vdim; ++rr) {
            for (int cc = rr; cc < vdim; ++cc) {
                int r = perm_ ? perm_[rr + base] : rr + base;  // apply permutation
                int c = perm_ ? perm_[cc + base] : cc + base;
                if (r > c) {
                    // make sure it's still upper triangular after applying the permutation
                    std::swap(r, c);
                }
                elemsToCompute.emplace_back(r, c);
            }
        }
        base = nbase;
    }

    // sort the elems to reduce the recursive calls
    sort(elemsToCompute.begin(), elemsToCompute.end());

    // compute the inverse elements we need
    for (auto me : elemsToCompute) {
        ComputeEntry(me.r, me.c);
    }

    // set the marginal covariance for the vertices, by writing to the blocks
    // memory
    base = 0;
    for (size_t i = 0; i < blockIndices.size(); ++i) {
        int nbase = blockIndices[i];
        int vdim = nbase - base;
        double* cov = covBlocks[i];
        for (int rr = 0; rr < vdim; ++rr) {
            for (int cc = rr; cc < vdim; ++cc) {
                int r = perm_ ? perm_[rr + base] : rr + base;  // apply permutation
                int c = perm_ ? perm_[cc + base] : cc + base;
                if (r > c) {
                    // upper triangle
                    std::swap(r, c);
                }

                int idx = ComputeIndex(r, c);

                auto foundIt = map_.find(idx);
                assert(foundIt != map_.end());
                cov[rr * vdim + cc] = foundIt->second;
                if (rr != cc) {
                    cov[cc * vdim + rr] = foundIt->second;
                }
            }
        }
        base = nbase;
    }
}

void MarginalCovarianceCholesky::ComputeCovariance(SparseBlockMatrix<lightning::MatrixX>& spinv,
                                                   const std::vector<int>& rowBlockIndices,
                                                   const std::vector<std::pair<int, int> >& blockIndices) {
    // allocate the sparse
    spinv = SparseBlockMatrix<lightning::MatrixX>(&rowBlockIndices[0], &rowBlockIndices[0], rowBlockIndices.size(),
                                                  rowBlockIndices.size(), true);
    map_.clear();
    std::vector<MatrixElem> elemsToCompute;

    for (size_t i = 0; i < blockIndices.size(); ++i) {
        int blockRow = blockIndices[i].first;
        int blockCol = blockIndices[i].second;
        assert(blockRow >= 0);
        assert(blockRow < (int)rowBlockIndices.size());
        assert(blockCol >= 0);
        assert(blockCol < (int)rowBlockIndices.size());

        int rowBase = spinv.RowBaseOfBlock(blockRow);
        int colBase = spinv.ColBaseOfBlock(blockCol);

        lightning::MatrixX* block = spinv.Block(blockRow, blockCol, true);
        assert(block);
        for (int iRow = 0; iRow < block->rows(); ++iRow)
            for (int iCol = 0; iCol < block->cols(); ++iCol) {
                int rr = rowBase + iRow;
                int cc = colBase + iCol;
                int r = perm_ ? perm_[rr] : rr;  // apply permutation
                int c = perm_ ? perm_[cc] : cc;
                if (r > c) {
                    std::swap(r, c);
                }

                elemsToCompute.push_back(MatrixElem(r, c));
            }
    }

    // sort the elems to reduce the number of recursive calls
    std::sort(elemsToCompute.begin(), elemsToCompute.end());

    // compute the inverse elements we need
    for (const auto& me : elemsToCompute) {
        ComputeEntry(me.r, me.c);
    }

    // set the marginal covariance
    for (size_t i = 0; i < blockIndices.size(); ++i) {
        int blockRow = blockIndices[i].first;
        int blockCol = blockIndices[i].second;
        int rowBase = spinv.RowBaseOfBlock(blockRow);
        int colBase = spinv.ColBaseOfBlock(blockCol);

        lightning::MatrixX* block = spinv.Block(blockRow, blockCol);
        assert(block);
        for (int iRow = 0; iRow < block->rows(); ++iRow)
            for (int iCol = 0; iCol < block->cols(); ++iCol) {
                int rr = rowBase + iRow;
                int cc = colBase + iCol;
                int r = perm_ ? perm_[rr] : rr;  // apply permutation
                int c = perm_ ? perm_[cc] : cc;
                if (r > c) {
                    std::swap(r, c);
                }
                int idx = ComputeIndex(r, c);
                auto foundIt = map_.find(idx);
                assert(foundIt != map_.end());
                (*block)(iRow, iCol) = foundIt->second;
            }
    }
}

}  // namespace lightning::miao