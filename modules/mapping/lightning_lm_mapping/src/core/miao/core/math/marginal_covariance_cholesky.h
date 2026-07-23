//
// Created by xiang on 24-4-26.
//

#ifndef MIAO_MARGINAL_COVARIANCE_CHOLESKY_H
#define MIAO_MARGINAL_COVARIANCE_CHOLESKY_H

#include <unordered_map>
#include <vector>

#include "common/eigen_types.h"
#include "core/sparse/sparse_block_matrix.h"

namespace lightning::miao {

/**
 * \brief computing the marginal covariance given a cholesky factor (lower
 * triangle of the factor)
 *
 * 这个主要给LinearSolverCCS 用
 */
class MarginalCovarianceCholesky {
   protected:
    /**
     * hash struct for storing the matrix elements needed to compute the
     * covariance
     */
    typedef std::unordered_map<int, double> LookupMap;

   public:
    MarginalCovarianceCholesky();
    ~MarginalCovarianceCholesky();

    /**
     * compute the marginal cov for the given Block indices, write the result to
     * the covBlocks memory (which has to be provided by the caller).
     */
    void ComputeCovariance(double** covBlocks, const std::vector<int>& blockIndices);

    /**
     * compute the marginal cov for the given Block indices, write the result in
     * spinv).
     */
    void ComputeCovariance(SparseBlockMatrix<lightning::MatrixX>& spinv, const std::vector<int>& rowBlockIndices,
                           const std::vector<std::pair<int, int> >& blockIndices);

    /**
     * set the CCS representation of the cholesky factor along with the inverse
     * permutation used to reduce the fill-in. permInv might be 0, will then not
     * permute the entries.
     *
     * The pointers provided by the user need to be still valid when calling
     * ComputeCovariance(). The pointers are owned by the caller,
     * MarginalCovarianceCholesky does not free the pointers.
     */
    void SetCholeskyFactor(int n, int* Lp, int* Li, double* Lx, int* permInv);

   protected:
    // information about the cholesky factor (lower triangle)
    int n_ = 0;             ///< L is an n X n matrix
    int* Ap_ = nullptr;     ///< column pointer of the CCS storage
    int* Ai_ = nullptr;     ///< row indices of the CCS storage
    double* Ax_ = nullptr;  ///< values of the cholesky factor
    int* perm_ = nullptr;   ///< permutation of the cholesky factor. Variable re-ordering for  better fill-in

    LookupMap map_;             ///< hash look up table for the already computed entries
    std::vector<double> diag_;  ///< cache 1 / H_ii to avoid recalculations

    //! compute the index used for hashing
    int ComputeIndex(int r, int c) const { /*assert(r <= c);*/ return r * n_ + c; }

    /**
     * compute one entry in the covariance, r and c are values after applying the
     * permutation, and upper triangular. May issue recursive calls to itself to
     * compute the missing values.
     */
    double ComputeEntry(int r, int c);
};

}  // namespace lightning::miao

#endif  // MIAO_MARGINAL_COVARIANCE_CHOLESKY_H
