//
// Created by xiang on 24-5-13.
//

#ifndef MIAO_LINEAR_SOLVER_EIGEN_H
#define MIAO_LINEAR_SOLVER_EIGEN_H

#include <glog/logging.h>
#include <Eigen/Sparse>

#include "linear_solver_ccs.h"

#include "utils/timer.h"

namespace lightning::miao {

/**
 * \brief linear solver which uses the sparse Cholesky Solver from Eigen
 *
 * Has no dependencies except Eigen. Hence, should compile almost everywhere
 * without to much issues. Performance should be similar to CSparse.
 */
template <typename MatrixType>
class LinearSolverEigen : public LinearSolverCCS<MatrixType> {
   public:
    typedef Eigen::SparseMatrix<double> SparseMatrix;
    typedef Eigen::Triplet<double> Triplet;
    typedef Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> PermutationMatrix;

    using CholeskyDecompositionBase = Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper>;

    /**
     * \brief Sub-classing Eigen's SimplicialLLT to perform ordering with a given
     * ordering
     */
    class CholeskyDecomposition : public CholeskyDecompositionBase {
       public:
        CholeskyDecomposition() : CholeskyDecompositionBase() {}

        //! use a given permutation for analyzing the pattern of the sparse matrix
        void AnalyzePatternWithPermutation(SparseMatrix &a, const PermutationMatrix &permutation) {
            m_Pinv = permutation;
            m_P = permutation.inverse();
            int size = a.cols();
            SparseMatrix ap(size, size);
            ap.selfadjointView<Eigen::Upper>() = a.selfadjointView<UpLo>().twistedBy(m_P);
            analyzePattern_preordered(ap, false);
        }

       protected:
        using CholeskyDecompositionBase::analyzePattern_preordered;
    };

   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LinearSolverEigen() : LinearSolverCCS<MatrixType>() {}

    bool Init() override {
        init_ = true;
        return true;
    }

    bool Solve(const SparseBlockMatrix<MatrixType> &A, double *x, double *b) override {
        bool cholState = false;
        cholState = ComputeCholesky(A);

        if (!cholState) {
            return false;
        }

        // Solving the system
        VectorX::MapType xx(x, sparse_matrix_.cols());
        VectorX::ConstMapType bb(b, sparse_matrix_.cols());

        xx = cholesky_.solve(bb);

        return true;
    }

   protected:
    bool init_ = true;
    SparseMatrix sparse_matrix_;
    CholeskyDecomposition cholesky_;

    // compute the cholesky factor
    bool ComputeCholesky(const SparseBlockMatrix<MatrixType> &A) {
        // perform some operations only once upon init
        if (init_) {
            sparse_matrix_.resize(A.Rows(), A.Cols());
        }

         FillSparseMatrix(A, !init_);

        if (init_) {
            ComputeSymbolicDecomposition(A);
        }
        init_ = false;

        cholesky_.factorize(sparse_matrix_);

        if (cholesky_.info() != Eigen::Success) {  // the matrix is not positive definite
            LOG(ERROR) << "error : Cholesky failure, solve failed---------------";
            return false;
        }
        return true;
    }

    /**
     * compute the symbolic decomposition of the matrix only once.
     * Since A has the same pattern in all the iterations, we only
     * compute the fill-in reducing ordering once and re-use for all
     * the following iterations.
     */
    void ComputeSymbolicDecomposition(const SparseBlockMatrix<MatrixType> &A) {
        if (!this->BlockOrdering()) {
            cholesky_.analyzePattern(sparse_matrix_);
        } else {
            assert(A.Rows() == A.Cols() && "Matrix A is not square");
            // block ordering with the Eigen Interface
            Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> blockP;
            {
                SparseMatrix auxBlockMatrix(A.BlockCols().size(), A.BlockCols().size());
                auxBlockMatrix.resizeNonZeros(A.NonZeroBlocks());
                // fill the CCS structure of the Eigen SparseMatrix
                A.FillBlockStructure(auxBlockMatrix.outerIndexPtr(), auxBlockMatrix.innerIndexPtr());
                // determine ordering by AMD
                using Ordering = Eigen::AMDOrdering<SparseMatrix::StorageIndex>;
                Ordering ordering;
                ordering(auxBlockMatrix, blockP);
            }

            // Adapt the block permutation to the scalar matrix
            PermutationMatrix scalarP(A.Rows());
            this->BlockToScalarPermutation(A, blockP.indices(), scalarP.indices());
            // analyze with the scalar permutation
            cholesky_.AnalyzePatternWithPermutation(sparse_matrix_, scalarP);
        }
    }

    void FillSparseMatrix(const SparseBlockMatrix<MatrixType> &A, bool onlyValues) {
        if (onlyValues) {
            this->ccs_matrix_->FillCCS(sparse_matrix_.valuePtr(), true);
            return;
        }

        this->InitMatrixStructure(A);
        sparse_matrix_.resizeNonZeros(A.NonZeros());
        int nz = this->ccs_matrix_->FillCCS(sparse_matrix_.outerIndexPtr(), sparse_matrix_.innerIndexPtr(),
                                            sparse_matrix_.valuePtr(), true);
        assert(nz <= static_cast<int>(sparse_matrix_.data().size()));
    }
};

}  // namespace lightning::miao

#endif  // MIAO_LINEAR_SOLVER_EIGEN_H
