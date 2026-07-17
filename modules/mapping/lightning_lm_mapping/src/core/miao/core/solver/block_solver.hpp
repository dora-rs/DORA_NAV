//
// Created by xiang on 24-4-26.
//

#include <glog/logging.h>

#include <execution>
#include "core/graph/edge.h"
#include "core/graph/vertex.h"
#include "utils/timer.h"

namespace lightning::miao {

template <typename Traits>
BlockSolver<Traits>::BlockSolver(std::unique_ptr<LinearSolverType> linearSolver)
    : Solver(), linear_solver_(std::move(linearSolver)) {
    // workspace
    x_size_ = 0;
    num_poses_ = 0;
    num_landmarks_ = 0;
    size_poses_ = 0;
    size_landmarks_ = 0;
    do_schur_ = true;
}

template <typename Traits>
void BlockSolver<Traits>::Resize(const std::vector<int>& blockPoseIndices, const std::vector<int>& blockLandmarkIndices,
                                 int s) {
    Deallocate();
    ResizeVector(s);

    if (do_schur_) {
        // the following two are only used in schur
        assert(size_poses_ > 0 && "allocating with wrong size");
        coeffs_.setZero(s);
        b_schur_.setZero(size_poses_);
    }

    Hpp_ = std::make_unique<PoseHessianType>(blockPoseIndices, blockPoseIndices);
    if (do_schur_) {
        Hschur_ = std::make_unique<PoseHessianType>(blockPoseIndices, blockPoseIndices);
        Hll_ = std::make_unique<LandmarkHessianType>(blockLandmarkIndices, blockLandmarkIndices);
        D_inv_schur_ = std::make_unique<SparseBlockMatrixDiagonal<LandmarkMatrixType>>(Hll_->ColBlockIndices());
        Hpl_ = std::make_unique<PoseLandmarkHessianType>(blockPoseIndices, blockLandmarkIndices);
        Hpl_CCS_ = std::make_unique<SparseBlockMatrixCCS<PoseLandmarkMatrixType>>(Hpl_->RowBlockIndices(),
                                                                                  Hpl_->ColBlockIndices());
        H_schur_transpose_CCS_ = std::make_unique<SparseBlockMatrixCCS<PoseMatrixType>>(Hschur_->ColBlockIndices(),
                                                                                        Hschur_->RowBlockIndices());
    }
}

template <typename Traits>
void BlockSolver<Traits>::Deallocate() {
    Hpp_.reset();
    Hll_.reset();
    Hpl_.reset();
    Hschur_.reset();
    D_inv_schur_.reset();
    coeffs_.setZero();
    b_schur_.setZero();

    Hpl_CCS_.reset();
    H_schur_transpose_CCS_.reset();
}

template <typename Traits>
BlockSolver<Traits>::~BlockSolver() = default;

template <typename Traits>
bool BlockSolver<Traits>::BuildStructure(bool zeroBlocks) {
    assert(optimizer_);
    if (optimizer_ == nullptr) {
        return false;
    }

    if (config_.incremental_mode_) {
        do_schur_ = false;
        BuildStructureInc(zeroBlocks);
    } else {
        BuildStructureFromRaw(zeroBlocks);
    }
    return true;
}

template <typename Traits>
bool BlockSolver<Traits>::BuildStructureFromRaw(bool zero_blocks) {
    size_t sparseDim = 0;
    num_poses_ = 0;
    num_landmarks_ = 0;
    size_poses_ = 0;
    size_landmarks_ = 0;

    int num_all_vertex = optimizer_->IndexMapping().size();
    std::vector<int> blockPoseIndices;
    std::vector<int> blockLandmarkIndices;
    blockPoseIndices.reserve(num_all_vertex);
    blockLandmarkIndices.reserve(num_all_vertex);

    for (const auto& v : optimizer_->IndexMapping()) {
        int dim = v->Dimension();
        if (!v->Marginalized()) {
            v->SetColInHessian(size_poses_);
            size_poses_ += dim;
            blockPoseIndices.emplace_back(size_poses_);
            ++num_poses_;
        } else {
            v->SetColInHessian(size_landmarks_);
            size_landmarks_ += dim;
            blockLandmarkIndices.emplace_back(size_landmarks_);
            ++num_landmarks_;
        }
        sparseDim += dim;
    }

    Resize(blockPoseIndices, blockLandmarkIndices, sparseDim);

    // allocate the diagonal on Hpp and Hll
    int poseIdx = 0;
    int landmarkIdx = 0;
    for (const auto& v : optimizer_->IndexMapping()) {
        if (!v->Marginalized()) {
            PoseMatrixType* m = Hpp_->Block(poseIdx, poseIdx, true);
            if (zero_blocks) {
                m->setZero();
            }

            v->MapHessianMemory(m->data());
            ++poseIdx;
        } else {
            LandmarkMatrixType* m = Hll_->Block(landmarkIdx, landmarkIdx, true);
            if (zero_blocks) {
                m->setZero();
            }

            v->MapHessianMemory(m->data());
            ++landmarkIdx;
        }
    }
    assert(poseIdx == num_poses_ && landmarkIdx == num_landmarks_);

    // temporary structures for building the pattern of the Schur complement
    SparseBlockMatrixHashMap<PoseMatrixType>* schurMatrixLookup = nullptr;
    if (do_schur_) {
        schurMatrixLookup =
            new SparseBlockMatrixHashMap<PoseMatrixType>(Hschur_->RowBlockIndices(), Hschur_->ColBlockIndices());
        schurMatrixLookup->BlockCols().resize(Hschur_->BlockCols().size());
    }

    // here we assume that the landmark indices start after the pose ones
    // create the structure in Hpp, Hll and in Hpl
    for (const auto& e : optimizer_->ActiveEdges()) {
        for (size_t viIdx = 0; viIdx < e->GetVertices().size(); ++viIdx) {
            auto v1 = e->GetVertex(viIdx);
            int ind1 = v1->HessianIndex();
            if (ind1 == -1) {
                continue;
            }

            int indexV1Bak = ind1;

            for (size_t vjIdx = viIdx + 1; vjIdx < e->GetVertices().size(); ++vjIdx) {
                auto v2 = e->GetVertex(vjIdx);
                int ind2 = v2->HessianIndex();
                if (ind2 == -1) {
                    continue;
                }

                ind1 = indexV1Bak;
                bool transposedBlock = ind1 > ind2;
                if (transposedBlock) {  // make sure, we allocate the upper triangle
                    // block
                    std::swap(ind1, ind2);
                }

                if (!v1->Marginalized() && !v2->Marginalized()) {
                    PoseMatrixType* m = Hpp_->Block(ind1, ind2, true);
                    if (zero_blocks) {
                        m->setZero();
                    }

                    e->MapHessianMemory(m->data(), viIdx, vjIdx, transposedBlock);

                    if (Hschur_) {
                        // assume this is only needed in case we solve with
                        // the schur complement
                        schurMatrixLookup->AddBlock(ind1, ind2);
                    }
                } else if (v1->Marginalized() && v2->Marginalized()) {
                    // RAINER hmm.... should we ever reach this here????
                    LandmarkMatrixType* m = Hll_->Block(ind1 - num_poses_, ind2 - num_poses_, true);
                    if (zero_blocks) {
                        m->setZero();
                    }
                    e->MapHessianMemory(m->data(), viIdx, vjIdx, false);
                } else {
                    if (v1->Marginalized()) {
                        PoseLandmarkMatrixType* m =
                            Hpl_->Block(v2->HessianIndex(), v1->HessianIndex() - num_poses_, true);
                        if (zero_blocks) {
                            m->setZero();
                        }

                        e->MapHessianMemory(m->data(), viIdx, vjIdx, true);  // transpose the block before writing to it
                    } else {
                        PoseLandmarkMatrixType* m =
                            Hpl_->Block(v1->HessianIndex(), v2->HessianIndex() - num_poses_, true);
                        if (zero_blocks) {
                            m->setZero();
                        }

                        e->MapHessianMemory(m->data(), viIdx, vjIdx, false);  // directly the block
                    }
                }
            }
        }
    }

    if (!do_schur_) {
        delete schurMatrixLookup;
        return true;
    }

    D_inv_schur_->Diagonal().resize(landmarkIdx);
    Hpl_->FillSparseBlockMatrixCCS(*Hpl_CCS_);

    /// 啧啧
    for (const auto& v : optimizer_->IndexMapping()) {
        if (!v->Marginalized()) {
            continue;
        }

        const auto& vedges = v->GetEdges();
        for (auto& e : vedges) {
            for (size_t i = 0; i < e->GetVertices().size(); ++i) {
                auto v1 = e->GetVertex(i);
                if (v1->HessianIndex() == -1 || v1 == v) {
                    continue;
                }
                for (auto& e2 : vedges) {
                    for (size_t j = 0; j < e2->GetVertices().size(); ++j) {
                        auto v2 = e2->GetVertex(j);
                        if (v2->HessianIndex() == -1 || v2 == v) {
                            continue;
                        }

                        int i1 = v1->HessianIndex();
                        int i2 = v2->HessianIndex();
                        if (i1 <= i2) {
                            schurMatrixLookup->AddBlock(i1, i2);
                        }
                    }
                }
            }
        }
    }

    Hschur_->TakePatternFromHash(*schurMatrixLookup);
    delete schurMatrixLookup;
    Hschur_->FillSparseBlockMatrixCCSTransposed(*H_schur_transpose_CCS_);

    return true;
}

template <typename Traits>
bool BlockSolver<Traits>::BuildStructureInc(bool zero_blocks) {
    /// 增量模式下没有landmark，且省略历史过程中的index building
    size_t sparseDim = size_poses_;

    int num_all_vertex = optimizer_->IndexMapping().size();

    /// for inc
    std::vector<int> new_block_pose_indices;
    new_block_pose_indices.reserve(num_all_vertex);

    int last_num_poses = num_poses_;
    for (const auto& v : optimizer_->NewVertices()) {
        int dim = v->Dimension();
        assert(v->Marginalized() == false);
        if (v->Fixed()) {
            continue;
        }
        v->SetColInHessian(size_poses_);
        size_poses_ += dim;
        new_block_pose_indices.emplace_back(size_poses_);
        ++num_poses_;
        sparseDim += dim;
    }

    ResizeVector(sparseDim);

    if (Hpp_ == nullptr) {
        Hpp_ = std::make_unique<PoseHessianType>(new_block_pose_indices, new_block_pose_indices);
    } else {
        Hpp_->SetBlockIndexInc(new_block_pose_indices, new_block_pose_indices);

        if (config_.incremental_mode_ && config_.max_vertex_size_ >= 0) {
            /// Hpp 不清空的话影响lambda之类的求解
            Hpp_->Clear();
        }
    }

    // allocate the diagonal on Hpp and Hll
    int poseIdx = last_num_poses;
    for (const auto& v : optimizer_->NewVertices()) {
        if (v->Fixed()) {
            continue;
        }
        PoseMatrixType* m = Hpp_->Block(poseIdx, poseIdx, true);
        if (zero_blocks) {
            m->setZero();
        }

        v->MapHessianMemory(m->data());
        ++poseIdx;
    }
    assert(poseIdx == num_poses_);

    for (const auto& e : optimizer_->NewEdges()) {
        for (size_t vi_idx = 0; vi_idx < e->GetVertices().size(); ++vi_idx) {
            auto v1 = e->GetVertex(vi_idx);
            int ind1 = v1->HessianIndex();
            if (ind1 == -1) {
                continue;
            }

            int indexV1Bak = ind1;

            for (size_t vj_idx = vi_idx + 1; vj_idx < e->GetVertices().size(); ++vj_idx) {
                auto v2 = e->GetVertex(vj_idx);
                int ind2 = v2->HessianIndex();
                if (ind2 == -1) {
                    continue;
                }

                ind1 = indexV1Bak;
                bool transposedBlock = ind1 > ind2;
                if (transposedBlock) {  // make sure, we allocate the upper triangle
                    // block
                    std::swap(ind1, ind2);
                }

                PoseMatrixType* m = Hpp_->Block(ind1, ind2, true);
                if (zero_blocks) {
                    m->setZero();
                }

                e->MapHessianMemory(m->data(), vi_idx, vj_idx, transposedBlock);
            }
        }
    }

    optimizer_->ClearNewElements();

    return true;
}

template <typename Traits>
bool BlockSolver<Traits>::Solve() {
    if (!do_schur_) {
        return linear_solver_->Solve(*Hpp_, x_.data(), b_.data());
    }

    // schur thing
    Hschur_->Clear();
    Hpp_->Add(*Hschur_);

    coeffs_.setZero();

    std::vector<int> landmark_idx_vec(Hll_->BlockCols().size());
    std::iota(landmark_idx_vec.begin(), landmark_idx_vec.end(), 0);

    std::vector<std::mutex> mutex_b(num_landmarks_);
    std::vector<std::mutex> mutex_H(num_poses_ * num_poses_);

    std::for_each(std::execution::par_unseq, landmark_idx_vec.begin(), landmark_idx_vec.end(), [&](int landmarkIndex) {
        const typename SparseBlockMatrix<LandmarkMatrixType>::IntBlockMap& marginalizeColumn =
            Hll_->BlockCols()[landmarkIndex];
        assert(marginalizeColumn.size() == 1 && "more than one Block in _Hll column");

        // calculate inverse block for the landmark
        const LandmarkMatrixType* D = marginalizeColumn.begin()->second;
        assert(D && D->rows() == D->cols() && "Error in landmark matrix");
        LandmarkMatrixType& Dinv = D_inv_schur_->Diagonal()[landmarkIndex];
        Dinv = D->inverse();

        LandmarkVectorType db(D->rows());
        for (int j = 0; j < D->rows(); ++j) {
            db[j] = b_[Hll_->RowBaseOfBlock(landmarkIndex) + size_poses_ + j];
        }
        db = Dinv * db;

        assert((size_t)landmarkIndex < Hpl_CCS_->BlockCols().size() && "Index out of bounds");
        const typename SparseBlockMatrixCCS<PoseLandmarkMatrixType>::SparseColumn& landmarkColumn =
            Hpl_CCS_->BlockCols()[landmarkIndex];

        for (auto it_outer = landmarkColumn.begin(); it_outer != landmarkColumn.end(); ++it_outer) {
            int i1 = it_outer->row;

            const PoseLandmarkMatrixType* Bi = it_outer->block;
            assert(Bi);

            PoseLandmarkMatrixType BDinv = (*Bi) * (Dinv);

            // 右侧b部分
            {
                int b_index = Hpl_CCS_->RowBaseOfBlock(i1);
                assert(b_index >= 0 && b_index < num_landmarks_);

                lightning::UL lock(mutex_b[b_index]);
                typename PoseVectorType::MapType Bb(&coeffs_[b_index], Bi->rows());
                Bb.noalias() += (*Bi) * db;
            }

            assert(i1 >= 0 && i1 < static_cast<int>(H_schur_transpose_CCS_->BlockCols().size()) &&
                   "Index out of bounds");
            typename SparseBlockMatrixCCS<PoseMatrixType>::SparseColumn::iterator targetColumnIt =
                H_schur_transpose_CCS_->BlockCols()[i1].begin();

            typename SparseBlockMatrixCCS<PoseLandmarkMatrixType>::RowBlock aux(i1, 0);
            typename SparseBlockMatrixCCS<PoseLandmarkMatrixType>::SparseColumn::const_iterator it_inner =
                lower_bound(landmarkColumn.begin(), landmarkColumn.end(), aux);
            for (; it_inner != landmarkColumn.end(); ++it_inner) {
                int i2 = it_inner->row;
                const PoseLandmarkMatrixType* Bj = it_inner->block;
                assert(Bj);
                while (targetColumnIt->row < i2 /*&& targetColumnIt != _HschurTransposedCCS->blockCols()[i1].end()*/) {
                    ++targetColumnIt;
                }

                assert(targetColumnIt != H_schur_transpose_CCS_->BlockCols()[i1].end() && targetColumnIt->row == i2 &&
                       "invalid iterator, something wrong with the matrix structure");
                lightning::UL lock(mutex_H[i1 * num_poses_ + i2]);
                PoseMatrixType* Hi1i2 = targetColumnIt->block;
                assert(Hi1i2);
                (*Hi1i2).noalias() -= BDinv * Bj->transpose();
            }
        }
    });

    memcpy(b_schur_.data(), b_.data(), size_poses_ * sizeof(double));
    for (int i = 0; i < size_poses_; ++i) {
        b_schur_[i] -= coeffs_[i];
    }

    /// linear solver 只解pose部分
    bool solvedPoses = true;
    solvedPoses = linear_solver_->Solve(*Hschur_, x_.data(), b_schur_.data());

    if (!solvedPoses) {
        return false;
    }

    // _x contains the solution for the poses, now applying it to the landmarks to
    // get the new part of the solution;
    double* xp = x_.data();
    double* cp = coeffs_.data();

    double* xl = x_.data() + size_poses_;
    double* cl = coeffs_.data() + size_poses_;
    double* bl = b_.data() + size_poses_;

    // cp = -xp
    for (int i = 0; i < size_poses_; ++i) {
        cp[i] = -xp[i];
    }

    // cl = bl
    memcpy(cl, bl, size_landmarks_ * sizeof(double));

    // cl = bl - Bt * xp
    // Bt->multiply(cl, cp);
    Hpl_CCS_->RightMultiply(cl, cp);

    // xl = Dinv * cl
    memset(xl, 0, size_landmarks_ * sizeof(double));
    D_inv_schur_->Multiply(xl, cl);

    return true;
}

template <typename Traits>
bool BlockSolver<Traits>::BuildSystem() {
    Hpp_->Clear();

    if (config_.parallel_) {
        // clear b vector
        std::for_each(std::execution::par_unseq, optimizer_->IndexMapping().begin(), optimizer_->IndexMapping().end(),
                      [](const auto& v) { v->ClearQuadraticForm(); });

        std::for_each(std::execution::par_unseq, optimizer_->ActiveEdges().begin(), optimizer_->ActiveEdges().end(),
                      [](const auto& e) {
                          /// 线性化，计算jacobian，可以并行
                          e->LinearizeOplus();
                          // 求AtA 和 AtB ，这个也可以并行，但需要锁每个block
                          e->ConstructQuadraticForm();
                          // 将边的Hessian拷回大的hessian阵
                          e->CopyHessianToSolver();
                      });

        // 将顶点的hessian拷回大的hessian阵
        std::for_each(std::execution::par_unseq, optimizer_->ActiveVertices().begin(),
                      optimizer_->ActiveVertices().end(), [](const auto& v) { v->CopyHessianToSolver(); });

    } else {
        // clear b vector
        std::for_each(std::execution::seq, optimizer_->IndexMapping().begin(), optimizer_->IndexMapping().end(),
                      [](const auto& v) { v->ClearQuadraticForm(); });

        std::for_each(std::execution::seq, optimizer_->ActiveEdges().begin(), optimizer_->ActiveEdges().end(),
                      [](const auto& e) {
                          /// 线性化，计算jacobian，可以并行
                          e->LinearizeOplus();
                          // 求AtA 和 AtB ，这个也可以并行，但需要锁每个block
                          e->ConstructQuadraticForm();
                          // 将边的Hessian拷回大的hessian阵
                          e->CopyHessianToSolver();
                      });

        // 将顶点的hessian拷回大的hessian阵
        std::for_each(std::execution::seq, optimizer_->ActiveVertices().begin(), optimizer_->ActiveVertices().end(),
                      [](const auto& v) { v->CopyHessianToSolver(); });
    }

    // flush the current system in a sparse block matrix
    // 把每个vertex部分中的b拷至整个b中
    for (const auto& v : optimizer_->IndexMapping()) {
        int iBase = v->ColInHessian();
        if (v->Marginalized()) {
            iBase += size_poses_;
        }

        v->CopyB(b_.data() + iBase);
    }

    return true;
}

template <typename Traits>
bool BlockSolver<Traits>::SetLambda(double lambda, bool backup) {
    if (backup) {
        diagonal_backup_pose_.resize(num_poses_);
        diagonal_backup_landmark_.resize(num_landmarks_);
    }

    for (int i = 0; i < num_poses_; ++i) {
        PoseMatrixType* b = Hpp_->Block(i, i);
        assert(b != nullptr);
        if (backup) {
            diagonal_backup_pose_[i] = b->diagonal();
        }
        b->diagonal().array() += lambda;
    }

    for (int i = 0; i < num_landmarks_; ++i) {
        LandmarkMatrixType* b = Hll_->Block(i, i);
        if (backup) {
            diagonal_backup_landmark_[i] = b->diagonal();
        }

        b->diagonal().array() += lambda;
    }
    return true;
}

template <typename Traits>
void BlockSolver<Traits>::RestoreDiagonal() {
    assert((int)diagonal_backup_pose_.size() == num_poses_ && "Mismatch in dimensions");
    assert((int)diagonal_backup_landmark_.size() == num_landmarks_ && "Mismatch in dimensions");
    for (int i = 0; i < num_poses_; ++i) {
        PoseMatrixType* b = Hpp_->Block(i, i);
        b->diagonal() = diagonal_backup_pose_[i];
    }
    for (int i = 0; i < num_landmarks_; ++i) {
        LandmarkMatrixType* b = Hll_->Block(i, i);
        b->diagonal() = diagonal_backup_landmark_[i];
    }
}

template <typename Traits>
bool BlockSolver<Traits>::Init(Optimizer* optimizer) {
    optimizer_ = optimizer;
    config_ = optimizer->GetConfig();

    if (!config_.incremental_mode_) {
        /// 不在增量模式下，就清空整个Hessian
        if (Hpp_) {
            Hpp_->Clear();
        }
        if (Hpl_) {
            Hpl_->Clear();
        }
        if (Hll_) {
            Hll_->Clear();
        }
    }

    linear_solver_->Init();
    return true;
}

}  // namespace lightning::miao
