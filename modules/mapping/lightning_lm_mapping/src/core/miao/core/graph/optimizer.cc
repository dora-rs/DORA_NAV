//
// Created by xiang on 24-4-23.
//

#include "optimizer.h"
#include "vertex.h"

#include "core/graph/edge.h"
#include "core/opti_algo/optimization_algorithm.h"
#include "core/robust_kernel/robust_kernel.h"

#include <glog/logging.h>
#include <execution>

namespace lightning::miao {

Optimizer::Optimizer() {}

Optimizer::~Optimizer() { algorithm_ = nullptr; }

bool Optimizer::InitializeOptimization(int level) {
    if (!config_.incremental_mode_) {
        InitFromRaw(level);
        return true;
    }

    InitFromLast(level);

    return true;
}

void Optimizer::InitFromRaw(int level) {
    std::set<Vertex *> vset;
    for (const auto &vp : vertices_) {
        vset.insert(vp.second.get());
    }

    if (edges_.empty()) {
        LOG(WARNING) << "Attempt to initialize an empty graph";
        return;
    }

    ClearIndexMapping();
    active_vertices_.clear();
    active_vertices_.reserve(vset.size());
    active_edges_.clear();

    std::set<Edge *> auxEdgeSet;  // temporary structure to avoid duplicates

    for (const auto &v : vset) {
        const auto &vEdges = v->GetEdges();

        // count if there are edges in that level. If not remove from the pool
        int levelEdges = 0;
        for (auto &e : vEdges) {
            if (level < 0 || e->Level() == level) {
                bool allVerticesOK = true;
                for (const auto &vit : e->GetVertices()) {
                    if (vset.find(vit) == vset.end()) {
                        allVerticesOK = false;
                        break;
                    }
                }

                if (allVerticesOK && !e->AllVerticesFixed()) {
                    auxEdgeSet.insert(e);
                    levelEdges++;
                }
            }
        }

        if (levelEdges) {
            active_vertices_.push_back(v);
        }
    }

    active_edges_.reserve(auxEdgeSet.size());
    for (const auto &e : auxEdgeSet) {
        active_edges_.push_back(e);
    }

    SortVectorContainers();
    BuildIndexMapping(active_vertices_);
}

void Optimizer::InitFromLast(int level) {
    /// 检查所有顶点都不应该有marg的部分
    for (const auto &v : vertices_) {
        if (v.second->Marginalized()) {
            LOG(ERROR) << "should not use marginalization in incremental mode. ";
            return;
        }
    }

    for (const auto &v : new_vertices_) {
        active_vertices_.emplace_back(v);

        if (v->Fixed()) {
            v->SetHessianIndex(-1);
        } else {
            v->SetHessianIndex(iv_map_.size());
            iv_map_.emplace_back(v);
        }
    }

    for (const auto &e : new_edges_) {
        if (e->Level() == level) {
            active_edges_.emplace_back(e.get());
        }
    }

    SortVectorContainers();
}

bool Optimizer::BuildIndexMapping(const std::vector<Vertex *> &vlist) {
    if (!vlist.size()) {
        iv_map_.clear();
        return false;
    }

    iv_map_.resize(vlist.size());
    size_t i = 0;
    for (int k = 0; k < 2; k++) {
        for (const auto &v : vlist) {
            if (!v->Fixed()) {
                if (static_cast<int>(v->Marginalized()) == k) {
                    v->SetHessianIndex(i);
                    iv_map_[i] = v;
                    i++;
                }
            } else {
                v->SetHessianIndex(-1);
            }
        }
    }

    iv_map_.resize(i);
    return true;
}

void Optimizer::ClearIndexMapping() {
    for (auto &iv : iv_map_) {
        iv->SetHessianIndex(-1);
        iv = nullptr;
    }
}

void Optimizer::SortVectorContainers() {
    std::sort(active_vertices_.begin(), active_vertices_.end(),
              [](const Vertex *v1, const Vertex *v2) { return v1->GetId() < v2->GetId(); });
    std::sort(active_edges_.begin(), active_edges_.end(),
              [](const Edge *e1, const Edge *e2) { return e1->GetInternalId() < e2->GetInternalId(); });
}

int Optimizer::Optimize(int iterations) {
    if (iv_map_.empty()) {
        LOG(WARNING) << "0 vertices to Optimize, maybe forgot to call "
                        "InitializeOptimization()";
        /// NOTE: 这个可以帮忙调，不用每次让用户调
        return -1;
    }

    int cjIterations = 0;
    bool ok = true;

    ok = algorithm_->Init();
    if (!ok) {
        LOG(ERROR) << "Error while initializing";
        return -1;
    }

    auto result = OptimizationAlgorithm::SolverResult::OK;
    double chi2 = 0;

    for (int i = 0; i < iterations && !Terminate() && ok; i++) {
        result = algorithm_->Solve(i);
        ok = (result == OptimizationAlgorithm::SolverResult::OK);

        ComputeActiveErrors();
        double this_chi2 = ActiveRobustChi2();

        if (Verbose()) {
            LOG(INFO) << "iteration= " << i << "\t chi2= " << std::fixed << this_chi2
                      << "\t edges= " << active_edges_.size();
        }

        if (i > 0 && fabs(this_chi2 - chi2) < config_.eps_chi2_) {
            break;
        }

        chi2 = this_chi2;

        ++cjIterations;
    }
    if (result == OptimizationAlgorithm::SolverResult::Fail) {
        return 0;
    }
    return cjIterations;
}

double Optimizer::ActiveChi2() const {
    double chi = 0.0;
    for (const auto &e : active_edges_) {
        chi += e->Chi2();
    }
    return chi;
}

double Optimizer::ActiveRobustChi2() const {
    lightning::Vector3 rho;
    double chi = 0.0;
    for (const auto &e : active_edges_) {
        if (e->GetRobustKernel()) {
            e->GetRobustKernel()->Robustify(e->Chi2(), rho);
            chi += rho[0];
        } else {
            chi += e->Chi2();
        }
    }
    return chi;
}

bool Optimizer::RemoveVertex(std::shared_ptr<Vertex> v, bool detach) {
    if (!config_.incremental_mode_ && v->HessianIndex() >= 0) {
        ClearIndexMapping();
        iv_map_.clear();
    } else {
        // 增量模式下不需要重建index mapping
    }

    return Graph::RemoveVertex(v, detach);
}

void Optimizer::SetAlgorithm(std::shared_ptr<OptimizationAlgorithm> algorithm) {
    if (algorithm_) {
        algorithm_->SetOptimizer(nullptr);
        algorithm_ = nullptr;
    }

    algorithm_ = algorithm;
    algorithm_->SetOptimizer(this);
}

void Optimizer::Push() {
    std::for_each(std::execution::par_unseq, active_vertices_.begin(), active_vertices_.end(),
                  [](const auto &v) { v->Push(); });
}

void Optimizer::Pop() {
    std::for_each(std::execution::par_unseq, active_vertices_.begin(), active_vertices_.end(),
                  [](const auto &v) { v->Pop(); });
}

void Optimizer::DiscardTop() {
    std::for_each(std::execution::par_unseq, active_vertices_.begin(), active_vertices_.end(),
                  [](const auto &v) { v->DiscardTop(); });
}

void Optimizer::Clear() {
    iv_map_.clear();
    active_vertices_.clear();
    active_edges_.clear();
    new_vertices_.clear();
    new_edges_.clear();
    vertices_id_deque_.clear();
    Graph::Clear();
}

void Optimizer::ComputeActiveErrors() {
    for (const auto &e : active_edges_) {
        e->ComputeError();
    }
}

void Optimizer::Update(const double *update) {
    // update the graph by calling oplus on the vertices
    for (const auto &v : iv_map_) {
        Eigen::Map<const lightning::Vector6> upd(update);
        // LOG(INFO) << "vert: " << v->GetId() << ", upd: " << upd.transpose();

        v->Oplus(update);
        update += v->Dimension();
    }
}

bool Optimizer::AddVertex(std::shared_ptr<Vertex> v) {
    if (config_.incremental_mode_) {
        if (config_.max_vertex_size_ > 0 && vertices_.size() == config_.max_vertex_size_) {
            // 更改v的id,并移除最旧的顶点
            // 由于是replacement,也不需要将它放入new vertices中
            int id_to_remove = vertices_id_deque_.front();
            auto v_remove = vertices_[id_to_remove];
            assert(v_remove->Dimension() == v->Dimension());  // 移除的顶点应该和加入的顶点有同样的维度

            v->SetHessianIndex(v_remove->HessianIndex());
            v->SetColInHessian(v_remove->ColInHessian());
            v->MapHessianMemory(v_remove->GetHessianMap());

            // set iv map
            for (auto &vv : iv_map_) {
                if (vv == v_remove.get()) {
                    vv = v.get();
                    break;
                }
            }

            for (auto &vv : active_vertices_) {
                if (vv == v_remove.get()) {
                    vv = v.get();
                    break;
                }
            }

            if (verbose_) {
                LOG(INFO) << "replacing new vertex " << v->GetId() << " with old " << v_remove->GetId();
            }

            /// 先remove active edges
            auto edges_to_remove = v_remove->GetEdges();
            std::set<Edge *> er_set;
            for (auto &e : edges_to_remove) {
                er_set.emplace(e);
            }

            std::vector<Edge *> new_active_edges;
            new_active_edges.reserve(active_edges_.size());

            for (auto &active_edge : active_edges_) {
                if (er_set.find(active_edge) == er_set.end()) {
                    new_active_edges.emplace_back(active_edge);
                } else {
                    continue;
                }
            }

            active_edges_.swap(new_active_edges);

            RemoveVertex(v_remove);
            v->SetId(id_to_remove);
            vertices_id_deque_.pop_front();

        } else {
            new_vertices_.emplace_back(v.get());
        }

        vertices_id_deque_.push_back(v->GetId());
    }

    bool ret = Graph::AddVertex(v);
    return ret;
}

bool Optimizer::AddEdge(std::shared_ptr<Edge> e) {
    bool ret = Graph::AddEdge(e);
    if (!ret) {
        return false;
    }

    if (config_.incremental_mode_) {
        new_edges_.emplace_back(e);
    }
    return true;
}

bool Optimizer::RemoveEdge(std::shared_ptr<Edge> e) {
    auto it =
        std::find_if(active_edges_.begin(), active_edges_.end(), [&](const auto &item) { return e.get() == item; });
    if (it == active_edges_.end()) {
        return false;
    }
    *it = active_edges_.back();
    active_edges_.pop_back();
    return Graph::RemoveEdge(e);
}
}  // namespace lightning::miao