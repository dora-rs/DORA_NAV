//
// Created by xiang on 24-3-19.
//

#include "graph.h"
#include "core/common/string_tools.h"
#include "core/graph/edge.h"
#include "vertex.h"

#include <glog/logging.h>
#include <numeric>
#include <unordered_set>

namespace lightning::miao {

std::shared_ptr<Vertex> Graph::GetVertex(int id) {
    auto it = vertices_.find(id);
    if (it == vertices_.end()) {
        return nullptr;
    }
    return it->second;
}

bool Graph::AddVertex(std::shared_ptr<Vertex> v) {
    auto result = vertices_.insert(std::make_pair(v->GetId(), v));
    return result.second;
}

bool Graph::AddEdge(std::shared_ptr<Edge> e) {
    for (const auto& v : e->GetVertices()) {
        if (v == nullptr) {
            return false;
        }
    }

    /// 检查edge中是否有重复顶点
    std::unordered_set<Vertex*> vertexPointer;
    auto ves = e->GetVertices();
    for (const auto& v : ves) {
        vertexPointer.insert(v);
    }

    if (vertexPointer.size() != ves.size()) {
        return false;
    }

    auto result = edges_.insert(e);
    if (!result.second) {
        return false;
    }

    for (const auto& v : vertexPointer) {  // connect the vertices to this edge
        v->AddEdge(e);
    }

    e->SetInternalId(next_edge_id_++);
    if (e->NumUndefinedVertices()) {
        LOG(ERROR) << "this edge has undefined vertex.";
        return true;
    }

    return true;
}

bool Graph::SetEdgeVertex(Edge* e, int pos, std::shared_ptr<Vertex> v) {
    auto vOld = e->GetVertex(pos);
    if (vOld) {
        vOld->RemoveEdge(e);
    }

    e->SetVertex(pos, v);
    if (v) {
        v->AddEdge(e);
    }

    return true;
}

bool Graph::DetachVertex(std::shared_ptr<Vertex> v) {
    auto it = vertices_.find(v->GetId());
    if (it == vertices_.end()) {
        return false;
    }

    assert(it->second == v);
    for (auto& iter : v->GetEdges()) {
        auto e = iter;
        for (size_t i = 0; i < e->GetVertices().size(); i++) {
            if (v.get() == e->GetVertex(i)) {
                SetEdgeVertex(e, i, nullptr);
            }
        }
    }
    return true;
}

Graph::~Graph() { Clear(); }

bool Graph::RemoveEdge(Edge* e) {
    auto it = std::find_if(edges_.begin(), edges_.end(), [&](const auto& item) { return e == item.get(); });
    if (it == edges_.end()) {
        return false;
    }

    /// note: 直接在edges里移除it会导致shared_ptr计数清零，e变为dangling pointer
    std::shared_ptr<Edge> edge_to_remove = *it;
    edges_.erase(it);
    for (const auto& vit : e->GetVertices()) {
        if (vit == nullptr) {
            continue;
        }

        vit->RemoveEdge((*it).get());
    }
    return true;
}

bool Graph::RemoveEdge(std::shared_ptr<Edge> e) { return RemoveEdge(e.get()); }

void Graph::Clear() {
    vertices_.clear();
    edges_.clear();
    next_edge_id_ = 0;
}

bool Graph::RemoveVertex(std::shared_ptr<Vertex> v, bool detach) {
    if (detach) {
        bool result = DetachVertex(v);
        if (!result) {
            assert(0 && "inconsistency in detaching vertex, ");
        }
    }

    auto it = vertices_.find(v->GetId());
    if (it == vertices_.end()) {
        return false;
    }

    assert(it->second == v);

    // remove all edges which are entering or leaving v;
    int cnt_removed_edges = 0;
    for (auto& e : v->GetEdges()) {
        RemoveEdge(e);
        cnt_removed_edges++;
    }

    vertices_.erase(it);
    return true;
}

double Graph::Chi2() const {
    return std::accumulate(edges_.begin(), edges_.end(), 0,
                           [](float sum, const std::shared_ptr<Edge>& e) { return e->Chi2() + sum; });
}

}  // namespace lightning::miao