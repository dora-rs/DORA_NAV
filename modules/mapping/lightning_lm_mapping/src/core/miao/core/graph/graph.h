//
// Created by xiang on 24-3-19.
//

#ifndef MIAO_GRAPH_H
#define MIAO_GRAPH_H

#include <algorithm>
#include <bitset>
#include <cassert>
#include <cstddef>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

#include "core/common/macros.h"

namespace lightning::miao {

class Vertex;
class Edge;

/**
 * 图优化中的图
 *
 * vertex和edge的指针实际由graph持有，而vertex内部的edge,
 * 或edge内部的vertex,使用原生指针，不需要delete，避免weak_ptr的性能问题
 */
using VertexIDMap = std::unordered_map<int, std::shared_ptr<Vertex>>;
class Graph {
   public:
    Graph() = default;
    using EdgeSet = std::set<std::shared_ptr<Edge>>;
    using VertexSet = std::set<std::shared_ptr<Vertex>>;
    using VertexContainer = std::vector<Vertex*>;
    using EdgeContainer = std::vector<std::shared_ptr<Edge>>;

    /// graph 不允许拷贝
    DISALLOW_COPY(Graph)

    /// 析构时清空
    virtual ~Graph();

    /// 添加顶点
    virtual bool AddVertex(std::shared_ptr<Vertex> v);

    /// 往图中添加一个边
    /// 如果边已经关联了vertex,也会在vertex中设置这个边
    virtual bool AddEdge(std::shared_ptr<Edge> e);

    /// 获取某个id的顶点，如果不存在返回nullptr
    std::shared_ptr<Vertex> GetVertex(int id);

    /// 移除一个顶点
    virtual bool RemoveVertex(std::shared_ptr<Vertex> v, bool detach = false);

    /// 移除一个边
    virtual bool RemoveEdge(std::shared_ptr<Edge> e);
    virtual bool RemoveEdge(Edge* e);

    /// 清空整个图
    virtual void Clear();

    /// 获取所有顶点
    const VertexIDMap& GetVertices() const { return vertices_; }
    VertexIDMap& GetVertices() { return vertices_; }

    /// 获取所有边
    const std::set<std::shared_ptr<Edge>>& GetEdges() const { return edges_; }
    std::set<std::shared_ptr<Edge>>& GetEdges() { return edges_; }

    /// 关联某个edge和某个vertex
    virtual bool SetEdgeVertex(Edge* e, int pos, std::shared_ptr<Vertex> v);

    /// 分离某个vertex
    bool DetachVertex(std::shared_ptr<Vertex> v);

    /// 执行优化
    virtual int Optimize(int interations) { return 0; }

    /// 整个优化问题的chi2
    double Chi2() const;

   protected:
    VertexIDMap vertices_;                   // 所有顶点
    std::set<std::shared_ptr<Edge>> edges_;  // 所有边
    long long next_edge_id_ = 0;             // edge内部id
};

}  // namespace lightning::miao

#endif  // MIAO_GRAPH_H
