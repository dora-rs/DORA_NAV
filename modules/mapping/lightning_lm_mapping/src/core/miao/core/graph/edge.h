//
// Created by xiang on 24-3-19.
//

#ifndef MIAO_EDGE_H
#define MIAO_EDGE_H

#include <algorithm>
#include <cassert>
#include <memory>
#include <set>
#include <vector>

#include "core/common/macros.h"

namespace lightning::miao {

class Vertex;

class Graph;

class RobustKernel;

/**
 * edge 是图中的一条边，也是图的基本元素
 * edge 表示vertex之间的约束，与measurement一起构成误差的计算
 * 继承关系：edge - base_edge - base_fixed_sized_edge - 各种实用edge
 */
class Edge {
   public:
    /// 基类Edge
    using VertexContainer = std::vector<Vertex *>;

    explicit Edge(int id = invalid_id) : id_(id) {}

    virtual ~Edge() = default;

    /// interface   ===========================================================================================
    /// 是否所有顶点都是固定的
    virtual bool AllVerticesFixed() const = 0;

    /// 计算误差，用户来定义
    virtual void ComputeError() = 0;

    /// 获取核函数
    std::shared_ptr<RobustKernel> GetRobustKernel() const { return robust_kernel_; }

    /// 设置核函数
    void SetRobustKernel(std::shared_ptr<RobustKernel> rb) { robust_kernel_ = rb; }

    /// 获取卡方
    virtual double Chi2() const = 0;

    /// 线性化
    virtual void ConstructQuadraticForm() = 0;

    /**
     * 将hessian转换到外部某个矩阵
     * maps the internal matrix to some external memory location,
     * you need to provide the memory before calling constructQuadraticForm
     * @param d the memory location to which we map
     * @param i index of the vertex i
     * @param j index of the vertex j (j > i, upper triangular fashion)
     * @param rowMajor if true, will write in rowMajor order to the Block. Since
     * EIGEN is columnMajor by default, this results in writing the transposed
     */
    virtual void MapHessianMemory(double *d, int i, int j, bool rowMajor) = 0;

    /// 将edge本地的jacobians拷贝至solver
    virtual void CopyHessianToSolver() = 0;

    /**
     * 线性化，并把jacobian放到某个矢量空间
     */
    virtual void LinearizeOplus() = 0;

    /// accessors   ===========================================================================================
    /// Resize the vertices container
    virtual void Resize(size_t size) { vertices_.resize(size); }

    /// accessors
    const VertexContainer &GetVertices() const { return vertices_; }
    VertexContainer &GetVertices() { return vertices_; }

    /// 获取中间某个vertex
    Vertex *GetVertex(size_t i) const {
        assert(i < vertices_.size());
        return vertices_[i];
    }

    /// 设置 vertex
    void SetVertex(size_t i, std::shared_ptr<Vertex> v) {
        assert(i < vertices_.size());
        vertices_[i] = v.get();
    }

    int GetId() const { return id_; }

    void SetId(int id) { id_ = id; }

    /// 获取非法的顶点数量
    int NumUndefinedVertices() const {
        return std::count_if(vertices_.begin(), vertices_.end(), [](const Vertex *ptr) { return ptr == nullptr; });
    }

    int Level() const { return level_; }

    void SetLevel(int level) { level_ = level; }

    int Dimension() const { return dimension_; }

    long long GetInternalId() const { return internal_id_; }

    void SetInternalId(long long internal_id) { internal_id_ = internal_id; }

   protected:
    VertexContainer vertices_;                               // 顶点
    int dimension_ = -1;                                     // edge的维度
    int level_ = 0;                                          // 优化层级，默认0级的边才参与优化
    std::shared_ptr<RobustKernel> robust_kernel_ = nullptr;  // 核函数
    int id_ = 0;                                             // id
    long long internal_id_ = -1;                             // 内部id
};

}  // namespace lightning::miao
#endif  // MIAO_EDGE_H
