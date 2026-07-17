//
// Created by xiang on 24-3-19.
//

#ifndef MIAO_VERTEX_H
#define MIAO_VERTEX_H

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "core/common/macros.h"

namespace lightning::miao {

class Graph;

class Edge;

// 基类顶点
/**
 * 最基本的vertex,是图的基本元素
 * 顶点表示一个优化的变量，内部存储变量的估计值，同时优化器会调用顶点的更新函数对其进行更新
 * 同时，每个顶点在优化矩阵的Hessian中占据一个矩阵块，因此有一个hessian index
 * 继承关系：vertex - base_vertex - 各种实用的vertex实现
 *
 * 尽管有些优化变量要在base_vertex中才有定义，但solver操作的是基础的vertex,所以在vertex中会有接口
 */
class Vertex {
   public:
    using EdgeVector = std::vector<Edge *>;

    explicit Vertex(int id = invalid_id) : id_(id) {}

    virtual ~Vertex() = default;

    /// 优化相关

    /// 为vertex关联一个edge
    void AddEdge(std::shared_ptr<Edge> e) { edges_.emplace_back(e.get()); }
    void AddEdge(Edge *e) { edges_.emplace_back(e); }

    /// 移除edge
    void RemoveEdge(Edge *e) {
        for (auto iter = edges_.begin(); iter != edges_.end(); iter++) {
            if ((*iter) == e) {
                *iter = edges_.back();
                edges_.pop_back();
                return;
            }
        }
    }

    /**
     * 对v进行增量更新
     */
    void Oplus(const double *v) { OplusImpl(v); }

    /// 虚基类接口 ==================================================================================
    /// interface declare

    /// 创建Hessian矩阵的映射关系
    virtual void MapHessianMemory(double *d) = 0;
    virtual double *GetHessianMap() = 0;

    /// 清空Hessian
    virtual void ClearHessian() = 0;

    /// 拷贝hessian至solver
    virtual void CopyHessianToSolver() = 0;
    /**
     * copies the GetB vector in the array b_
     * 将b拷贝至给定的数组中
     * @return the number of elements copied
     */
    virtual int CopyB(double *b) const = 0;

    //! backup the position of the vertex to a stack
    // 将该顶点的估计值放入缓存栈
    virtual void Push() = 0;

    //! restore the position of the vertex by retrieving the position from the stack
    /// 从缓存栈中恢复该顶点的估计值
    virtual void Pop() = 0;

    //! Pop the last element from the stack, without restoring the current estimate
    /// 丢弃缓存栈中的估计值
    virtual void DiscardTop() = 0;

    /// 获取Hessian
    virtual const double &Hessian(int i, int j) const = 0;
    virtual double &Hessian(int i, int j) = 0;

    /**
     * Update the position of the node from the parameters in v.
     * Implement in your class!
     */
    virtual void OplusImpl(const double *v) = 0;

    /// accessors ===================================================================================
    int GetId() const { return id_; }

    virtual void SetId(int id) { id_ = id; }

    const EdgeVector &GetEdges() const { return edges_; }

    EdgeVector &GetEdges() { return edges_; }

    // accessors
    int HessianIndex() const { return hessian_index_; }

    void SetHessianIndex(int ti) { hessian_index_ = ti; }

    /// 是否固定
    bool Fixed() const { return fixed_; }

    //! true => this node should be considered fixed during the optimization
    void SetFixed(bool fixed) { fixed_ = fixed; }

    /// 是否边缘化
    //! true => this node is marginalized out during the optimization
    bool Marginalized() const { return marginalized_; }

    //! true => this node should be marginalized out during the optimization
    void SetMarginalized(bool marginalized) { marginalized_ = marginalized; }

    //! dimension of the estimated state belonging to this node
    int Dimension() const { return dimension_; }

    //! set the row of this vertex in the Hessian
    /**
     * 记录这个顶点在整个hessian中占第几列
     * @param c
     */
    void SetColInHessian(int c) { col_in_hessian_ = c; }

    //! get the row of this vertex in the Hessian
    int ColInHessian() const { return col_in_hessian_; }

    /**
     * set the GetB vector part of this vertex to zero
     */
    virtual void ClearQuadraticForm() = 0;

   protected:
    int id_ = 0;                 // 顶点的id
    std::vector<Edge *> edges_;  // 关联的edges 由于edges所属权在graph

    /// 优化参数
    bool fixed_ = false;         // 顶点是否固定
    bool marginalized_ = false;  // 顶点是否被边缘化
    int dimension_ = 0;          // 顶点的维度，在base_vertex中使用模板参数固定
    int col_in_hessian_ = -1;    // 顶点在整个Hessian矩阵中占第几列（含维度）
    int hessian_index_ = -1;
};
}  // namespace lightning::miao

using VertexPtr = std::shared_ptr<lightning::miao::Vertex>;

#endif  // MIAO_VERTEX_H
