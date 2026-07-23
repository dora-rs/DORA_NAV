//
// Created by xiang on 24-4-23.
//

#ifndef MIAO_OPTIMIZER_H
#define MIAO_OPTIMIZER_H

#include "core/common/config.h"
#include "core/graph/graph.h"
#include "core/sparse/sparse_block_matrix.h"

#include <deque>

namespace lightning::miao {

class OptimizationAlgorithm;
class Solver;

/// 基础的优化器，也是graph的一种
// TODO: block solver是编译期的，不是运行期的
class Optimizer : public Graph {
   public:
    /// TODO: 根据优化器选择要用的solver
    Optimizer();
    ~Optimizer() override;

    void SetConfig(OptimizerConfig config) { config_ = config; }
    OptimizerConfig GetConfig() const { return config_; }

    /**
     * Initializes the structures for optimizing the whole graph.
     * Before calling it be sure to invoke marginalized() and fixed() to the
     * vertices you want to include in the Schur complement or to set as fixed
     * during the optimization.
     * @param level: is the level (in multilevel optimization)
     * @returns false if somethings goes wrong
     */
    /**
     * 初始化优化器
     * @param level 优化的edge级别
     * @return
     */
    bool InitializeOptimization(int level = 0);

    /// 添加顶点
    bool AddVertex(std::shared_ptr<Vertex> v) override;

    /// 添加边
    bool AddEdge(std::shared_ptr<Edge> e) override;

    /// 移除边
    bool RemoveEdge(std::shared_ptr<Edge> e) override;

    /**
     * starts one optimization run given the current configuration of the graph,
     * and the current settings stored in the class instance.
     * It can be called only after InitializeOptimization
     */
    int Optimize(int iterations);

    /**returns the cached chi2 of the active portion of the graph*/
    double ActiveChi2() const;
    /**
     * returns the cached chi2 of the active portion of the graph.
     * In contrast to ActiveChi2() this functions considers the weighting
     * of the error according to the robustification of the error functions.
     */
    double ActiveRobustChi2() const;

    //! Verbose information during optimization
    bool Verbose() const { return verbose_; }
    void SetVerbose(bool verbose) { verbose_ = verbose; }

    /**
     * sets a variable checked at every iteration to force a user stop. The
     * iteration exits when the variable is true;
     */
    void SetForceStopFlag(bool* flag) { force_stop_flag_ = flag; }
    bool* ForceStopFlag() const { return force_stop_flag_; };

    //! if external stop flag is given, return its state. False otherwise
    bool Terminate() { return force_stop_flag_ != nullptr && (*force_stop_flag_); }

    //! the index mapping of the vertices
    const VertexContainer& IndexMapping() const { return iv_map_; }
    //! the vertices active in the current optimization
    const VertexContainer& ActiveVertices() const { return active_vertices_; }
    //! the edges active in the current optimization
    const std::vector<Edge*>& ActiveEdges() const { return active_edges_; }

    //! new verticies
    const VertexContainer& NewVertices() const { return new_vertices_; }
    //! new edges
    const EdgeContainer& NewEdges() const { return new_edges_; }

    /// 清空新增的部分
    void ClearNewElements() {
        new_vertices_.clear();
        new_edges_.clear();
    }

    /**
     * Remove a vertex. If the vertex is contained in the currently active set
     * of vertices, then the internal temporary structures are cleaned, e.g., the
     * index mapping is erased. In case you need the index mapping for
     * manipulating the graph, you have to store it in your own copy.
     */
    bool RemoveVertex(std::shared_ptr<Vertex> v, bool detach = false) override;

    /// TODO Algorithm 的 set optimizer 没用shared from this
    void SetAlgorithm(std::shared_ptr<OptimizationAlgorithm> algorithm);

    //! Push all the active vertices onto a stack
    void Push();
    //! Pop (restore) the estimate of the active vertices from the stack
    void Pop();

    //! same as above, but for the active vertices
    void DiscardTop();

    /**
     * clears the graph, and polishes some intermediate structures
     * Note that this only removes nodes / edges. Parameters can be removed
     * with clearParameters().
     */
    void Clear() override;

    /**
     * computes the error vectors of all edges in the activeSet, and caches them
     */
    void ComputeActiveErrors();

    /**
     * Update the estimate of the active vertices
     * @param update: the double vector containing the stacked
     * elements of the increments on the vertices.
     */
    void Update(const double* update);

   protected:
    /**
     * builds the mapping of the active vertices to the (Block) row / column in
     * the Hessian
     */
    bool BuildIndexMapping(const std::vector<Vertex*>& vlist);

    /// 从原始的vertices, edges开始初始化
    void InitFromRaw(int level);
    void InitFromLast(int level);

    void ClearIndexMapping();

    OptimizerConfig config_;

    bool* force_stop_flag_ = nullptr;
    bool verbose_ = false;

    VertexContainer iv_map_;           /// 按ID顺序排列好的vertex，被marg部分在前
    VertexContainer active_vertices_;  ///< sorted according to id
    std::vector<Edge*> active_edges_;  ///< sorted according to id

    /// incremental 模式
    VertexContainer new_vertices_;       /// inc模式下，新增的vertex
    EdgeContainer new_edges_;            /// inc模式下，新增的edges
    std::deque<int> vertices_id_deque_;  /// inc模式下，需要记录添加的顶点顺序

    void SortVectorContainers();

    std::shared_ptr<OptimizationAlgorithm> algorithm_ = nullptr;
};

}  // namespace lightning::miao

#endif  // MIAO_OPTIMIZER_H
