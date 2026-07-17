//
// Created by xiang on 24-3-19.
//

#ifndef MIAO_BASE_VERTEX_H
#define MIAO_BASE_VERTEX_H

#include "common/eigen_types.h"
#include "common/std_types.h"
#include "vertex.h"

#include <mutex>
#include <stack>

namespace lightning::miao {

/**
 * 带模板参数的基类顶点
 * @tparam D 顶点维度
 * @tparam T 顶点的估计值变量类型,可以为自定义的结构体
 *
 * 由于模板参数指定了维度，此时我们可以实际定义vertex的估计值，hessian块大小等变量了，而在基类的vertex只能定义接口
 *
 * 由于顶点维度在编译期已知，Hessian Blocks的矩阵块维度也是已知的
 * 在block solver中，优化器先建立整个优化问题的结构，再调用每个顶点和边的hessian构造函数
 */
template <int D, typename T>
class BaseVertex : public Vertex {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType = T;
    using BackupStackType = std::stack<EstimateType, std::vector<EstimateType>>;

    /// NOTE base vertex 既可以通过Dimension变量获取维度（编译期），也可以通过Dimension()函数获取（运行期）
    static const int Dimension = D;  ///< dimension of the estimate (minimal) in the manifold space

    /// hessian 矩阵类型，本质上是一个映射，实际数据在block solver里, 为稀疏矩阵块
    using HessianBlockType = Eigen::Matrix<double, D, D>;

   public:
    BaseVertex() : Vertex() {
        dimension_ = D;
        hessian_.setZero();
    }

    BaseVertex(const BaseVertex<D, T> &other) : Vertex(other) {
        dimension_ = other.dimension_;
        hessian_ = other.hessian_;
        b_ = other.b_;
        hessian_in_solver_ = other.hessian_in_solver_;
        estimate_ = other.estimate_;
        backup_ = other.backup_;
    }

    const double &Hessian(int i, int j) const override { return hessian_(i, j); }

    double &Hessian(int i, int j) override { return hessian_(i, j); }

    /**
     * 在hessian位置上创建一个矩阵块
     * @param d     实际存储的数据地址
     */
    inline void MapHessianMemory(double *d) override {
        hessian_in_solver_ = d;
        // CopyHessianFromSolver();
        // new (&hessian_) HessianBlockType(d, Dimension, Dimension);
    }

    inline double *GetHessianMap() override { return hessian_in_solver_; }

    void ClearHessian() override { hessian_.setZero(); }

    /// 拷贝hessian至solver
    void CopyHessianToSolver() override {
        if (hessian_in_solver_) {
            memcpy(hessian_in_solver_, hessian_.data(), Dimension * Dimension * sizeof(double));
        }
    }

    /// 将b中的元素拷贝到成员变量
    int CopyB(double *b) const override {
        memcpy(b, b_.data(), Dimension * sizeof(double));
        return Dimension;
    }

    /// 获取整个的B vector
    Eigen::Matrix<double, D, 1> &GetB() { return b_; }

    const Eigen::Matrix<double, D, 1> &GetB() const { return b_; }

    //! return the hessian Block associated with the vertex
    HessianBlockType &A() { return hessian_; }

    const HessianBlockType &A() const { return hessian_; }

    /// 更新内部的H, b值
    virtual void UpdateHessianAndBias(const HessianBlockType &H, const Eigen::Matrix<double, D, 1> &b);

    /// 估计值放入缓存
    void Push() override { backup_.push(estimate_); }

    /// 从缓存中提取估计值
    void Pop() override {
        assert(!backup_.empty());
        estimate_ = backup_.top();
        backup_.pop();
    }

    /// 放弃缓存中的估计值
    void DiscardTop() override {
        assert(!backup_.empty());
        backup_.pop();
    }

    //! return the current estimate of the vertex
    const EstimateType &Estimate() const { return estimate_; }

    //! set the estimate for the vertex also calls updateCache()
    void SetEstimate(const EstimateType &et) { estimate_ = et; }

    /// 清空b
    inline void ClearQuadraticForm() override {
        hessian_.setZero();
        b_.setZero();
    }

   protected:
    /// NOTE: 现在hessian和b都存储于本地，通过Copy操作再进入solver,而不是直接map to solver
    std::mutex hessian_mutex_;
    HessianBlockType hessian_;
    Eigen::Matrix<double, D, 1> b_ = Eigen::Matrix<double, D, 1>::Zero();
    double *hessian_in_solver_ = nullptr;

    EstimateType estimate_;   // 估计值
    BackupStackType backup_;  // 备份值
};

template <int D, typename T>
void BaseVertex<D, T>::UpdateHessianAndBias(const BaseVertex::HessianBlockType &H,
                                            const Eigen::Matrix<double, D, 1> &b) {
    UL lock(hessian_mutex_);
    hessian_.noalias() += H;
    b_.noalias() += b;
}

}  // namespace lightning::miao

#endif  // MIAO_BASE_VERTEX_H
