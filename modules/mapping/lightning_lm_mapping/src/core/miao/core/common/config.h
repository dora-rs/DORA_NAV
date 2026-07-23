//
// Created by xiang on 24-6-4.
//

#ifndef MIAO_CONFIG_H
#define MIAO_CONFIG_H

namespace lightning::miao {

/// 优化器使用的优化方法
enum class AlgorithmType {
    GAUSS_NEWTON = 0,
    LEVENBERG_MARQUARDT,
    DOGLEG,
};

/// 线性方程的求解方法
enum class LinearSolverType {
    LINEAR_SOLVER_DENSE,         // eigen 稠密求解器
    LINEAR_SOLVER_SPARSE_EIGEN,  // eigen 稀疏求解器
    LINEAR_SOLVER_PCG,           // PCG 求解器，快但是糙一些

    /// 后面的还未移植，目前不是特别想做
    // LINEAR_SOLVER_CSPARSE,  // Csparse 求解器
    // LINEAR_SOLVER_CHOLMOD,  // Cholmod 求解器
};

/// 配置优化器
struct OptimizerConfig {
    OptimizerConfig() = default;
    explicit OptimizerConfig(AlgorithmType algo_type,
                             LinearSolverType linear_type = LinearSolverType::LINEAR_SOLVER_DENSE, bool is_dense = true)
        : algo_type_(algo_type), is_dense_(is_dense), linear_solver_type_(linear_type) {}

    // 优化方法,从Gauss-newton, Levenberg-marquardt, dogleg中选
    AlgorithmType algo_type_ = AlgorithmType::LEVENBERG_MARQUARDT;

    // 线性方程求解方法,从eigen, dense, pcg中选
    LinearSolverType linear_solver_type_ = LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN;

    // 稀疏还是稠密
    // 稠密问题不允许有marg
    // 稀疏问题可以有marg, 也可以没有marg
    bool is_dense_ = true;

    /// 增量模式下配置
    /// ==============================================================================
    /// 增量模式可用于实时定位、实时pose graph等应用场景，但增量模式不适用于带Marg的顶点（因为Hessian index需要重排），
    /// 所以增量适用于带各种约束下的pose graph
    /// NOTE: 目前增加模式不支持SetFixed的节点，因为setFixed会跳过fixed nodes，导致Hessian index重排

    // 打开增量模式
    bool incremental_mode_ = false;

    // 最大顶点数量限制，为-1时，不会限制最大vertex size,可用于在线slam模式下pose graph之类的应用
    // 不为-1时，optimizer会用新的顶点替换掉最旧的顶点，保持block solver不变
    int max_vertex_size_ = -1;

    // 并发策略
    // 打开并发时，求解器会使用最快的并发进行求解
    bool parallel_ = true;

    double eps_chi2_ = 1e-4;  // 允许的退出chi2误差
};

}  // namespace lightning::miao

#endif  // MIAO_CONFIG_H
