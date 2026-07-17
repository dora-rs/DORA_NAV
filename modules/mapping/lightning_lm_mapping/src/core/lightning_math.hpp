//
// Created by xiang on 25-3-12.
//

#ifndef LIGHTNING_LIGHTNING_MATH_HPP
#define LIGHTNING_LIGHTNING_MATH_HPP

#pragma once

#include <glog/logging.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/array.hpp>
#include <boost/math/tools/precision.hpp>
#include <cmath>
#include <numeric>

// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <rclcpp/time.hpp>

#include "common/eigen_types.h"
#include "common/options.h"
#include "common/point_def.h"
#include "common/pose_rpy.h"

namespace lightning::math {

template <typename T>
inline Eigen::Matrix<T, 3, 3> SKEW_SYM_MATRIX(const Eigen::Matrix<T, 3, 1>& v) {
    Eigen::Matrix<T, 3, 3> m;
    m << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
    return m;
}

template <typename T>
inline Eigen::Matrix<T, 3, 3> SKEW_SYM_MATRIX(const T& v1, const T& v2, const T& v3) {
    Eigen::Matrix<T, 3, 3> m;
    m << 0.0, -v3, v2, v3, 0.0, -v1, -v2, v1, 0.0;
    return m;
}

template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>&& ang) {
    T ang_norm = ang.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (ang_norm > 0.0000001) {
        Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
        Eigen::Matrix<T, 3, 3> K;
        K = SKEW_SYM_MATRIX(r_axis);

        return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
    } else {
        return Eye3;
    }
}

template <typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>& ang_vel, const Ts& dt) {
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    if (ang_vel_norm > 0.0000001) {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix<T, 3, 3> K;

        K = SKEW_SYM_MATRIX(r_axis);

        T r_ang = ang_vel_norm * dt;

        /// Roderigous Tranformation
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    } else {
        return Eye3;
    }
}

template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const T& v1, const T& v2, const T& v3) {
    T&& norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (norm > 0.00001) {
        Eigen::Matrix<T, 3, 3> K;
        K = SKEW_SYM_MATRIX(v1 / norm, v2 / norm, v3 / norm);

        /// Roderigous Tranformation
        return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
    } else {
        return Eye3;
    }
}

/* Logrithm of a Rotation Matrix */
template <typename T>
Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3>& R) {
    T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
    Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
    return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

template <typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3>& rot) {
    T sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
    bool singular = sy < 1e-6;
    T x, y, z;
    if (!singular) {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);
        z = atan2(rot(1, 0), rot(0, 0));
    } else {
        x = atan2(-rot(1, 2), rot(1, 1));
        y = atan2(-rot(2, 0), sy);
        z = 0;
    }
    Eigen::Matrix<T, 3, 1> ang(x, y, z);
    return ang;
}

// template <typename T>
// Eigen::Matrix<T, 3, 3> RpyToRotM(const T r, const T p, const T y) {
//     auto q = tf::createQuaternionFromRPY(r, p, y);
//     Eigen::Quaterniond res;
//     tf::quaternionTFToEigen(q, res);
//     return res.toRotationMatrix().cast<T>();
// }

/// xt: 经验证，此转换与ros中tf::createQuaternionFromRPY结果一致
template <typename T>
Eigen::Matrix<T, 3, 3> RpyToRotM2(const T r, const T p, const T y) {
    using AA = Eigen::AngleAxis<T>;
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    return Eigen::Matrix<T, 3, 3>(AA(y, Vec3::UnitZ()) * AA(p, Vec3::UnitY()) * AA(r, Vec3::UnitX()));
}

template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const std::vector<double>& v) {
    return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const boost::array<S, 3>& v) {
    return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> MatFromArray(const std::vector<double>& v) {
    Eigen::Matrix<S, 3, 3> m;
    m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return m;
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> MatFromArray(const boost::array<S, 9>& v) {
    Eigen::Matrix<S, 3, 3> m;
    m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return m;
}

template <typename T>
T rad2deg(const T& radians) {
    return radians * 180.0 / M_PI;
}

template <typename T>
T deg2rad(const T& degrees) {
    return degrees * M_PI / 180.0;
}

/**
 * 将某个数限定在范围内
 * @tparam T
 * @param num
 * @param min_limit
 * @param max_limit
 * @return
 */
template <typename T, typename T2>
void limit_in_range(T&& num, T2&& min_limit, T2&& max_limit) {
    if (num < min_limit) {
        num = min_limit;
    }
    if (num >= max_limit) {
        num = max_limit;
    }
}

/**
 * 计算均值与对角线形式的协方差
 * @tparam C 数据类型
 * @tparam D 均值的数据类型
 * @tparam Getter 获取被计算的字段
 * @param data 数据的某种容器
 * @param mean 均值
 * @param cov_diag 对角形式的协方差阵j
 * @param getter 从数据中获取被计算字段的函数
 */
template <typename C, typename D, typename Getter>
inline void ComputeMeanAndCovDiag(const C& data, D& mean, D& cov_diag, Getter&& getter) {
    size_t len = data.size();
    if (len == 1) {
        mean = getter(data[0]);
        cov_diag = D::Zero().eval();
    } else {
        // clang-format off
        mean = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                               [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
        cov_diag = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                                   [&mean, &getter](const D& sum, const auto& data) -> D {
                                     return sum + (getter(data) - mean).cwiseAbs2();
                                   }) / (len);
        // clang-format on
    }
}

/**
 * 高斯分布的增量更新
 * @see https://www.cnblogs.com/yoyaprogrammer/p/delta_variance.html
 * @param hist_n     历史数据个数
 * @param hist_mean 历史均值
 * @param hist_var2 历史方差
 * @param curr_n    当前数据个数
 * @param curr_mean 当前真值
 * @param curr_var2 当前方差
 * @param new_mean 合并后均值
 * @param new_var2 合并后方差
 */
inline void HistoryMeanAndVar(size_t hist_n, float hist_mean, float hist_var2, size_t curr_n, float curr_mean,
                              float curr_var2, float& new_mean, float& new_var2) {
    new_mean = (hist_n * hist_mean + curr_n * curr_mean) / (hist_n + curr_n);
    new_var2 = (hist_n * (hist_var2 + (new_mean - hist_mean) * (new_mean - hist_mean)) +
                curr_n * (curr_var2 + (new_mean - curr_mean) * (new_mean - curr_mean))) /
               (hist_n + curr_n);
}

/**
 * Calculate cosine and sinc of sqrt(x2).
 * @param x2 the squared angle must be non-negative
 * @return a pair containing cos and sinc of sqrt(x2)
 */
template <class scalar>
inline std::pair<scalar, scalar> cos_sinc_sqrt(const scalar& x2) {
    using std::cos;
    using std::sin;
    using std::sqrt;
    static scalar const taylor_0_bound = boost::math::tools::epsilon<scalar>();
    static scalar const taylor_2_bound = sqrt(taylor_0_bound);
    static scalar const taylor_n_bound = sqrt(taylor_2_bound);

    assert(x2 >= 0 && "argument must be non-negative");

    // FIXME check if bigger bounds are possible
    if (x2 >= taylor_n_bound) {
        // slow fall-back solution
        scalar x = sqrt(x2);
        return std::make_pair(cos(x), sin(x) / x);  // x is greater than 0.
    }

    // FIXME Replace by Horner-Scheme (4 instead of 5 FLOP/term, numerically more stable, theoretically cos and sinc can
    // be calculated in parallel using SSE2 mulpd/addpd)
    // TODO Find optimal coefficients using Remez algorithm
    static scalar const inv[] = {1 / 3., 1 / 4., 1 / 5., 1 / 6., 1 / 7., 1 / 8., 1 / 9.};
    scalar cosi = 1., sinc = 1;
    scalar term = -1 / 2. * x2;
    for (int i = 0; i < 3; ++i) {
        cosi += term;
        term *= inv[2 * i];
        sinc += term;
        term *= -inv[2 * i + 1] * x2;
    }

    return std::make_pair(cosi, sinc);
}

inline SO3 exp(const Vec3d& vec, const double& scale = 1) {
    double norm2 = vec.squaredNorm();
    std::pair<double, double> cos_sinc = cos_sinc_sqrt(scale * scale * norm2);
    double mult = cos_sinc.second * scale;
    Vec3d result = mult * vec;
    return SO3(Quatd(cos_sinc.first, result[0], result[1], result[2]));
}

inline Eigen::Matrix<double, 2, 3> PseudoInverse(const Eigen::Matrix<double, 3, 2>& X) {
    Eigen::JacobiSVD<Eigen::Matrix<double, 3, 2>> svd(X, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Vec2d sv = svd.singularValues();
    Eigen::Matrix<double, 3, 2> U = svd.matrixU().block<3, 2>(0, 0);
    Eigen::Matrix<double, 2, 2> V = svd.matrixV();
    Eigen::Matrix<double, 2, 3> U_adjoint = U.adjoint();
    double tolerance = std::numeric_limits<double>::epsilon() * 3 * std::abs(sv(0, 0));
    sv(0, 0) = std::abs(sv(0, 0)) > tolerance ? 1.0 / sv(0, 0) : 0;
    sv(1, 0) = std::abs(sv(1, 0)) > tolerance ? 1.0 / sv(1, 0) : 0;

    return V * sv.asDiagonal() * U_adjoint;
}

/**
 * SO3 Jl()/JacobianL()
 * @param v
 * @return
 */
inline Eigen::Matrix<double, 3, 3> A_matrix(const Vec3d& v) {
    Eigen::Matrix<double, 3, 3> res;
    double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    double norm = std::sqrt(squaredNorm);
    if (norm < 1e-5) {
        res = Eigen::Matrix<double, 3, 3>::Identity();
    } else {
        res = Eigen::Matrix<double, 3, 3>::Identity() + (1 - std::cos(norm)) / squaredNorm * SO3::hat(v) +
              (1 - std::sin(norm) / norm) / squaredNorm * SO3::hat(v) * SO3::hat(v);
    }
    return res;
}

/// SO3 Jlinv()
inline Eigen::Matrix<double, 3, 3> A_inv(const Vec3d& v) {
    Eigen::Matrix<double, 3, 3> res;
    if (v.norm() > 1e-5) {
        res = Eigen::Matrix<double, 3, 3>::Identity() - 0.5 * SKEW_SYM_MATRIX(v) +
              (1 - v.norm() * std::cos(v.norm() / 2) / 2 / std::sin(v.norm() / 2)) * SKEW_SYM_MATRIX(v) *
                  SKEW_SYM_MATRIX(v) / v.squaredNorm();

    } else {
        // res = Eigen::Matrix<double, 3, 3>::Identity();
        res = Eigen::Matrix<double, 3, 3>::Identity() - 0.5 * SKEW_SYM_MATRIX(v) +
              ((1. / 12) + v.squaredNorm() * 1. / 720) * SKEW_SYM_MATRIX(v) * SKEW_SYM_MATRIX(v);
    }

    return res;
}

/// hash of vector
template <int N>
struct hash_vec {
    inline size_t operator()(const Eigen::Matrix<int, N, 1>& v) const;
};

/// vec 2 hash
/// @see Optimized Spatial Hashing for Collision Detection of Deformable Objects, Matthias Teschner et. al., VMV 2003
template <>
inline size_t hash_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v) const {
    return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943)) % 10000000;
}

/// vec 3 hash
template <>
inline size_t hash_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v) const {
    return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943) ^ ((v[2]) * 83492791)) % 10000000;
}

/**
 * estimate a plane
 * @tparam T
 * @param pca_result
 * @param point
 * @param threshold
 * @return
 */
template <typename T>
inline bool esti_plane(Eigen::Matrix<T, 4, 1>& pca_result, const PointVector& point, const T& threshold = 0.1f) {
    if (point.size() < fasterlio::MIN_NUM_MATCH_POINTS) {
        return false;
    }

    Eigen::Matrix<T, 3, 1> normvec;

    if (point.size() == fasterlio::NUM_MATCH_POINTS) {
        Eigen::Matrix<T, fasterlio::NUM_MATCH_POINTS, 3> A;
        Eigen::Matrix<T, fasterlio::NUM_MATCH_POINTS, 1> b;

        A.setZero();
        b.setOnes();
        b *= -1.0f;

        for (int j = 0; j < fasterlio::NUM_MATCH_POINTS; j++) {
            A(j, 0) = point[j].x;
            A(j, 1) = point[j].y;
            A(j, 2) = point[j].z;
        }

        normvec = A.colPivHouseholderQr().solve(b);
    } else {
        Eigen::MatrixXd A(point.size(), 3);
        Eigen::VectorXd b(point.size(), 1);

        A.setZero();
        b.setOnes();
        b *= -1.0f;

        for (int j = 0; j < point.size(); j++) {
            A(j, 0) = point[j].x;
            A(j, 1) = point[j].y;
            A(j, 2) = point[j].z;
        }

        Eigen::MatrixXd n = A.colPivHouseholderQr().solve(b);
        normvec(0, 0) = n(0, 0);
        normvec(1, 0) = n(1, 0);
        normvec(2, 0) = n(2, 0);
    }

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (const auto& p : point) {
        Eigen::Matrix<T, 4, 1> temp = p.getVector4fMap();
        temp[3] = 1.0;
        if (fabs(pca_result.dot(temp)) > threshold) {
            return false;
        }
    }
    return true;
}

/// 体素滤波
inline CloudPtr VoxelGrid(CloudPtr cloud, float voxel_size = 0.05) {
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);

    CloudPtr output(new PointCloudType);
    voxel.filter(*output);

    return output;
}

/// pcl 时间戳
inline double ToSec(uint64_t t) { return double(t) * 1e-9; }

// convert from pcl point to eigen
template <typename T, int dim, typename PointType>
inline Eigen::Matrix<T, dim, 1> ToEigen(const PointType& pt) {
    return Eigen::Matrix<T, dim, 1>(pt.x, pt.y, pt.z);
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3, pcl::PointXYZ>(const pcl::PointXYZ& pt) {
    return pt.getVector3fMap();
}

template <>
inline Eigen::Matrix<float, 2, 1> ToEigen<float, 2, PointXYZIT>(const PointXYZIT& pt) {
    return Vec2f(pt.x, pt.y);
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3, pcl::PointXYZI>(const pcl::PointXYZI& pt) {
    return pt.getVector3fMap();
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3, pcl::PointXYZINormal>(const pcl::PointXYZINormal& pt) {
    return pt.getVector3fMap();
}

/**
 * squared distance
 * @param p1
 * @param p2
 * @return
 */
inline float calc_dist(const PointType& p1, const PointType& p2) {
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
}

inline float calc_dist(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2) { return (p1 - p2).squaredNorm(); }

// squared distance of two pcl points
template <typename PointT>
inline double distance2(const PointT& pt1, const PointT& pt2) {
    Eigen::Vector3f d = pt1.getVector3fMap() - pt2.getVector3fMap();
    return d.squaredNorm();
}

template <typename S>
inline Eigen::Matrix<S, 4, 4> Mat4FromArray(const std::vector<double>& v) {
    Eigen::Matrix<S, 4, 4> m;
    for (int i = 0; i < m.size(); ++i) m(i / 4, i % 4) = v[i];
    return m;
}

/// 矢量比较
template <int N>
struct less_vec {
    inline bool operator()(const Eigen::Matrix<int, N, 1>& v1, const Eigen::Matrix<int, N, 1>& v2) const;
};

// 实现2D和3D的比较
template <>
inline bool less_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v1, const Eigen::Matrix<int, 2, 1>& v2) const {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
}

template <>
inline bool less_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v1, const Eigen::Matrix<int, 3, 1>& v2) const {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]) || (v1[0] == v2[0] && v1[1] == v2[1] && v1[2] < v2[2]);
}

/**
 * 计算一个容器内数据的均值与矩阵形式协方差
 * @tparam C    容器类型
 * @tparam int 　数据维度
 * @tparam Getter   获取数据函数, 接收一个容器内数据类型，返回一个Eigen::Matrix<double, dim,1> 矢量类型
 */
template <typename C, int dim, typename S, typename Getter>
void ComputeMeanAndCov(const C& data, Eigen::Matrix<S, dim, 1>& mean, Eigen::Matrix<S, dim, dim>& cov,
                       Getter&& getter) {
    using D = Eigen::Matrix<S, dim, 1>;
    using E = Eigen::Matrix<S, dim, dim>;
    size_t len = data.size();
    assert(len > 1);

    // clang-format off
    mean = std::accumulate(data.begin(), data.end(), Eigen::Matrix<S, dim, 1>::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov = std::accumulate(data.begin(), data.end(), E::Zero().eval(),
                          [&mean, &getter](const E& sum, const auto& data) -> E {
                            D v = getter(data) - mean;
                            return sum + v * v.transpose();
                          }) / (len - 1);
    // clang-format on
}

/**
 * 高斯分布合并
 * @tparam S    scalar type
 * @tparam D    dimension
 * @param hist_m        历史点数
 * @param curr_n        当前点数
 * @param hist_mean     历史均值
 * @param hist_var      历史方差
 * @param curr_mean     当前均值
 * @param curr_var      当前方差
 * @param new_mean      新的均值
 * @param new_var       新的方差
 */
template <typename S, int D>
void UpdateMeanAndCov(int hist_m, int curr_n, const Eigen::Matrix<S, D, 1>& hist_mean,
                      const Eigen::Matrix<S, D, D>& hist_var, const Eigen::Matrix<S, D, 1>& curr_mean,
                      const Eigen::Matrix<S, D, D>& curr_var, Eigen::Matrix<S, D, 1>& new_mean,
                      Eigen::Matrix<S, D, D>& new_var) {
    assert(hist_m > 0);
    assert(curr_n > 0);
    new_mean = (hist_m * hist_mean + curr_n * curr_mean) / (hist_m + curr_n);
    new_var = (hist_m * (hist_var + (hist_mean - new_mean) * (hist_mean - new_mean).template transpose()) +
               curr_n * (curr_var + (curr_mean - new_mean) * (curr_mean - new_mean).template transpose())) /
              (hist_m + curr_n);
}

/// 将角度保持在正负PI以内
inline void KeepAngleInPI(double& angle) {
    while (angle < -M_PI) {
        angle = angle + 2 * M_PI;
    }
    while (angle > M_PI) {
        angle = angle - 2 * M_PI;
    }
}

inline void KeepAngleIn2PI(double& angle) {
    while (angle <= 0) {
        angle = angle + 2 * M_PI;
    }
    while (angle > 2 * M_PI) {
        angle = angle - 2 * M_PI;
    }
}

// inline builtin_interfaces::msg::Time FromSec(double t) {
//     builtin_interfaces::msg::Time ret;
//     ret.sec = int32_t(t);
//     ret.nanosec = int32_t((t - ret.sec) * 1e9);
//     return ret;
// }

/// 从pose中取出yaw pitch roll
/// 使用的顺序是：roll pitch yaw ，最左是roll
/// 由于欧拉角的多解性，Eigen在分解时选择最小化第一个（也就是roll）。否则，如果选择最小化yaw，那么roll和pitch可能出现大范围旋转，不符合实际
/// @see https://stackoverflow.com/questions/33895970/about-eulerangles-conversion-from-eigen-c-library
/// 这个用在lego-loam的lidar odom里，符合它的定义
/// 2020.11 change back to tf to keep consist
inline PoseRPYD SE3ToRollPitchYaw(const SE3& pose) {
    auto rot = pose.rotationMatrix();
    // tf2::Matrix3x3 temp_tf_matrix(rot(0, 0), rot(0, 1), rot(0, 2), rot(1, 0), rot(1, 1), rot(1, 2), rot(2, 0),
    //                               rot(2, 1), rot(2, 2));
    PoseRPYD output;
    output.x = pose.translation()[0];
    output.y = pose.translation()[1];
    output.z = pose.translation()[2];

    // temp_tf_matrix.getRPY(output.roll, output.pitch, output.yaw);

    /* * 3. 提取 RPY (欧拉角)  crp 修改
     * Eigen 的 eulerAngles(2, 1, 0) 表示按照 Z-Y-X 的顺序分解。
     * 返回值的向量顺序与参数顺序一致：即 [0] 是 Yaw, [1] 是 Pitch, [2] 是 Roll。
     */
    Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0);

    output.roll = euler[2];
    output.pitch = euler[1];
    output.yaw = euler[0];
    return output;
}

/// 从xyz和yaw pitch roll转成SE3
/// 使用0,1,2顺序（从右读）,yaw最先旋转
inline SE3 XYZRPYToSE3(const PoseRPYD& pose) {
    using AA = Eigen::AngleAxisd;
    return SE3((AA(pose.yaw, Vec3d::UnitZ()) * AA(pose.pitch, Vec3d::UnitY()) * AA(pose.roll, Vec3d::UnitX())),
               Vec3d(pose.x, pose.y, pose.z));
}

/**
 * pose 插值算法
 * @tparam T    数据类型
 * @tparam C 数据容器类型
 * @tparam FT 获取时间函数
 * @tparam FP 获取pose函数
 * @param query_time 查找时间
 * @param data  数据容器
 * @param take_pose_func 从数据中取pose的谓词，接受一个数据，返回一个SE3
 * @param result 查询结果
 * @param best_match_iter 查找到的最近匹配
 *
 * NOTE 要求query_time必须在data最大时间和最小时间之间(容许0.5s内误差)
 * data的map按时间排序
 * @return
 */
template <typename T, typename C, typename FT, typename FP>
inline bool PoseInterp(double query_time, C&& data, FT&& take_time_func, FP&& take_pose_func, SE3& result,
                       T& best_match, float time_th = 0.5, bool verbose = false) {
    if (data.size() <= 1) {
        if (verbose) {
            LOG(INFO) << "data size is too small for interp: " << data.size();
        }
        return false;
    }

    double last_time = take_time_func(*data.rbegin());
    double first_time = take_time_func(*data.begin());
    if (query_time > last_time) {
        if (verbose) {
            LOG(WARNING) << "query time is later than last time.";
        }

        if (query_time < (last_time + time_th)) {
            // 如果时间差在0.5s之内，尚可接受
            // 统一改成外推形式

            if (data.size() == 1) {
                result = take_pose_func(*data.rbegin());
                best_match = *data.rbegin();
                //        std::stringstream ss;
                //        ss << ", r: " << result.translation().transpose();
                //        LOG_I(ss.str());
                return true;
            } else {
                auto second = data.rbegin();
                auto first = second;
                ++first;
                result = take_pose_func(*second);  // 姿态取最后一帧的姿态
                double time_first = take_time_func(*first);
                double time_second = take_time_func(*second);

                /// 防止末尾两个时间相等
                if (time_second <= time_first) {
                    result = take_pose_func(*data.rbegin());
                    best_match = *data.rbegin();
                    if (verbose) {
                        //            LOG_I("r: {}", result.translation().transpose());
                    }
                    return true;
                }

                /// 防止末尾两个时间太近
                while ((time_second - time_first) < 0.05) {
                    first++;
                    if (first == data.rend()) {
                        result = take_pose_func(*data.rbegin());
                        best_match = *data.rbegin();
                        if (verbose) {
                            //              LOG_I("r: {}", result.translation().transpose());
                        }
                        return true;
                    }
                    time_first = take_time_func(*first);
                }

                SE3 pose_first = take_pose_func(*first);
                SE3 pose_second = take_pose_func(*second);
                Vec6d xi = (pose_first.inverse() * pose_second).log();
                double s = (query_time - time_second) / (time_second - time_first);
                result = result * SE3::exp(s * xi);
                best_match = *data.rbegin();

                if (verbose) {
                    //          LOG_I("f: {}, s: {}, r: {}", pose_first.translation().transpose(),
                    //                pose_second.translation().transpose(), result.translation().transpose());
                }
                return true;
            }
        }

        if (verbose) {
            LOG(WARNING) << "pose interp failed: " << query_time << ", " << (last_time + time_th) << ", "
                         << (query_time < (last_time + time_th));
        }
        return false;
    } else if (query_time < first_time) {
        if (query_time < (first_time - time_th)) {
            if (verbose) {
                LOG(WARNING) << "query time too early: " << ", " << query_time << ", " << (first_time - time_th);
            }
            return false;
        } else {
            result = take_pose_func(*data.begin());
            best_match = *data.begin();
            return true;
        }
    }

    auto match_iter = data.begin();
    for (auto iter = data.begin(); iter != data.end(); ++iter) {
        auto next_iter = iter;
        next_iter++;

        if (next_iter == data.end() ||
            (take_time_func(*iter) < query_time && take_time_func(*next_iter) >= query_time)) {
            match_iter = iter;
            break;
        }
    }

    auto match_iter_n = match_iter;
    match_iter_n++;
    if (match_iter_n == data.end() || take_time_func(*match_iter) >= query_time) {
        // 就一个，那就返回他
        best_match = *match_iter;
        result = take_pose_func(*match_iter);
        return true;
    }

    double dt = take_time_func(*match_iter_n) - take_time_func(*match_iter);
    double s = (query_time - take_time_func(*match_iter)) / dt;  // s=0 时为第一帧，s=1时为next
    // 出现了 dt为0的bug
    if (fabs(dt) < 1e-6) {
        best_match = *match_iter;
        result = take_pose_func(*match_iter);
        return true;
    }

    SE3 pose_first = take_pose_func(*match_iter);
    SE3 pose_next = take_pose_func(*match_iter_n);
    result = {pose_first.unit_quaternion().slerp(s, pose_next.unit_quaternion()),
              pose_first.translation() * (1 - s) + pose_next.translation() * s};
    best_match = s < time_th ? *match_iter : *match_iter_n;
    return true;
}

}  // namespace lightning::math

#endif  // LIGHTNING_MATH_HPP