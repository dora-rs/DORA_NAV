//
// Created by xiang on 2022/2/15.
//
#pragma once

#include "core/lightning_math.hpp"

namespace lightning {

/**
 * sphere 2
 * 本身是三维的，更新量是二维的一种东东
 * 底下那堆计算我反正看不懂，别问我是啥，问就是照抄的
 *
 * @todo S2的长度应当可以外部指定
 */
struct S2 {
    using SO3 = Sophus::SO3d;

    static constexpr int den_ = 98090;
    static constexpr int num_ = 10000;
    static constexpr double length_ = double(den_) / double(num_);
    Vec3d vec_;

   public:
    S2() { vec_ = length_ * Vec3d(1, 0, 0); }
    S2(const Vec3d &vec) : vec_(vec) {
        vec_.normalize();
        vec_ = vec_ * length_;
    }

    double operator[](int idx) const { return vec_[idx]; }

    Eigen::Matrix<double, 3, 3> S2_hat() const {
        Eigen::Matrix<double, 3, 3> skew_vec;
        skew_vec << double(0), -vec_[2], vec_[1], vec_[2], double(0), -vec_[0], -vec_[1], vec_[0], double(0);
        return skew_vec;
    }

    /**
     * S2_Mx: the partial derivative of x.boxplus(u) w.r.t. u
     */
    Eigen::Matrix<double, 3, 2> S2_Mx(const Eigen::Matrix<double, 2, 1> &delta) const {
        Eigen::Matrix<double, 3, 2> res;
        Eigen::Matrix<double, 3, 2> Bx = S2_Bx();

        if (delta.norm() < 1e-5) {
            res = -SO3::hat(vec_) * Bx;
        } else {
            Vec3d Bu = Bx * delta;
            SO3 exp_delta = math::exp(Bu, 0.5f);
            /**
             * Derivation of d(Exp(Bx dx)x)/d(dx)=d(Exp(Bu)x)/d(dx):
             *  d(Exp(Bu)x)/d(dx)=d(Exp(Bu)x)/d(Bu) Bx; then
             *  d(Exp(Bu)x)/d(Bu)=d[Exp(Bu+dBu)x]/d(dBu)=d[Exp(Jl(Bu)dBu)Exp(Bu)x]/d(dBu)=d[(Jl(Bu)dBu)^Exp(Bu)x]/d(dBu)
             *   =d[-(Exp(Bu)x)^Jl(Bu)dBu]/d(dBu)=-Exp(Bu)x^Exp(-Bu)Jl(Bu);
             *    for Exp(x+dx)=Exp(x)Exp(Jr(x)dx)=Exp(Jl(x)dx)Exp(x)=Exp(x)Exp(Exp(-x)Jl(x)dx) =>
             *    Exp(-x)Jl(x)=Jr(x)=Jl(-x) =>
             *   =-Exp(Bu)x^Jl(-Bu) => d(Exp(Bu)x)/d(dx)= -Exp(Bu) x^ Jl(Bu)^T Bx or A_matrix is just Jl()
             */
            res = -exp_delta.matrix() * SO3::hat(vec_) * math::A_matrix(Bu).transpose() * Bx;
        }
        return res;
    }

    /**
     * Bx两个列向量为正切空间的局部坐标系
     *
     * S2 = [a b c], l = length
     * Bx = [
     * -b              -c
     * l-bb/(l+a)      -bc/(l+a)
     * -bc/(l+a)        l-cc/(l+a)
     * ] / l
     * Derivation of origin MTK: (112) Rv = [v Bx] = [x -r 0; y xc -s; z xs c]
     *  where c=cos(alpha),s=sin(alpha),r=sqrt(y^2+z^2),||v||=1:
     *  For y-axis or (0,1,0) in local frame is mapped to (-r;xc;xs) in world frame,
     *  meaning x/OG(or gravity vector) projects to A of yz plane,
     *  and this projecting line will intersect the perpendicular plane of OG at O at the point B,
     *  then from OB we can found OC with ||OC||=1, which is just the y-axis, then we get its coordinate in world frame:
     *  ∠AOG+∠AGO=pi/2=∠AOG+∠AOB => ∠AOB=∠AGO, for sin(∠AGO)=r/||v||=r => ||OC||*sin(∠AOB)=r;||OC||*cos(∠AOB)=x =>
     *  (-r;xc;xs) is the y-axis coordinate in world frame, then z-axis is easy to get
     * Derivation of current MTK with S2 = [x y z], ||S2|| = 1:
     *  just a rotation of origin one around x/OG axis to meet y-axis coordinate in word frame to be (-y;a;c),
     *  then z-axis must be (+-z;b;d) for x^+y^+z^=1, where a,b,c,d is just variable to be solved
     *  for current MTK chooses clockwise rotation(meaning -z):
     *  Rv=[x -y -z;
     *      y  a  b;
     *      z  c  d]
     *  then for -xy+ya+zc=0=xy-ya-zb => b=c
     *  for yz+ab+cd=0; b=c => a=-d-yz/c; for -xz+yb+zd=0; b=c => d=x-yc/z
     *  for -xy+ya+zc=0; a=-d-yz/c=yc/z-x-yz/c => -xy+y^2c/z-xy-y^2z/c+zc=0 => (z+y^2/z)c^2 -2xy c -y^2z = 0 =>
     *  c=[2xy +- sqrt(4x^2y^2 + 4(z+y^2/z)y^2z)]/[2(z+y^2/z)]; for z^2+y^2=1-x^2 =>
     *  c=[xy +- y]z/(1-x^2), for rotation is clocewise, thus c<0 => c=(xy-y)z/(1-x^2)=-yz/(1+x)
     *  then b,a,d is easy to get and also if ||S2||=l, it is easy to prove c=-ylzl/(l+lx)/l=-bc/(l+a)/l
     */
    Eigen::Matrix<double, 3, 2> S2_Bx() const {
        Eigen::Matrix<double, 3, 2> res;
        if (vec_[0] + length_ > 1e-5) {
            res << -vec_[1], -vec_[2], length_ - vec_[1] * vec_[1] / (length_ + vec_[0]),
                -vec_[2] * vec_[1] / (length_ + vec_[0]), -vec_[2] * vec_[1] / (length_ + vec_[0]),
                length_ - vec_[2] * vec_[2] / (length_ + vec_[0]);
            res /= length_;
        } else {
            res = Eigen::Matrix<double, 3, 2>::Zero();
            res(1, 1) = -1;
            res(2, 0) = 1;
        }
        return res;
    }

    /**
     * S2_Nx: the partial derivative of x.boxminus(y) w.r.t. x, where x and y belong to S2
     * S2_Nx_yy: simplified S2_Nx when x is equal to y
     */
    Eigen::Matrix<double, 2, 3> S2_Nx_yy() const {
        Eigen::Matrix<double, 2, 3> res;
        Eigen::Matrix<double, 3, 2> Bx = S2_Bx();
        res = 1 / length_ / length_ * Bx.transpose() * SO3::hat(vec_);
        return res;
    }

    void oplus(const Vec3d &delta, double scale = 1.0) { vec_ = math::exp(delta, scale * 0.5) * vec_; }

    /**
     * 广义减
     * @param res
     * @param other
     */
    Vec2d boxminus(const S2 &other) const {
        Vec2d res;
        double v_sin = (SO3::hat(vec_) * other.vec_).norm();
        double v_cos = vec_.transpose() * other.vec_;
        double theta = std::atan2(v_sin, v_cos);
        if (v_sin < 1e-5) {
            if (std::fabs(theta) > 1e-5) {
                res[0] = 3.1415926;
                res[1] = 0;
            } else {
                res[0] = 0;
                res[1] = 0;
            }
        } else {
            S2 other_copy = other;
            Eigen::Matrix<double, 3, 2> Bx = other_copy.S2_Bx();
            res = theta / v_sin * Bx.transpose() * SO3::hat(other.vec_) * vec_;
        }
        return res;
    }

    /**
     * 广义加
     * @param delta
     * @param scale
     */
    void boxplus(const Vec2d &delta, double scale = 1) {
        Eigen::Matrix<double, 3, 2> Bx = S2_Bx();
        SO3 res = math::exp(Bx * delta, scale / 2);
        vec_ = res.matrix() * vec_;
    }
};

}  // namespace lightning
