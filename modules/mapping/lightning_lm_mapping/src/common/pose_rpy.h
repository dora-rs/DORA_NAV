//
// Created by xiang on 25-7-7.
//

#ifndef LIGHTNING_POSE_RPY_H
#define LIGHTNING_POSE_RPY_H

namespace lightning{

/// 以欧拉角表达的Pose
template <typename T>
struct PoseRPY {
    PoseRPY() = default;
    PoseRPY(T xx, T yy, T zz, T r, T p, T y) : x(xx), y(yy), z(zz), roll(r), pitch(p), yaw(y) {}
    T x     = 0;
    T y     = 0;
    T z     = 0;
    T roll  = 0;
    T pitch = 0;
    T yaw   = 0;
};

using PoseRPYD  = PoseRPY<double>;
using PoseRPYF = PoseRPY<float>;
}

#endif  // LIGHTNING_POSE_RPY_H
