//
// Created by xiang on 25-3-12.
//

#ifndef LIGHTNING_CONSTANT_H
#define LIGHTNING_CONSTANT_H

#include "common/eigen_types.h"

namespace lightning::constant {

/// 各类常量定义
constexpr double kDEG2RAD = 0.017453292519943;   // deg -> rad
constexpr double kRAD2DEG = 57.295779513082323;  // rad -> deg
constexpr double kPI = M_PI;                     // pi
constexpr double kPI_2 = kPI / 2.0;              // pi/2

// 地理参数
constexpr double kGRAVITY = 9.80665;  // g
constexpr double G_m_s2 = 9.806;      // 重力大小

}  // namespace lightning::constant

#endif  // LIGHTNING_CONSTANT_H
