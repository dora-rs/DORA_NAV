//
// Created by xiang on 24-5-8.
//

#ifndef MISC_H
#define MISC_H

#include <cmath>

namespace lightning::miao {

/**
 * converts a number constant to a double constant at compile time
 * to avoid having to cast everything to avoid warnings.
 **/
inline constexpr double cst(long double v) { return (double)v; }

}  // namespace lightning::miao

#endif  // MISC_H
