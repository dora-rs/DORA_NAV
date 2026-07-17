//
// Created by xiang on 24-4-26.
//

#include "solver.h"

#include <algorithm>
#include <cstring>

namespace lightning::miao {

Solver::Solver() {}

void Solver::ResizeVector(size_t sx) {
    x_size_ = sx;
    x_.conservativeResize(x_size_, 1);
    b_.conservativeResize(x_size_, 1);
}

Solver::~Solver() {}

}  // namespace lightning::miao