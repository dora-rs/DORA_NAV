//
// Created by xiang on 23-10-12.
//
#pragma once

#include "common/eigen_types.h"

namespace lightning {

/// 功能点定义
struct FunctionalPoint {
    FunctionalPoint() {}
    FunctionalPoint(const std::string& name, const SE3& pose) : name_(name), pose_(pose) {}

    std::string name_;
    SE3 pose_;
};

}  // namespace lightning
