//
// Created by xiang on 24-3-19.
//

#ifndef MIAO_MACROS_H
#define MIAO_MACROS_H

namespace lightning::miao {

/// 禁止拷贝与复制
#define DISALLOW_COPY(T)  \
   public:                \
    T(const T&) = delete; \
    T& operator=(const T&) = delete;

constexpr int invalid_id = -2;

}  // namespace lightning::miao

#endif  // MIAO_MACROS_H
