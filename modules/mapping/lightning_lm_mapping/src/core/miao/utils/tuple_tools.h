//
// Created by xiang on 24-5-11.
//

#ifndef MIAO_TUPLE_TOOLS_H
#define MIAO_TUPLE_TOOLS_H

#include <tuple>

namespace lightning::miao {

template <typename F, typename T, std::size_t... I>
void tuple_apply_i_(F &&f, T &t, int i, std::index_sequence<I...>) {
    (..., (I == i ? f(std::get<I>(t)) : void()));
}

template <typename F, typename T>
void tuple_apply_i(F &&f, T &t, int i) {
    tuple_apply_i_(f, t, i, std::make_index_sequence<std::tuple_size_v<std::decay_t<T>>>());
}
}  // namespace lightning::miao

#endif  // MIAO_TUPLE_TOOLS_H
