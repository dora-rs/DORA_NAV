//
// Created by xiang on 25-3-12.
//

#ifndef LIGHTNING_STD_TYPES_H
#define LIGHTNING_STD_TYPES_H

#include <mutex>
#include <thread>

namespace lightning {

using UL = std::unique_lock<std::mutex>;

}

#endif  // LIGHTNING_STD_TYPES_H
