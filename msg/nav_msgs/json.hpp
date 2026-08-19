#pragma once

#if __has_include(<nlohmann/json.hpp>)
#include <nlohmann/json.hpp>
#else
#error "nlohmann/json.hpp not found. Install nlohmann-json and add it to the include path."
#endif
