#pragma once
 

#include "BaseMessage.hpp" // nlohmann/json (需提前下载)

  
// Twist消息（cmd_vel）[1,7](@ref)
struct Twist {
    Vector3 linear;
    Vector3 angular;
    
    Twist() = default;
    Twist(const Vector3& lin, const Vector3& ang) : linear(lin), angular(ang) {}
     
};

 