#include "astar_planner/coordinate_transform.hpp"
#include <cmath>

namespace astar_planner {

WorldPoint gridToWorld(int gx, int gy, const MapInfo& map) {
    WorldPoint wp;
    wp.x = map.origin_x + (gx + 0.5) * map.resolution;
    wp.y = map.origin_y + (map.height - gy - 0.5) * map.resolution;
    return wp;
}

GridPoint worldToGrid(double wx, double wy, const MapInfo& map) {
    GridPoint gp;
    gp.x = static_cast<int>(std::floor((wx - map.origin_x) / map.resolution));
    gp.y = map.height - 1 - static_cast<int>(std::floor((wy - map.origin_y) / map.resolution));
    return gp;
}

} // namespace astar_planner
