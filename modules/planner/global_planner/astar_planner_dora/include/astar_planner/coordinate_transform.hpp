#ifndef COORDINATE_TRANSFORM_HPP
#define COORDINATE_TRANSFORM_HPP

#include "astar_algorithm.hpp"

namespace astar_planner {

WorldPoint gridToWorld(int gx, int gy, const MapInfo& map);

GridPoint worldToGrid(double wx, double wy, const MapInfo& map);

} // namespace astar_planner

#endif // COORDINATE_TRANSFORM_HPP
