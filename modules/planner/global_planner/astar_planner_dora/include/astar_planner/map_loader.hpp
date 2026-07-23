#ifndef MAP_LOADER_HPP
#define MAP_LOADER_HPP

#include "astar_algorithm.hpp"
#include <string>

namespace astar_planner {

bool loadPGM(const std::string& filename, MapInfo& map);

bool loadMapYaml(const std::string& yaml_file, MapInfo& map);

MapInfo inflateMap(const MapInfo& map, double inflation_radius);

} // namespace astar_planner

#endif // MAP_LOADER_HPP
