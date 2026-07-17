#ifndef ASTAR_ALGORITHM_HPP
#define ASTAR_ALGORITHM_HPP

#include <vector>
#include <optional>
#include <cstdint>

namespace astar_planner {

struct GridPoint {
    int x;
    int y;

    GridPoint() : x(0), y(0) {}
    GridPoint(int x_, int y_) : x(x_), y(y_) {}

    bool operator==(const GridPoint& other) const {
        return x == other.x && y == other.y;
    }
};

struct WorldPoint {
    double x;
    double y;

    WorldPoint() : x(0.0), y(0.0) {}
    WorldPoint(double x_, double y_) : x(x_), y(y_) {}
};

struct MapInfo {
    int width;
    int height;
    double resolution;
    double origin_x;
    double origin_y;
    std::vector<uint8_t> data;

    MapInfo() : width(0), height(0), resolution(0.05),
                origin_x(0.0), origin_y(0.0) {}
};

bool isFreeCell(int x, int y, const MapInfo& map);

std::optional<GridPoint> findNearestFreeCell(const GridPoint& start, const MapInfo& map);

std::optional<std::vector<GridPoint>> runAStar(const MapInfo& map,
                                                const GridPoint& start,
                                                const GridPoint& goal);

} // namespace astar_planner

#endif // ASTAR_ALGORITHM_HPP
