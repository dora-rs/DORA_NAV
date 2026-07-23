#include "astar_planner/astar_algorithm.hpp"
#include <queue>
#include <algorithm>
#include <limits>
#include <cmath>
#include <iostream>

namespace astar_planner {

bool isFreeCell(int x, int y, const MapInfo& map) {
    if (x < 0 || y < 0 || x >= map.width || y >= map.height) {
        return false;
    }
    unsigned char value = map.data[y * map.width + x];

    // MapTrans地图约定：0=障碍物(黑色), 128=未知区域(灰色), 255=可行区域(白色)
    // 膨胀后的地图：0=原始障碍物, 64=膨胀区域, 128=可行区域
    // 因此把大于64的区域当作可行区域
    const int OBSTACLE_THRESHOLD = 64;  // 小于等于64认为是障碍物（包括膨胀区域）
    return value > OBSTACLE_THRESHOLD;
}

std::optional<GridPoint> findNearestFreeCell(const GridPoint& point, const MapInfo& map) {
    if (isFreeCell(point.x, point.y, map)) {
        return point;
    }

    std::vector<bool> visited(map.width * map.height, false);
    std::queue<GridPoint> q;
    q.push(point);
    visited[point.y * map.width + point.x] = true;

    int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!q.empty()) {
        GridPoint cur = q.front();
        q.pop();

        for (int i = 0; i < 8; i++) {
            int nx = cur.x + dxs[i];
            int ny = cur.y + dys[i];

            if (nx < 0 || ny < 0 || nx >= map.width || ny >= map.height) {
                continue;
            }
            if (visited[ny * map.width + nx]) {
                continue;
            }

            visited[ny * map.width + nx] = true;

            if (isFreeCell(nx, ny, map)) {
                return GridPoint(nx, ny);
            }

            q.push(GridPoint(nx, ny));
        }
    }

    return std::nullopt;
}

std::optional<std::vector<GridPoint>> runAStar(const MapInfo& map,
                                                const GridPoint& start,
                                                const GridPoint& goal) {
    const double diag_cost = std::sqrt(2.0);
    int size = map.width * map.height;

    std::vector<double> g_cost(size, std::numeric_limits<double>::infinity());
    std::vector<int> parent(size, -1);
    std::vector<bool> closed(size, false);

    auto idx = [&](int x, int y) -> int {
        return y * map.width + x;
    };

    // 启发函数：欧氏距离（可容许，保证找到最优路径）
    // 8方向栅格 A* 输出的路径受网格约束会产生阶梯折线，
    // 这是正常现象，由下游 smoothPath 负责消除。
    auto heuristic = [&](int x, int y) -> double {
        const double dx = static_cast<double>(x - goal.x);
        const double dy = static_cast<double>(y - goal.y);
        return std::sqrt(dx * dx + dy * dy);
    };

    struct Node {
        int x;
        int y;
        double f;
    };

    struct Compare {
        bool operator()(const Node& a, const Node& b) const {
            return a.f > b.f;
        }
    };

    std::priority_queue<Node, std::vector<Node>, Compare> open_list;

    int start_idx = idx(start.x, start.y);
    g_cost[start_idx] = 0.0;
    open_list.push(Node{start.x, start.y, heuristic(start.x, start.y)});

    std::cout << "Starting A* search..." << std::endl;

    int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!open_list.empty()) {
        Node cur = open_list.top();
        open_list.pop();

        int cur_idx = idx(cur.x, cur.y);

        if (closed[cur_idx]) {
            continue;
        }
        closed[cur_idx] = true;

        if (cur.x == goal.x && cur.y == goal.y) {
            std::cout << "Path found!" << std::endl;

            std::vector<GridPoint> path;
            int back_idx = cur_idx;
            while (back_idx != -1) {
                int bx = back_idx % map.width;
                int by = back_idx / map.width;
                path.push_back(GridPoint(bx, by));
                back_idx = parent[back_idx];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int i = 0; i < 8; i++) {
            int nx = cur.x + dxs[i];
            int ny = cur.y + dys[i];

            if (nx < 0 || ny < 0 || nx >= map.width || ny >= map.height) {
                continue;
            }

            if (!isFreeCell(nx, ny, map)) {
                continue;
            }

            // 对角线移动检查：确保对角线路径不会穿过障碍物
            if (i >= 4) {  // 对角线移动 (i=4,5,6,7)
                // 检查两个相邻的正交方向是否都可通行
                int check_x1 = cur.x + dxs[i];
                int check_y1 = cur.y;
                int check_x2 = cur.x;
                int check_y2 = cur.y + dys[i];

                if (!isFreeCell(check_x1, check_y1, map) ||
                    !isFreeCell(check_x2, check_y2, map)) {
                    continue;  // 对角线路径被阻挡
                }
            }

            int neighbor_idx = idx(nx, ny);

            if (closed[neighbor_idx]) {
                continue;
            }

            double move_cost = (i < 4) ? 1.0 : diag_cost;
            double tentative_g = g_cost[cur_idx] + move_cost;

        if (tentative_g < g_cost[neighbor_idx] - 1e-9) {
                g_cost[neighbor_idx] = tentative_g;
                parent[neighbor_idx] = cur_idx;
                double f = tentative_g + heuristic(nx, ny);
                open_list.push(Node{nx, ny, f});
            }
        }
    }

    std::cerr << "No path found!" << std::endl;
    return std::nullopt;
}

} // namespace astar_planner
