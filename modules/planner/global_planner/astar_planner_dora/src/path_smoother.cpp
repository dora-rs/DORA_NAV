#include "astar_planner/path_smoother.hpp"
#include "astar_planner/coordinate_transform.hpp"
#include <cmath>
#include <iostream>

namespace astar_planner {

// ─────────────────────────────────────────────────────────────────────────────
// 内部工具：世界坐标 → 栅格坐标（复用 coordinate_transform 的逻辑）
// ─────────────────────────────────────────────────────────────────────────────
static GridPoint toGrid(const WorldPoint& wp, const MapInfo& map) {
    return worldToGrid(wp.x, wp.y, map);
}

// ─────────────────────────────────────────────────────────────────────────────
// lineOfSight：Bresenham 光线追踪
//   沿 a→b 在栅格地图上逐格采样，任一格不可通行则返回 false。
//   使用改进的 Bresenham 算法，同时检查对角线穿越时的两个正交格，
//   与 A* 扩展节点时的障碍物判断保持一致。
// ─────────────────────────────────────────────────────────────────────────────
bool lineOfSight(const GridPoint& a, const GridPoint& b, const MapInfo& map) {
    int x0 = a.x, y0 = a.y;
    int x1 = b.x, y1 = b.y;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;

    // 检查起点
    if (!isFreeCell(x0, y0, map)) return false;

    int err = dx - dy;

    while (x0 != x1 || y0 != y1) {
        int e2 = 2 * err;

        // 判断本步是水平、垂直还是对角线移动
        bool step_x = (e2 > -dy);
        bool step_y = (e2 <  dx);

        if (step_x && step_y) {
            // 对角线移动：额外检查两侧的正交格，防止路径穿墙角
            if (!isFreeCell(x0 + sx, y0, map) ||
                !isFreeCell(x0, y0 + sy, map)) {
                return false;
            }
        }

        if (step_x) { err -= dy; x0 += sx; }
        if (step_y) { err += dx; y0 += sy; }

        if (!isFreeCell(x0, y0, map)) return false;
    }

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// shortcutPath：贪心 Shortcut 剪枝
//
//   算法：
//     1. 锚点 anchor = path[0]，加入结果
//     2. 从末尾向前扫描，找到第一个与 anchor 有直视连线的点 target
//     3. 将 target 加入结果，anchor = target，重复直到到达终点
//
//   复杂度：O(n²) 最坏情况，但对典型室内地图路径实测很快。
// ─────────────────────────────────────────────────────────────────────────────
std::vector<WorldPoint> shortcutPath(const std::vector<WorldPoint>& path,
                                     const MapInfo& map) {
    if (path.size() <= 2) return path;

    std::vector<WorldPoint> result;
    result.reserve(path.size());
    result.push_back(path.front());

    size_t anchor = 0;

    while (anchor < path.size() - 1) {
        // 从末尾往前找最远的可直视点
        size_t target = anchor + 1;  // 至少前进一步（保底）

        for (size_t j = path.size() - 1; j > anchor + 1; --j) {
            GridPoint ga = toGrid(path[anchor], map);
            GridPoint gb = toGrid(path[j], map);
            if (lineOfSight(ga, gb, map)) {
                target = j;
                break;
            }
        }

        result.push_back(path[target]);
        anchor = target;
    }

    std::cout << "[Smoother] Shortcut: " << path.size()
              << " → " << result.size() << " points" << std::endl;
    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// resamplePath：等弧长重采样
//
//   算法：
//     沿折线路径累计弧长，每积累 interval 米插入一个新点（线性插值）；
//     起点和终点始终保留。
// ─────────────────────────────────────────────────────────────────────────────
std::vector<WorldPoint> resamplePath(const std::vector<WorldPoint>& path,
                                     double interval) {
    if (path.size() < 2 || interval <= 0.0) return path;

    std::vector<WorldPoint> result;
    result.reserve(path.size());
    result.push_back(path.front());

    // 距离上一个已输出点还差多少才到下一个采样位置
    double remaining_to_next = interval;

    for (size_t i = 1; i < path.size(); i++) {
        double dx = path[i].x - path[i - 1].x;
        double dy = path[i].y - path[i - 1].y;
        double seg_len = std::sqrt(dx * dx + dy * dy);

        if (seg_len < 1e-9) continue;  // 重合点，跳过

        double traveled = 0.0;  // 在当前段上已走过的距离

        // 在当前段内尽可能多地插入采样点
        while (traveled + remaining_to_next <= seg_len - 1e-9) {
            traveled += remaining_to_next;
            double ratio = traveled / seg_len;
            WorldPoint wp;
            wp.x = path[i - 1].x + ratio * dx;
            wp.y = path[i - 1].y + ratio * dy;
            result.push_back(wp);
            remaining_to_next = interval;  // 重置到下一个采样距离
        }

        // 当前段走完后，剩余未累积的距离
        remaining_to_next -= (seg_len - traveled);
    }

    // 始终保留终点
    const WorldPoint& last = path.back();
    const WorldPoint& cur_last = result.back();
    double end_dx = last.x - cur_last.x;
    double end_dy = last.y - cur_last.y;
    if (std::sqrt(end_dx * end_dx + end_dy * end_dy) > 1e-6) {
        result.push_back(last);
    }

    std::cout << "[Smoother] Resample (interval=" << interval << "m): "
              << path.size() << " → " << result.size() << " points" << std::endl;
    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// smoothPath：完整平滑流水线
//   Step 1: Shortcut 剪枝  ── 去除锯齿，直线化
//   Step 2: 等弧长重采样   ── 均匀间距，便于下游跟踪
// ─────────────────────────────────────────────────────────────────────────────
std::vector<WorldPoint> smoothPath(const std::vector<WorldPoint>& raw_path,
                                   const MapInfo& map,
                                   double resample_interval) {
    if (raw_path.size() <= 1) return raw_path;

    std::cout << "[Smoother] Raw path: " << raw_path.size() << " points" << std::endl;

    // Step 1: 可视性剪枝
    std::vector<WorldPoint> pruned = shortcutPath(raw_path, map);

    // Step 2: 等弧长重采样
    std::vector<WorldPoint> smoothed = resamplePath(pruned, resample_interval);

    std::cout << "[Smoother] Final: " << raw_path.size()
              << " → " << smoothed.size() << " points ("
              << static_cast<int>(100.0 * smoothed.size() / raw_path.size())
              << "% of original)" << std::endl;

    return smoothed;
}

} // namespace astar_planner
