#ifndef PATH_SMOOTHER_HPP
#define PATH_SMOOTHER_HPP

#include "astar_algorithm.hpp"
#include <vector>

namespace astar_planner {

// ─────────────────────────────────────────────────────────────────────────────
// 可视性检测（Bresenham 光线追踪）
//   在栅格地图上沿 a→b 连线逐格采样，全部可通行则返回 true
// ─────────────────────────────────────────────────────────────────────────────
bool lineOfSight(const GridPoint& a, const GridPoint& b, const MapInfo& map);

// ─────────────────────────────────────────────────────────────────────────────
// Step 1: Shortcut 剪枝（贪心可视性裁剪）
//   从当前锚点出发，尽量跳到最远的可直视路径点；
//   去除所有冗余中间点，消除栅格锯齿。
//   输入/输出均为世界坐标路径。
// ─────────────────────────────────────────────────────────────────────────────
std::vector<WorldPoint> shortcutPath(const std::vector<WorldPoint>& path,
                                     const MapInfo& map);

// ─────────────────────────────────────────────────────────────────────────────
// Step 2: 等弧长重采样
//   沿路径按固定弧长 interval（米）均匀插值，使路径点间距一致；
//   起点和终点始终保留。
// ─────────────────────────────────────────────────────────────────────────────
std::vector<WorldPoint> resamplePath(const std::vector<WorldPoint>& path,
                                     double interval);

// ─────────────────────────────────────────────────────────────────────────────
// 完整平滑流水线：Shortcut → 等弧长重采样
//   resample_interval: 重采样间距（米），默认 0.3m
// ─────────────────────────────────────────────────────────────────────────────
std::vector<WorldPoint> smoothPath(const std::vector<WorldPoint>& raw_path,
                                   const MapInfo& map,
                                   double resample_interval = 0.3);

} // namespace astar_planner

#endif // PATH_SMOOTHER_HPP
