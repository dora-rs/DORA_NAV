//
// Created by xiang on 25-6-23.
//

#ifndef LIGHTNING_G2P5_H
#define LIGHTNING_G2P5_H

#include "common/eigen_types.h"
#include "common/keyframe.h"
#include "core/g2p5/g2p5_map.h"
#include "core/system/async_message_process.h"

#include <thread>

namespace lightning::g2p5 {

/**
 * 地图渲染部分+3转2的部分
 *
 * g2p5 是一个2.5D地图模块，它计算从雷达中心点出发，打到障碍物部分的射线，然后进行occupancy map投影。
 * 可以很方便地将g2p5转换成opencv或者ros格式的地图进行发布和存储
 *
 * g2p5假设雷达是水平安装的。在雷达水平面以下，存在一个地面的平面（floor），地面的高度应该低于雷达安装高度
 * 这个地面参数可以动态估计，也可以事先指定
 * 比如，我们默认雷达水平面为z=0，雷达安装在1m的高度上，那么地面默认为z=-1.0
 * 在地面上方一定区间内的点云才会参与栅格地图的计算，默认取 (min_floor, max_floor)
 * 之间的点云，比如地面以上的0.5至1.0米的障碍物 那么取 min_floor = 0.5, max_floor = 1.0
 *
 * 如果想过滤掉天花板，应该将max_floor适当设小一点
 *
 * 如果是这个区间的障碍物，g2p5会模拟一条从雷达高度的点射向障碍物点，并计算沿途的射线高度
 * 如果栅格中的物体高度低于此射线，则不会被刷新；如果高于此射线，则会被刷白
 *
 * 在lightning-lm中，栅格地图仅用于显示和存储（输出给导航），并不用于定位，所以默认不需要过高的分辨率
 *
 * 地图渲染主要有两个步骤组成：
 * 1. 前端在计算完关键帧时，会立即更新到最新的地图上，此时应该发布最新的地图
 * 2. 后端在发现回环时，触发一次地图重绘过程。由于重绘时间较长，需要在重绘完成后再放到1,
 * 此时1仍然是活跃的，仍然会发布地图
 *
 * g2p5的3转2目前角分辨率精度较低，但速度很快，正常情况一帧点云应该在10ms以内
 */
class G2P5 {
   public:
    struct Options {
        Options() {}

        bool online_mode_ = true;  // 是否为在线模式
        bool esti_floor_ = false;  // 是否需要估计地面

        double lidar_height_ = 0.0;  // 雷达的安装高度

        double default_floor_height_ = -1.0;  // 默认的地面高度(通常在雷达下方)
        double min_th_floor_ = 0.5;           /// 距离地面这个高的障碍物讲被扫入栅格地图中
        double max_th_floor_ = 1.2;           /// 距离地面这个高的障碍物讲被扫入栅格地图中
        float usable_scan_range_ = 50.0;      // 用于计算栅格地图的最远障碍物距离

        double grid_map_resolution_ = 0.1;  // 栅格地图的分辨率，室外建议为0.1，室内可以用0.05

        bool verbose_ = true;
    };

    explicit G2P5(Options options = Options()) : options_(options) {}
    ~G2P5();

    using MapUpdateCallback = std::function<void(G2P5MapPtr map)>;

    /// 从yaml中读取配置信息
    void Init(std::string yaml_path);

    /// 当地图发生改变时，可以向外发布，在这里设置它的回调
    void SetMapUpdateCallback(MapUpdateCallback func) { map_update_cb_ = func; }

    /// 退出渲染器
    void Quit();

    /// 增加一个关键帧，该关键帧会放到关键帧队列中
    void PushKeyframe(Keyframe::Ptr kf);

    /// 触发一次重绘，如果程序正在重绘过程中，再次触发会导致当前重绘过程停止
    void RedrawGlobalMap();

    /// 获取最新的地图
    G2P5MapPtr GetNewestMap();

    /// 设置是否需要最快渲染速度
    bool SetParallelRendering(bool enable_parallel = false) {
        parallel_render_ = enable_parallel;
        return true;
    }

    /// 查看是否仍然有没绘制完的关键帧
    bool IsBusy() { return is_busy_; }

   private:
    /// 在已有的map上增加一些关键帧
    bool AddKfToMap(const std::vector<Keyframe::Ptr> &kfs, G2P5MapPtr &map);

    /// 对地图进行缩放
    bool ResizeMap(const std::vector<Keyframe::Ptr> &kfs, G2P5MapPtr &map);

    /// 渲染前端的地图
    void RenderFront(Keyframe::Ptr kf);

    /// 渲染后端的地图
    void RenderBack();

    /// 3D 点云转2D scan
    void Convert3DTo2DScan(Keyframe::Ptr kf, G2P5MapPtr &map);

    /// 检测地面参数
    bool DetectPlaneCoeffs(Keyframe::Ptr kf);

    void SetWhitePoints(const std::vector<Vec2d> &ang_distance_height, Keyframe::Ptr kf, G2P5MapPtr &map);

    Options options_;

    std::atomic_bool parallel_render_ = false;  // 是否需要并行渲染
    std::atomic_bool is_busy_ = false;          // 是否仍有未渲染完的
    MapUpdateCallback map_update_cb_;           // 地图更新的回调函数
    std::atomic_bool quit_flag_ = false;        // 退出标记位

    std::mutex kf_mutex_;
    std::vector<Keyframe::Ptr> all_keyframes_;  // 全部关键帧

    std::mutex newest_map_mutex_;
    G2P5MapPtr newest_map_ = nullptr;

    /// 前端相关
    std::mutex frontend_mutex_;                                         // 前端锁
    G2P5MapPtr frontend_map_ = nullptr;                                 // 前端最新绘制的地图
    Keyframe::Ptr frontend_current_ = nullptr;                          // 前端正在绘制的关键帧
    sys::AsyncMessageProcess<Keyframe::Ptr> draw_frontend_map_thread_;  // 前端绘制地图的线程

    /// 后端相关
    std::thread draw_backend_map_thread_;           // 后端重绘的线程
    std::atomic_bool backend_redraw_flag_ = false;  // 后端重绘的flag
    G2P5MapPtr backend_map_ = nullptr;              // 正在绘制的地图

    Vec4d floor_coeffs_ = Vec4d(0, 0, 1.0, 1.0);  // 地面方程参数
};

}  // namespace lightning::g2p5

#endif  // LIGHTNING_G2P5_H
