//
// Created by xiang on 25-5-6.
//

#ifndef LIGHTNING_SLAM_H
#define LIGHTNING_SLAM_H

#include <atomic>
#include <memory>
#include <string>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"
#include "common/point_def.h"

namespace lightning {

class LaserMapping;  //  lio 前端
class LoopClosing;   // 回环检测

// namespace ui {  // DORA: 不需要 UI
// class PangolinWindow;
// }

namespace g2p5 {
class G2P5;
class G2P5Map;
using G2P5MapPtr = std::shared_ptr<G2P5Map>;
}  // namespace g2p5

/**
 * SLAM 系统调用接口
 */
class SlamSystem {
   public:
    struct Options {
        Options() {}

        bool online_mode_ = true;  // 在线模式，在线模式下会起一些子线程来做异步处理

        bool with_cc_ = true;               // 是否需要带交叉验证
        bool with_gridmap_ = true;          // 是否需要2D栅格
        bool with_loop_closing_ = true;     // 是否需要回环检测
        bool with_visualization_ = true;    // 是否需要可视化UI
        bool with_2dvisualization_ = true;  // 是否需要2D可视化UI

        bool step_on_kf_ = true;  // 是否在关键帧处暂停p
    };

    SlamSystem(Options options);
    ~SlamSystem();

    /// 初始化
    bool Init(const std::string& yaml_path);

    /// 对外部交互接口
    /// 开始建图，输入地图名称
    void StartSLAM(std::string map_name);

    /// 保存地图，默认保存至./data/地图名/ 下方
    void SaveMap(const std::string& path = "");

    /// 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);

    /// 处理点云
    void ProcessLidar(CloudPtr cloud);

    /// 获取G2P5地图
    std::shared_ptr<g2p5::G2P5> GetG2P5() { return g2p5_; }

    /// 获取LIO前端（用于外部获取位姿等信息）
    std::shared_ptr<LaserMapping> GetLIO() { return lio_; }

   private:
    Options options_;
    std::atomic_bool running_ = false;

    std::string map_name_;  // 地图名

    std::shared_ptr<LaserMapping> lio_ = nullptr;       // lio 前端
    std::shared_ptr<LoopClosing> lc_ = nullptr;         // 回环检测
    // std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;  // DORA: 不需要 UI
    std::shared_ptr<g2p5::G2P5> g2p5_ = nullptr;        // 栅格地图

    Keyframe::Ptr cur_kf_ = nullptr;
};
}  // namespace lightning

#endif  // LIGHTNING_SLAM_H
