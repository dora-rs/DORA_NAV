//
// Created by xiang on 25-3-12.
//

#ifndef LIGHTNING_SYNC_H
#define LIGHTNING_SYNC_H

#include "common/measure_group.h"
#include "common/point_def.h"
#include "common/std_types.h"

#include <glog/logging.h>

namespace lightning {

/**
 * 将scan和odom同步
 * 类似于ros中的消息同步器，但适用于离线模型，不需要subscriber和publisher
 */
class MessageSync {
   public:
    using Callback = std::function<void(const MeasureGroup&)>;

    MessageSync(Callback cb) : callback_(cb) {}

    /// 处理 scan 数据
    void ProcessScan(CloudPtr scan) {
        UL lock(scan_mutex_);
        double timestamp = math::ToSec(scan->header.stamp);
        if (timestamp < last_timestamp_scan_) {
            scan_buffer_.clear();
        }

        last_timestamp_scan_ = timestamp;
        scan_buffer_.emplace_back(scan);

        Sync();
    }

    /**
     * 处理 imu 消息
     * @param msg
     */
    void ProcessIMU(IMUPtr msg) {
        UL lock(imu_mutex_);
        double timestamp = msg->timestamp;
        if (timestamp < last_timestamp_imu_) {
            LOG(WARNING) << "imu loop back, time difference is " << last_timestamp_imu_ - timestamp;
            return;
        }

        imu_buffer_.emplace_back(msg);
        last_timestamp_imu_ = timestamp;

        while (imu_buffer_.size() >= 500) {
            imu_buffer_.pop_front();
        }
    }
    /**
     * 处理 odom 消息
     * @param msg
     */
    void ProcessOdom(OdomPtr msg) {
        UL lock(odom_mutex_);
        double timestamp = msg->timestamp_;
        if (timestamp < last_timestamp_odom_) {
            LOG(WARNING) << "odom (enc) loop back, time difference is " << last_timestamp_odom_ - timestamp;
        }

        odom_buffer_.emplace_back(msg);
        last_timestamp_odom_ = timestamp;

        while (odom_buffer_.size() >= 500) {
            odom_buffer_.pop_front();
        }
    }

   private:
    /// 尝试同步odom和scan
    inline bool Sync() {
        UL lock_odom(odom_mutex_);
        UL lock(imu_mutex_);
        if (scan_buffer_.empty() || imu_buffer_.empty()) {
            return false;
        }

        MeasureGroup measures = MeasureGroup();                               // 这个要重新构建一个
        measures.scan_raw_ = scan_buffer_.back();                             // 使用最近的scan
        measures.timestamp_ = math::ToSec(measures.scan_raw_->header.stamp);  // 这个给的是结束时间
        double curr_time = math::ToSec(measures.scan_raw_->header.stamp);
        double curr_time_end = curr_time + lo::lidar_time_interval;

        /// 查找最近时刻的imu
        bool found = false;
        double best_dt[2] = {1e9, 1e9};
        IMUPtr best = nullptr;
        OdomPtr best_odom = nullptr;

        // NOTE 插个值更好一些     // NOTE
        // 由于odom精确时间只有底层通讯程序最清楚，期初想把插值工作交给底层（odom发布源），只需要插值后，发出跟激光同一时间戳
        // best_dt > -1
        for (auto& imu : imu_buffer_) {
            double dt = fabs(imu->timestamp - curr_time_end);
            if (dt < best_dt[0]) {
                best_dt[0] = dt;
                best = imu;
            }
        }
        for (auto& odom : odom_buffer_) {
            double dt = fabs(odom->timestamp_ - curr_time_end);
            if (dt < best_dt[1]) {
                best_dt[1] = dt;
                best_odom = odom;
            }
        }

        // &&: refuse to solve difficult logic if one is pass
        if (best_dt[0] < max_time_diff_ && (odom_buffer_.empty() || best_dt[1] < max_time_diff_)) {
            found = true;
        }

        if (!found) {
            // clean the buffers
            while (!imu_buffer_.empty() && imu_buffer_.front()->timestamp < (curr_time_end - 0.1)) {
                imu_buffer_.pop_front();
            }
            while (!odom_buffer_.empty() && odom_buffer_.front()->timestamp_ < (curr_time_end - 0.1)) {
                odom_buffer_.pop_front();
            }
            scan_buffer_.clear();

            return false;
        }

        // clean the buffers
        while (!imu_buffer_.empty()) {
            bool bbreak = imu_buffer_.front() == best;
            measures.imu_.emplace_back(imu_buffer_.front());
            imu_buffer_.pop_front();
            if (bbreak) break;
        }

        while (!odom_buffer_.empty()) {
            bool bbreak = odom_buffer_.front() == best_odom;
            measures.odom_.emplace_back(odom_buffer_.front());
            odom_buffer_.pop_front();
            if (bbreak) break;
        }

        /// 找到了
        lock.unlock();  // callback阶段不需要再锁定IMU
        lock_odom.unlock();
        if (callback_) {
            callback_(measures);
        }

        scan_buffer_.clear();
        return true;
    }

    std::mutex scan_mutex_;
    std::mutex imu_mutex_;
    std::mutex odom_mutex_;
    Callback callback_;                 // 同步数据后的回调函数
    std::deque<CloudPtr> scan_buffer_;  // scan 数据缓冲
    std::deque<IMUPtr> imu_buffer_;     // odom imu cache
    std::deque<OdomPtr> odom_buffer_;   // odom (enc) cache

    double last_timestamp_scan_ = -1.0;  // 最近scan时间
    double last_timestamp_imu_ = 0;      // nearest odom imu timestamp
    double last_timestamp_odom_ = 0;     // nearest odom (enc) timestamp
    const double max_time_diff_ = 0.5;   // 同步之允许的时间误差
};
}  // namespace lightning

#endif  // LIGHTNING_SYNC_H
