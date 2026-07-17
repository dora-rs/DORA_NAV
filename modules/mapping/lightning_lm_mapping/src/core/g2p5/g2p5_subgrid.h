//
// Created by xiang on 25-6-24.
//

#ifndef LIGHTNING_G2P5_SUBGRID_H
#define LIGHTNING_G2P5_SUBGRID_H

#include "common/std_types.h"
#include "core/g2p5/g2p5_grid_data.h"

namespace lightning::g2p5 {

/// 子网格定义，坐标系与MapToGrid类似
class SubGrid {
   public:
    static inline constexpr int SUB_GRID_SIZE = 4;

    SubGrid(int num_x = 0, int num_y = 0);

    ~SubGrid();

    SubGrid(const SubGrid &other);
    SubGrid &operator=(const SubGrid &other);

    /// 设置某个网格的占据或者非占据
    void SetGridHitPoint(bool hit, int sub_xi, int sub_yi, float height) {
        UL lock(data_mutex_);
        if (grid_data_ == nullptr) {
            MallocGrid();
        }

        int index = sub_xi + sub_yi * width_;
        GridData &cell = grid_data_[index];

        if (hit) {
            // 占据，黑色
            if (height < cell.height_) {
                cell.height_ = height;
                cell.hit_cnt_ += 1;
                cell.visit_cnt_ += 1;
            }
        } else {
            // 非占据，白色
            if (cell.hit_cnt_ > 3) {
                // 本身为黑色，黑色刷白有高度要求
                if (height < cell.height_) {
                    cell.visit_cnt_ += 1;
                }
            } else {
                cell.visit_cnt_ += 1;
                cell.height_ = height;
            }
        }
    }

    void RemoveCarNoise(int sub_xi, int sub_yi) {
        UL lock(data_mutex_);
        if (grid_data_ == nullptr) {
            MallocGrid();
        }
        int index = sub_xi + sub_yi * width_;
        grid_data_[index].visit_cnt_ += 4;
        grid_data_[index].hit_cnt_ = 0;
    }

    /// 判定数据是否为空
    bool IsEmpty() {
        UL lock(data_mutex_);
        return grid_data_ == nullptr;
    }

    /// 获取hit cnt和visit cnt
    void GetHitAndVisit(int sx, int sy, unsigned int &hit_cnt, unsigned int &visit_cnt) {
        UL lock(data_mutex_);
        GridData &d = grid_data_[sx + sy * width_];

        hit_cnt = d.hit_cnt_;
        visit_cnt = d.visit_cnt_;
    }

   private:
    // 分配这个grid的内存
    void MallocGrid();

    /// 子网格的大小
    static inline constexpr int width_ = (1 << SUB_GRID_SIZE);
    static inline constexpr int width_2_ = (1 << SUB_GRID_SIZE) * (1 << SUB_GRID_SIZE);

    std::mutex data_mutex_;
    GridData *grid_data_ = nullptr;
};

}  // namespace lightning::g2p5

#endif  // LIGHTNING_G2P5_SUBGRID_H
