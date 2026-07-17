//
// Created by xiang on 25-6-24.
//

#include "core/g2p5/g2p5_subgrid.h"

#include <cstring>
#include <memory>

namespace lightning::g2p5 {

SubGrid::SubGrid(int num_x, int num_y) {
    if (num_x * num_y == 0) {
        grid_data_ = nullptr;
        return;
    }

    grid_data_ = new GridData[num_x * num_y];
}

void SubGrid::MallocGrid() {
    if (grid_data_ != nullptr) {
        return;
    }
    grid_data_ = new GridData[width_2_];
}

SubGrid::~SubGrid() {
    if (grid_data_ != nullptr) {
        delete[] grid_data_;
        grid_data_ = nullptr;
    }
}

SubGrid::SubGrid(const SubGrid &other) {
    if (grid_data_ != nullptr) {
        delete[] grid_data_;
    }

    grid_data_ = nullptr;
    if (other.grid_data_ != nullptr) {
        MallocGrid();
        memcpy(grid_data_, other.grid_data_, sizeof(GridData) * width_2_);
    }
}

SubGrid &SubGrid::operator=(const SubGrid &other) {
    if (this != &other) {
        if (grid_data_ != nullptr) {
            delete[] grid_data_;
        }
        grid_data_ = nullptr;
        if (other.grid_data_ != nullptr) {
            MallocGrid();
            memcpy(grid_data_, other.grid_data_, sizeof(GridData) * width_2_);
        }
    }
    return *this;
}

}  // namespace lightning::g2p5