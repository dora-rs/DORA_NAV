#include "pgm_editor.h"
#include <iostream>

// 栅格值定义（与map_trans保持一致）
const unsigned char OCCUPIED = 0;    // 占据（黑色，障碍物）
const unsigned char FREE = 128;      // 空闲（灰色，可行区域）

PGMEditor::PGMEditor()
    : window_name_("PGM地图编辑器")
    , mode_(EditMode::BRUSH)
    , brush_size_(5)
    , is_drawing_(false)
    , history_index_(-1)
    , zoom_factor_(1.0f)
    , pan_offset_(0, 0)
    , is_panning_(false)
{
}

PGMEditor::~PGMEditor() {
}

bool PGMEditor::loadMap(const std::string& pgm_file) {
    map_ = cv::imread(pgm_file, cv::IMREAD_GRAYSCALE);

    if (map_.empty()) {
        std::cerr << "错误: 无法加载地图文件 " << pgm_file << std::endl;
        return false;
    }

    std::cout << "成功加载地图: " << pgm_file << std::endl;
    std::cout << "地图尺寸: " << map_.cols << " x " << map_.rows << " 像素" << std::endl;

    // 初始化历史记录
    history_.clear();
    saveHistory();

    return true;
}

bool PGMEditor::saveMap(const std::string& pgm_file) {
    if (map_.empty()) {
        std::cerr << "错误: 没有可保存的地图" << std::endl;
        return false;
    }

    if (!cv::imwrite(pgm_file, map_)) {
        std::cerr << "错误: 保存地图失败 " << pgm_file << std::endl;
        return false;
    }

    std::cout << "地图已保存到: " << pgm_file << std::endl;
    return true;
}

void PGMEditor::saveHistory() {
    // 删除当前索引之后的历史记录
    if (history_index_ < static_cast<int>(history_.size()) - 1) {
        history_.erase(history_.begin() + history_index_ + 1, history_.end());
    }

    // 添加新的历史记录
    history_.push_back(map_.clone());
    history_index_++;

    // 限制历史记录数量
    if (history_.size() > MAX_HISTORY) {
        history_.erase(history_.begin());
        history_index_--;
    }
}

void PGMEditor::undo() {
    if (history_index_ > 0) {
        history_index_--;
        map_ = history_[history_index_].clone();
        updateDisplay();
        std::cout << "撤销操作" << std::endl;
    }
}

void PGMEditor::redo() {
    if (history_index_ < static_cast<int>(history_.size()) - 1) {
        history_index_++;
        map_ = history_[history_index_].clone();
        updateDisplay();
        std::cout << "重做操作" << std::endl;
    }
}

void PGMEditor::drawPixel(int x, int y) {
    if (x < 0 || x >= map_.cols || y < 0 || y >= map_.rows) {
        return;
    }

    // 画笔模式：清除障碍物，设为可行区域（灰色128）
    // 橡皮擦模式：添加障碍物，设为占用（黑色0）
    uchar value = (mode_ == EditMode::BRUSH) ? FREE : OCCUPIED;

    // 绘制圆形画笔
    cv::circle(map_, cv::Point(x, y), brush_size_, cv::Scalar(value), -1);
}

void PGMEditor::drawRectangle(int x1, int y1, int x2, int y2) {
    uchar value = (mode_ == EditMode::BRUSH) ? FREE : OCCUPIED;

    cv::Rect rect(std::min(x1, x2), std::min(y1, y2),
                  std::abs(x2 - x1), std::abs(y2 - y1));

    // 限制在地图范围内
    rect &= cv::Rect(0, 0, map_.cols, map_.rows);

    cv::rectangle(map_, rect, cv::Scalar(value), -1);
}

void PGMEditor::updateDisplay() {
    // 转换为彩色图以便显示
    cv::cvtColor(map_, display_, cv::COLOR_GRAY2BGR);

    // 应用缩放
    cv::Mat zoomed;
    if (zoom_factor_ != 1.0f) {
        cv::resize(display_, zoomed, cv::Size(), zoom_factor_, zoom_factor_, cv::INTER_NEAREST);
    } else {
        zoomed = display_.clone();
    }

    // 应用平移 - 创建一个ROI区域用于显示
    cv::Mat transformed;
    int offset_x = static_cast<int>(pan_offset_.x);
    int offset_y = static_cast<int>(pan_offset_.y);

    // 确保偏移在合理范围内
    offset_x = std::max(-zoomed.cols + 100, std::min(zoomed.cols - 100, offset_x));
    offset_y = std::max(-zoomed.rows + 100, std::min(zoomed.rows - 100, offset_y));
    pan_offset_ = cv::Point2f(offset_x, offset_y);

    // 创建显示画布
    int canvas_width = 1200;
    int canvas_height = 900;
    transformed = cv::Mat(canvas_height, canvas_width, CV_8UC3, cv::Scalar(50, 50, 50));

    // 计算要显示的区域
    int src_x = std::max(0, -offset_x);
    int src_y = std::max(0, -offset_y);
    int dst_x = std::max(0, offset_x);
    int dst_y = std::max(0, offset_y);

    int copy_width = std::min(zoomed.cols - src_x, canvas_width - dst_x);
    int copy_height = std::min(zoomed.rows - src_y, canvas_height - dst_y);

    if (copy_width > 0 && copy_height > 0) {
        cv::Rect src_roi(src_x, src_y, copy_width, copy_height);
        cv::Rect dst_roi(dst_x, dst_y, copy_width, copy_height);
        zoomed(src_roi).copyTo(transformed(dst_roi));
    }

    // 在图像上绘制状态信息
    std::string mode_str;
    cv::Scalar color;

    switch (mode_) {
        case EditMode::BRUSH:
            mode_str = "BRUSH (Clear Obstacles)";
            color = cv::Scalar(0, 255, 0);
            break;
        case EditMode::ERASER:
            mode_str = "ERASER (Add Obstacles)";
            color = cv::Scalar(0, 0, 255);
            break;
        case EditMode::FILL_RECT:
            mode_str = "FILL_RECT";
            color = cv::Scalar(255, 0, 255);
            break;
    }

    // 添加半透明背景
    cv::rectangle(transformed, cv::Point(5, 5), cv::Point(400, 125),
                  cv::Scalar(0, 0, 0), -1);
    cv::rectangle(transformed, cv::Point(5, 5), cv::Point(400, 125),
                  cv::Scalar(255, 255, 255), 2);

    cv::putText(transformed, "Mode: " + mode_str, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
    cv::putText(transformed, "Brush Size: " + std::to_string(brush_size_), cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    cv::putText(transformed, "Zoom: " + std::to_string(static_cast<int>(zoom_factor_ * 100)) + "%",
                cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

    std::string help_text = "Press H for help";
    cv::putText(transformed, help_text, cv::Point(10, 115),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);

    cv::imshow(window_name_, transformed);
}

void PGMEditor::onMouse(int event, int x, int y, int flags, void* userdata) {
    PGMEditor* editor = static_cast<PGMEditor*>(userdata);

    // 转换到原始图像坐标（考虑缩放和平移）
    int map_x = static_cast<int>((x - editor->pan_offset_.x) / editor->zoom_factor_);
    int map_y = static_cast<int>((y - editor->pan_offset_.y) / editor->zoom_factor_);

    if (event == cv::EVENT_LBUTTONDOWN) {
        if (editor->mode_ == EditMode::FILL_RECT) {
            editor->rect_start_ = cv::Point(map_x, map_y);
            editor->is_drawing_ = true;
        } else {
            editor->is_drawing_ = true;
            editor->drawPixel(map_x, map_y);
            editor->updateDisplay();
        }
    } else if (event == cv::EVENT_LBUTTONUP) {
        if (editor->is_drawing_) {
            if (editor->mode_ == EditMode::FILL_RECT) {
                editor->drawRectangle(editor->rect_start_.x, editor->rect_start_.y,
                                     map_x, map_y);
            }
            editor->saveHistory();
            editor->is_drawing_ = false;
            editor->updateDisplay();
        }
    } else if (event == cv::EVENT_MOUSEMOVE) {
        if (editor->is_panning_) {
            // 处理平移
            int dx = x - editor->last_mouse_pos_.x;
            int dy = y - editor->last_mouse_pos_.y;
            editor->pan_offset_.x += dx;
            editor->pan_offset_.y += dy;
            editor->last_mouse_pos_ = cv::Point(x, y);
            editor->updateDisplay();
        } else if (editor->is_drawing_ && editor->mode_ != EditMode::FILL_RECT) {
            editor->drawPixel(map_x, map_y);
            editor->updateDisplay();
        }
    } else if (event == cv::EVENT_RBUTTONDOWN) {
        editor->is_panning_ = true;
        editor->last_mouse_pos_ = cv::Point(x, y);
    } else if (event == cv::EVENT_RBUTTONUP) {
        editor->is_panning_ = false;
    } else if (event == cv::EVENT_MOUSEWHEEL) {
        // 鼠标滚轮缩放
        int delta = cv::getMouseWheelDelta(flags);
        float old_zoom = editor->zoom_factor_;

        if (delta > 0) {
            editor->zoom_factor_ *= 1.2f;
        } else {
            editor->zoom_factor_ /= 1.2f;
        }
        editor->zoom_factor_ = std::max(0.1f, std::min(10.0f, editor->zoom_factor_));

        // 以鼠标位置为中心缩放
        float zoom_ratio = editor->zoom_factor_ / old_zoom;
        editor->pan_offset_.x = x - (x - editor->pan_offset_.x) * zoom_ratio;
        editor->pan_offset_.y = y - (y - editor->pan_offset_.y) * zoom_ratio;

        editor->updateDisplay();
        std::cout << "缩放: " << static_cast<int>(editor->zoom_factor_ * 100) << "%" << std::endl;
    }
}

void PGMEditor::showHelp() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "         PGM地图编辑器 - 帮助" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "\n鼠标操作:" << std::endl;
    std::cout << "  左键拖动      - 绘制/擦除" << std::endl;
    std::cout << "  右键拖动      - 平移视图" << std::endl;
    std::cout << "  滚轮          - 缩放视图" << std::endl;
    std::cout << "\n键盘快捷键:" << std::endl;
    std::cout << "  B            - 切换到画笔模式（清除障碍物）" << std::endl;
    std::cout << "  E            - 切换到橡皮擦模式（添加障碍物）" << std::endl;
    std::cout << "  R            - 切换到矩形填充模式" << std::endl;
    std::cout << "  +/=          - 增加画笔大小" << std::endl;
    std::cout << "  -/_          - 减小画笔大小" << std::endl;
    std::cout << "  Z            - 撤销" << std::endl;
    std::cout << "  Y            - 重做" << std::endl;
    std::cout << "  S            - 保存地图" << std::endl;
    std::cout << "  H            - 显示帮助" << std::endl;
    std::cout << "  ESC/Q        - 退出" << std::endl;
    std::cout << "========================================\n" << std::endl;
}

void PGMEditor::run() {
    if (map_.empty()) {
        std::cerr << "错误: 没有加载地图" << std::endl;
        return;
    }

    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::setMouseCallback(window_name_, onMouse, this);

    showHelp();
    updateDisplay();

    std::string output_file;

    while (true) {
        int key = cv::waitKey(10);

        if (key == 27 || key == 'q' || key == 'Q') {  // ESC or Q
            std::cout << "退出编辑器" << std::endl;
            break;
        } else if (key == 'b' || key == 'B') {
            mode_ = EditMode::BRUSH;
            std::cout << "切换到画笔模式（清除障碍物）" << std::endl;
            updateDisplay();
        } else if (key == 'e' || key == 'E') {
            mode_ = EditMode::ERASER;
            std::cout << "切换到橡皮擦模式（添加障碍物）" << std::endl;
            updateDisplay();
        } else if (key == 'r' || key == 'R') {
            mode_ = EditMode::FILL_RECT;
            std::cout << "切换到矩形填充模式" << std::endl;
            updateDisplay();
        } else if (key == '+' || key == '=') {
            brush_size_ = std::min(50, brush_size_ + 1);
            std::cout << "画笔大小: " << brush_size_ << std::endl;
            updateDisplay();
        } else if (key == '-' || key == '_') {
            brush_size_ = std::max(1, brush_size_ - 1);
            std::cout << "画笔大小: " << brush_size_ << std::endl;
            updateDisplay();
        } else if (key == 'z' || key == 'Z') {
            undo();
        } else if (key == 'y' || key == 'Y') {
            redo();
        } else if (key == 's' || key == 'S') {
            std::cout << "请输入保存文件路径（留空使用原文件）: ";
            std::getline(std::cin, output_file);
            if (output_file.empty()) {
                std::cout << "未保存" << std::endl;
            } else {
                saveMap(output_file);
            }
        } else if (key == 'h' || key == 'H') {
            showHelp();
        }
    }

    cv::destroyAllWindows();
}
