#ifndef PGM_EDITOR_H
#define PGM_EDITOR_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

enum class EditMode {
    BRUSH,      // 画笔模式：清除障碍物（设置为白色/空闲）
    ERASER,     // 橡皮擦模式：添加障碍物（设置为黑色/占用）
    FILL_RECT   // 矩形填充模式
};

class PGMEditor {
public:
    PGMEditor();
    ~PGMEditor();

    // 加载PGM地图
    bool loadMap(const std::string& pgm_file);

    // 保存PGM地图
    bool saveMap(const std::string& pgm_file);

    // 运行编辑器GUI
    void run();

private:
    // OpenCV回调函数
    static void onMouse(int event, int x, int y, int flags, void* userdata);

    // 绘制像素
    void drawPixel(int x, int y);

    // 绘制矩形
    void drawRectangle(int x1, int y1, int x2, int y2);

    // 更新显示
    void updateDisplay();

    // 历史记录管理
    void saveHistory();
    void undo();
    void redo();

    // 显示帮助信息
    void showHelp();

private:
    cv::Mat map_;                           // 原始地图（灰度图）
    cv::Mat display_;                       // 显示用的彩色图
    std::string window_name_;               // 窗口名称

    EditMode mode_;                         // 当前编辑模式
    int brush_size_;                        // 画笔大小（像素）

    bool is_drawing_;                       // 是否正在绘制
    cv::Point rect_start_;                  // 矩形起点

    std::vector<cv::Mat> history_;          // 历史记录栈
    int history_index_;                     // 当前历史索引
    static const int MAX_HISTORY = 50;      // 最大历史记录数

    float zoom_factor_;                     // 缩放因子
    cv::Point2f pan_offset_;                // 平移偏移
    bool is_panning_;                       // 是否正在平移
    cv::Point last_mouse_pos_;              // 上次鼠标位置
};

#endif // PGM_EDITOR_H
