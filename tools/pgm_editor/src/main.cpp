#include "pgm_editor.h"
#include <iostream>
#include <string>

void printUsage(const char* program_name) {
    std::cout << "用法: " << program_name << " <pgm_file> [output_file]" << std::endl;
    std::cout << "\n参数:" << std::endl;
    std::cout << "  pgm_file     - 输入的PGM地图文件" << std::endl;
    std::cout << "  output_file  - (可选) 输出的PGM地图文件，默认覆盖原文件" << std::endl;
    std::cout << "\n示例:" << std::endl;
    std::cout << "  " << program_name << " map.pgm" << std::endl;
    std::cout << "  " << program_name << " map.pgm map_edited.pgm" << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "         PGM栅格地图编辑器" << std::endl;
    std::cout << "========================================\n" << std::endl;

    if (argc < 2) {
        printUsage(argv[0]);
        return -1;
    }

    std::string input_file = argv[1];
    std::string output_file = (argc >= 3) ? argv[2] : input_file;

    PGMEditor editor;

    // 加载地图
    if (!editor.loadMap(input_file)) {
        return -1;
    }

    // 运行编辑器
    editor.run();

    // 保存地图
    std::cout << "\n是否保存修改? (y/n): ";
    char choice;
    std::cin >> choice;

    if (choice == 'y' || choice == 'Y') {
        if (editor.saveMap(output_file)) {
            std::cout << "地图已成功保存!" << std::endl;
        } else {
            std::cerr << "保存地图失败" << std::endl;
            return -1;
        }
    } else {
        std::cout << "未保存修改" << std::endl;
    }

    return 0;
}
