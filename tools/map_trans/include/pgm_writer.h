#ifndef PGM_WRITER_H
#define PGM_WRITER_H

#include "grid_map.h"
#include <string>

// PGM文件写入器类
class PGMWriter {
public:
    // 保存为PGM格式（P5二进制格式）
    static bool savePGM(const GridMap& map, const std::string& filename);

    // 保存YAML元数据文件
    static bool saveYAML(const GridMap& map,
                        const std::string& pgm_filename,
                        const std::string& yaml_filename);
};

#endif // PGM_WRITER_H
