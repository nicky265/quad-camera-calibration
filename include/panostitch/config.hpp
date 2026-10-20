// config.hpp —— 读取 config.json（用 nlohmann/json 单头文件）
//
// 对应 Python: common/config.py 里本项目用到的字段。
// 只保留 C++ 单张合成器需要的部分：相机数、柱面输出尺寸/FOV、视差默认参数。
#pragma once

#include <string>

namespace pan {

struct Config {
    int    count       = 4;        // 相机数
    int    out_w       = 2048;     // 柱面输出宽
    int    out_h       = 512;      // 柱面输出高
    double fov_h_deg   = 360.0;    // 水平覆盖角（度）

    // 视差补偿默认参数（--parallax inmem 且未在命令行覆盖时使用）
    double focus_distance_m = 0.0; // 单距离补偿焦点（米），0=关
    double ground_height_m  = 0.0; // 地面平面高度（米，相机离地），>0 启用
    double far_distance_m   = 0.0; // 地面模式远景距离（米），0=纯旋转
    double max_ground_m     = 30.0;// 地面最大有效距离（米）

    // 路径均相对于 config.json 所在目录；绝对路径则原样使用。
    std::string calib_root;        // 标定产物根目录（空则回落到 out_root）
    std::string frames_root;       // 默认源帧目录（空则回落到 data_root/preview）
    std::string out_root = "output";  // 结果输出根目录
    std::string data_root = "data";   // 旧配置兼容字段
};

// 读取 config.json；文件不存在或解析失败时返回默认值并打印警告。
Config loadConfig(const std::string& path);

} // namespace pan
