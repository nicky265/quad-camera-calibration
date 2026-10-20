// calib_io.hpp —— 标定产物读取
//
//  · lut.bin   自定义二进制（见 common/io_calib.py 头部格式，magic 'PLUT'）
//  · weight.bin 同上（magic 'PWGT'），负载 W*H float32
//  · intrinsics.yml / extrinsics.yml  OpenCV FileStorage YAML
//
// 与 Python: common/io_calib.py 对应。
#pragma once

#include <string>
#include <opencv2/core.hpp>

namespace pan {

// ---- 二进制 LUT / 权重 -----------------------------------------------------
struct Lut {
    int w = 0, h = 0;
    cv::Mat mapx;   // CV_32F (h,w)，无效像素为 -1
    cv::Mat mapy;   // CV_32F (h,w)
};

// 读取 lut.bin（仅支持 dtype=0，即 float32 mapx/mapy）。
Lut loadLut(const std::string& path);

// 读取 weight.bin，返回 CV_32F (h,w)。
cv::Mat loadWeight(const std::string& path);

// ---- 内外参 yml -----------------------------------------------------------
struct Intrinsics {
    cv::Mat K;                 // 3x3 CV_64F
    cv::Mat D;                 // 1x4 或 4x1 CV_64F（取前 4）
    int  w = 0, h = 0;         // 图像尺寸（宽,高）
    bool has_circle = false;   // 是否有图像圆
    double ccx = 0, ccy = 0, cr = 0;  // 圆心/半径
    double fold_radius = 0.0;  // θd 上限（0=无）
};

struct Extrinsics {
    cv::Mat Rcw;               // 3x3 CV_64F：X_world = Rcw * X_cam + Tcw
    cv::Mat Tcw;               // 3x1 CV_64F（相机光心 = Tcw）
    bool has_angle = false;
    double center_angle_deg = 0.0;
};

Intrinsics loadIntrinsicsYml(const std::string& path);
Extrinsics loadExtrinsicsYml(const std::string& path);

} // namespace pan
