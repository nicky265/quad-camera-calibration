// config.cpp —— config.json 读取实现（OpenCV FileStorage，无第三方 JSON 依赖）
#include <panostitch/config.hpp>

#include <exception>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>

namespace pan {

namespace {

// 仅在字段存在且类型匹配时覆盖默认值
void readInt(const cv::FileNode& node, int& dst) {
    if (node.empty() || !(node.isInt() || node.isReal())) return;
    double v = 0.0;
    node >> v;
    dst = static_cast<int>(v);
}

void readDouble(const cv::FileNode& node, double& dst) {
    if (node.empty() || !(node.isInt() || node.isReal())) return;
    node >> dst;
}

void readString(const cv::FileNode& node, std::string& dst) {
    if (node.empty() || !node.isString()) return;
    node >> dst;
}

std::string singleLine(std::string s) {
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
    return s;
}

} // namespace

Config loadConfig(const std::string& path) {
    Config c;

    std::ifstream probe(path);
    if (!probe.good()) {
        std::cerr << "[config] cannot open " << path << ", using defaults\n";
    } else {
        try {
            cv::FileStorage fs(path, cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
            if (!fs.isOpened()) {
                std::cerr << "[config] cannot open " << path << ", using defaults\n";
            } else {
                readInt(fs["cameras"]["count"], c.count);

                const cv::FileNode pano = fs["pano"];
                const cv::FileNode out_size = pano["out_size"];
                if (out_size.isSeq() && out_size.size() == 2) {
                    readInt(out_size[0], c.out_w);
                    readInt(out_size[1], c.out_h);
                }
                readDouble(pano["fov_h_deg"], c.fov_h_deg);
                readDouble(pano["focus_distance_m"], c.focus_distance_m);
                readDouble(pano["ground_height_m"], c.ground_height_m);
                readDouble(pano["far_distance_m"], c.far_distance_m);
                readDouble(pano["max_ground_m"], c.max_ground_m);

                const cv::FileNode paths = fs["paths"];
                readString(paths["calib_root"], c.calib_root);
                readString(paths["frames_root"], c.frames_root);
                readString(paths["out_root"], c.out_root);
                readString(paths["data_root"], c.data_root);
            }
        } catch (const cv::Exception& e) {
            std::cerr << "[config] parse failed (" << singleLine(e.msg) << "), using defaults\n";
        } catch (const std::exception& e) {
            std::cerr << "[config] parse failed (" << e.what() << "), using defaults\n";
        }
    }

    // 兼容旧配置：没有新字段时沿用原 Python 工程语义。
    if (c.calib_root.empty()) c.calib_root = c.out_root;
    if (c.frames_root.empty()) c.frames_root = c.data_root + "/preview";
    return c;
}

} // namespace pan
