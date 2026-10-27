// config.cpp —— config.json 读取实现（OpenCV FileStorage，无第三方 JSON 依赖）
#include <panostitch/config.hpp>

#include <exception>
#include <fstream>
#include <iostream>
#include <string>

#include <opencv2/core.hpp>

namespace pan {

namespace {

// 仅在字段存在且类型匹配时覆盖默认值
void readInt(const cv::FileNode& node, int& dst) {
    if (node.empty() || !(node.isInt() || node.isReal())) return;
    double value = 0.0;
    node >> value;
    dst = static_cast<int>(value);
}

void readDouble(const cv::FileNode& node, double& dst) {
    if (node.empty() || !(node.isInt() || node.isReal())) return;
    node >> dst;
}

void readString(const cv::FileNode& node, std::string& dst) {
    if (node.empty() || !node.isString()) return;
    node >> dst;
}

// 异常消息可能带换行，打印前压成一行。
std::string singleLine(std::string text) {
    while (!text.empty() && (text.back() == '\n' || text.back() == '\r'))
        text.pop_back();
    return text;
}

// 兼容旧配置：没有新字段时沿用原 Python 工程语义。
Config applyPathFallbacks(Config config) {
    if (config.calib_root.empty()) config.calib_root = config.out_root;
    if (config.frames_root.empty()) config.frames_root = config.data_root + "/preview";
    return config;
}

} // namespace

Config loadConfig(const std::string& path) {
    Config config;

    // 先用 ifstream 探测文件是否存在，避免 OpenCV 在缺文件时额外打印 ERROR 日志。
    std::ifstream probe(path);
    if (!probe.good()) {
        std::cerr << "[config] cannot open " << path << ", using defaults\n";
        return applyPathFallbacks(config);
    }
    probe.close();

    try {
        cv::FileStorage fs(path, cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
        if (!fs.isOpened()) {
            std::cerr << "[config] cannot open " << path << ", using defaults\n";
        } else {
            readInt(fs["cameras"]["count"], config.count);

            const cv::FileNode pano = fs["pano"];
            const cv::FileNode out_size = pano["out_size"];
            if (out_size.isSeq() && out_size.size() == 2) {
                readInt(out_size[0], config.out_w);
                readInt(out_size[1], config.out_h);
            }
            readDouble(pano["fov_h_deg"], config.fov_h_deg);
            readDouble(pano["focus_distance_m"], config.focus_distance_m);
            readDouble(pano["ground_height_m"], config.ground_height_m);
            readDouble(pano["far_distance_m"], config.far_distance_m);
            readDouble(pano["max_ground_m"], config.max_ground_m);

            const cv::FileNode paths = fs["paths"];
            readString(paths["calib_root"], config.calib_root);
            readString(paths["frames_root"], config.frames_root);
            readString(paths["out_root"], config.out_root);
            readString(paths["data_root"], config.data_root);
        }
    } catch (const cv::Exception& e) {
        std::cerr << "[config] parse failed (" << singleLine(e.msg) << "), using defaults\n";
    } catch (const std::exception& e) {
        std::cerr << "[config] parse failed (" << e.what() << "), using defaults\n";
    }

    return applyPathFallbacks(config);
}

} // namespace pan
