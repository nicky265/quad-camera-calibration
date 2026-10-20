// config.cpp —— config.json 读取实现
#include <panostitch/config.hpp>

#include <fstream>
#include <iostream>
#include "nlohmann/json.hpp"

namespace pan {

Config loadConfig(const std::string& path) {
    Config c;
    std::ifstream f(path);
    if (!f.good()) {
        std::cerr << "[config] cannot open " << path << ", using defaults\n";
        return c;
    }
    try {
        nlohmann::json j;
        f >> j;
        if (j.contains("cameras")) {
            const auto& cam = j["cameras"];
            if (cam.contains("count")) c.count = cam["count"].get<int>();
        }
        if (j.contains("pano")) {
            const auto& p = j["pano"];
            if (p.contains("out_size") && p["out_size"].is_array() && p["out_size"].size() == 2) {
                c.out_w = p["out_size"][0].get<int>();
                c.out_h = p["out_size"][1].get<int>();
            }
            if (p.contains("fov_h_deg"))        c.fov_h_deg        = p["fov_h_deg"].get<double>();
            if (p.contains("focus_distance_m")) c.focus_distance_m = p["focus_distance_m"].get<double>();
            if (p.contains("ground_height_m"))  c.ground_height_m  = p["ground_height_m"].get<double>();
            if (p.contains("far_distance_m"))   c.far_distance_m   = p["far_distance_m"].get<double>();
            if (p.contains("max_ground_m"))     c.max_ground_m     = p["max_ground_m"].get<double>();
        }
        if (j.contains("paths")) {
            const auto& p = j["paths"];
            if (p.contains("calib_root"))  c.calib_root  = p["calib_root"].get<std::string>();
            if (p.contains("frames_root")) c.frames_root = p["frames_root"].get<std::string>();
            if (p.contains("out_root"))    c.out_root    = p["out_root"].get<std::string>();
            if (p.contains("data_root"))   c.data_root   = p["data_root"].get<std::string>();
        }
    } catch (const std::exception& e) {
        std::cerr << "[config] parse failed (" << e.what() << "), using defaults\n";
    }

    // 兼容旧配置：没有新字段时沿用原 Python 工程语义。
    if (c.calib_root.empty())  c.calib_root = c.out_root;
    if (c.frames_root.empty()) c.frames_root = c.data_root + "/preview";
    return c;
}

} // namespace pan
