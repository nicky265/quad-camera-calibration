// calib_io.cpp —— lut.bin / weight.bin / yml 读取实现
#include <panostitch/calib_io.hpp>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace pan {

namespace {

// 24 字节头：magic(4s) ver(u16) dtype(u16) w(i32) h(i32) cam(i32) res(i32)
struct BinHeader {
    char magic[4];
    uint16_t ver;
    uint16_t dtype;
    int32_t w;
    int32_t h;
    int32_t cam;
    int32_t res;
};
static_assert(sizeof(BinHeader) == 24, "binary header layout must stay 24 bytes");

BinHeader readHeader(std::ifstream& f, const char* expect_magic, const char* kind) {
    BinHeader h{};
    f.read(reinterpret_cast<char*>(&h), sizeof(BinHeader));
    if (!f || std::memcmp(h.magic, expect_magic, 4) != 0) {
        throw std::runtime_error(std::string("invalid ") + kind + " header (magic mismatch)");
    }
    if (h.w <= 0 || h.h <= 0) {
        throw std::runtime_error(std::string("invalid ") + kind + " header (bad size)");
    }
    return h;
}

size_t pixelCount(const BinHeader& h, const char* kind) {
    const size_t w = static_cast<size_t>(h.w);
    const size_t rows = static_cast<size_t>(h.h);
    if (w > std::numeric_limits<size_t>::max() / rows) {
        throw std::runtime_error(std::string("invalid ") + kind + " header (size overflow)");
    }
    return w * rows;
}

void requireFloat32(const BinHeader& h, const char* kind, const std::string& path) {
    if (h.dtype != 0) {
        throw std::runtime_error(std::string("only float32 ") + kind +
                                 " (dtype=0) supported: " + path);
    }
}

} // namespace

Lut loadLut(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f.good()) throw std::runtime_error("cannot open LUT: " + path);

    const BinHeader h = readHeader(f, "PLUT", "LUT");
    requireFloat32(h, "LUT", path);
    const size_t n = pixelCount(h, "LUT");
    if (n > std::numeric_limits<size_t>::max() / 2) {
        throw std::runtime_error("LUT is too large: " + path);
    }

    std::vector<float> buf(2 * n);
    f.read(reinterpret_cast<char*>(buf.data()),
           static_cast<std::streamsize>(buf.size() * sizeof(float)));
    if (!f) throw std::runtime_error("LUT payload incomplete: " + path);

    // mapx 在前、mapy 在后（与 io_calib.save_lut 一致）
    Lut lut;
    lut.w = h.w;
    lut.h = h.h;
    lut.mapx = cv::Mat(h.h, h.w, CV_32F, buf.data()).clone();
    lut.mapy = cv::Mat(h.h, h.w, CV_32F, buf.data() + n).clone();
    return lut;
}

cv::Mat loadWeight(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f.good()) throw std::runtime_error("cannot open weight: " + path);

    const BinHeader h = readHeader(f, "PWGT", "weight");
    requireFloat32(h, "weight", path);
    const size_t n = pixelCount(h, "weight");

    std::vector<float> buf(n);
    f.read(reinterpret_cast<char*>(buf.data()),
           static_cast<std::streamsize>(buf.size() * sizeof(float)));
    if (!f) throw std::runtime_error("weight payload incomplete: " + path);
    return cv::Mat(h.h, h.w, CV_32F, buf.data()).clone();
}

Intrinsics loadIntrinsicsYml(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error("cannot open intrinsics: " + path);

    Intrinsics in;
    fs["K"] >> in.K;
    fs["D"] >> in.D;
    if (in.K.rows != 3 || in.K.cols != 3) {
        throw std::runtime_error("invalid intrinsics K (expect 3x3): " + path);
    }
    if (in.D.empty()) throw std::runtime_error("missing intrinsics D: " + path);

    in.K.convertTo(in.K, CV_64F);
    in.D = in.D.reshape(1, 1).clone();
    in.D.convertTo(in.D, CV_64F);
    if (in.D.total() < 4) {
        throw std::runtime_error("invalid intrinsics D (expect >= 4 coefficients): " + path);
    }

    const cv::FileNode image_width = fs["image_width"];
    const cv::FileNode image_height = fs["image_height"];
    if (!image_width.empty()) in.w = static_cast<int>(static_cast<double>(image_width));
    if (!image_height.empty()) in.h = static_cast<int>(static_cast<double>(image_height));
    if (in.w <= 0 || in.h <= 0) {
        throw std::runtime_error("invalid intrinsics image size: " + path);
    }

    cv::Mat circle_center;
    fs["circle_center"] >> circle_center;
    if (!circle_center.empty()) {
        circle_center.convertTo(circle_center, CV_64F);
        circle_center = circle_center.reshape(1, 1);
        if (circle_center.total() >= 2) {
            in.has_circle = true;
            in.ccx = circle_center.at<double>(0, 0);
            in.ccy = circle_center.at<double>(0, 1);
        }
    }

    const cv::FileNode circle_radius = fs["circle_radius"];
    if (!circle_radius.empty()) {
        double radius = 0.0;
        circle_radius >> radius;
        if (radius > 0.0) in.cr = radius;
    }

    const cv::FileNode fold_radius = fs["fold_radius"];
    if (!fold_radius.empty()) fold_radius >> in.fold_radius;
    return in;
}

Extrinsics loadExtrinsicsYml(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error("cannot open extrinsics: " + path);

    Extrinsics ex;
    fs["R_cw"] >> ex.Rcw;
    fs["T_cw"] >> ex.Tcw;
    if (ex.Rcw.rows != 3 || ex.Rcw.cols != 3) {
        throw std::runtime_error("invalid extrinsics R_cw (expect 3x3): " + path);
    }
    if (ex.Tcw.total() != 3) {
        throw std::runtime_error("invalid extrinsics T_cw (expect 3 values): " + path);
    }

    ex.Rcw.convertTo(ex.Rcw, CV_64F);
    ex.Tcw = ex.Tcw.reshape(1, 1).clone(); // 统一成 1x3 行向量
    ex.Tcw.convertTo(ex.Tcw, CV_64F);

    const cv::FileNode angle = fs["center_angle_deg"];
    if (!angle.empty() && (angle.isReal() || angle.isInt())) {
        ex.center_angle_deg = static_cast<double>(angle);
        ex.has_angle = true;
    }
    return ex;
}

} // namespace pan
