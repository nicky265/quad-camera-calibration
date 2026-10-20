// calib_io.cpp —— lut.bin / weight.bin / yml 读取实现
#include <panostitch/calib_io.hpp>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <vector>
#include <opencv2/imgproc.hpp>

namespace pan {

namespace {

// 读取并校验 24 字节头：magic(4s) ver(u16) dtype(u16) w(i32) h(i32) cam(i32) res(i32)
struct BinHeader {
    char     magic[4];
    uint16_t ver;
    uint16_t dtype;
    int32_t  w;
    int32_t  h;
    int32_t  cam;
    int32_t  res;
};

BinHeader readHeader(std::ifstream& f, const char* expect_magic) {
    BinHeader h{};
    f.read(reinterpret_cast<char*>(&h), sizeof(BinHeader));
    if (!f || std::memcmp(h.magic, expect_magic, 4) != 0) {
        throw std::runtime_error("invalid binary header (magic mismatch)");
    }
    return h;
}

} // namespace

Lut loadLut(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f.good()) throw std::runtime_error("cannot open LUT: " + path);
    BinHeader h = readHeader(f, "PLUT");
    if (h.dtype != 0) throw std::runtime_error("only float32 LUT (dtype=0) supported: " + path);

    Lut lut;
    lut.w = h.w;
    lut.h = h.h;
    const size_t n = static_cast<size_t>(h.w) * static_cast<size_t>(h.h);
    std::vector<float> buf(2 * n);
    f.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(buf.size() * sizeof(float)));
    if (!f) throw std::runtime_error("LUT payload incomplete: " + path);

    // mapx 在前、mapy 在后（与 io_calib.save_lut 一致）
    cv::Mat mapx(h.h, h.w, CV_32F, buf.data());
    cv::Mat mapy(h.h, h.w, CV_32F, buf.data() + n);
    lut.mapx = mapx.clone();
    lut.mapy = mapy.clone();
    return lut;
}

cv::Mat loadWeight(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f.good()) throw std::runtime_error("cannot open weight: " + path);
    BinHeader h = readHeader(f, "PWGT");
    const size_t n = static_cast<size_t>(h.w) * static_cast<size_t>(h.h);
    std::vector<float> buf(n);
    f.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(n * sizeof(float)));
    if (!f) throw std::runtime_error("weight payload incomplete: " + path);
    cv::Mat w(h.h, h.w, CV_32F, buf.data());
    return w.clone();
}

Intrinsics loadIntrinsicsYml(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error("cannot open intrinsics: " + path);
    Intrinsics in;
    fs["K"] >> in.K;
    fs["D"] >> in.D;
    in.K.convertTo(in.K, CV_64F);
    in.D = in.D.reshape(1, 1).clone();
    in.D.convertTo(in.D, CV_64F);
    in.w = static_cast<int>(static_cast<double>(fs["image_width"]));
    in.h = static_cast<int>(static_cast<double>(fs["image_height"]));
    cv::Mat cc;
    fs["circle_center"] >> cc;
    if (!cc.empty()) { in.has_circle = true; in.ccx = cc.at<double>(0, 0); in.ccy = cc.at<double>(0, 1); }
    double cr = 0; fs["circle_radius"] >> cr;
    if (cr > 0) in.cr = cr;
    double fr = 0; fs["fold_radius"] >> fr;
    in.fold_radius = fr;
    return in;
}

Extrinsics loadExtrinsicsYml(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error("cannot open extrinsics: " + path);
    Extrinsics ex;
    fs["R_cw"] >> ex.Rcw;
    fs["T_cw"] >> ex.Tcw;
    ex.Rcw.convertTo(ex.Rcw, CV_64F);
    ex.Tcw = ex.Tcw.reshape(1, 1).clone();          // 统一成 3x1
    ex.Tcw.convertTo(ex.Tcw, CV_64F);
    cv::FileNode n = fs["center_angle_deg"];
    if (!n.empty() && n.isReal()) {
        ex.center_angle_deg = static_cast<double>(n);
        ex.has_angle = true;
    }
    return ex;
}

} // namespace pan
