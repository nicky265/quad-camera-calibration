// npy_io.cpp —— .npy 解析实现（参考 numpy 格式规范 v1.0/v2.0）
#include <panostitch/npy_io.hpp>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace pan {

namespace {
// 从 header 字典字符串里取 "'key': value" 的值（粗糙但够用）
std::string field(const std::string& h, const std::string& key) {
    auto p = h.find("'" + key + "'");
    if (p == std::string::npos) return "";
    p = h.find(':', p);
    if (p == std::string::npos) return "";
    ++p;
    auto q = h.find_first_of(",", p);
    std::string v = h.substr(p, q == std::string::npos ? std::string::npos : q - p);
    // 去空白
    size_t a = v.find_first_not_of(" \t");
    size_t b = v.find_last_not_of(" \t");
    return (a == std::string::npos) ? "" : v.substr(a, b - a + 1);
}
} // namespace

cv::Mat loadNpy(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f.good()) throw std::runtime_error("cannot open npy: " + path);

    char magic[6];
    f.read(magic, 6);
    if (!f || std::memcmp(magic, "\x93NUMPY", 6) != 0)
        throw std::runtime_error("not a valid npy: " + path);

    uint8_t major = 0, minor = 0;
    f.read(reinterpret_cast<char*>(&major), 1);
    f.read(reinterpret_cast<char*>(&minor), 1);

    uint32_t hlen = 0;
    if (major == 1) {
        uint16_t l = 0; f.read(reinterpret_cast<char*>(&l), 2); hlen = l;
    } else {
        f.read(reinterpret_cast<char*>(&hlen), 4);
    }
    std::string header(hlen, '\0');
    f.read(&header[0], hlen);
    if (!f) throw std::runtime_error("npy header truncated: " + path);

    std::string descr = field(header, "descr");
    std::string fpath = field(header, "fortran_order");
    if (descr.find("f4") == std::string::npos)
        throw std::runtime_error("npy only supports float32, got " + descr + ": " + path);
    if (fpath.find("True") != std::string::npos)
        throw std::runtime_error("npy fortran_order not supported: " + path);

    // shape: (...) 里的整数
    std::string shp = field(header, "shape");
    std::vector<int> dims;
    {
        std::stringstream ss(shp);
        char c;
        while (ss >> c) {
            if (c == '(' || c == ',' || c == ' ') continue;
            if (c == ')') break;
            ss.putback(c);
            int d; ss >> d; dims.push_back(d);
        }
    }
    if (dims.empty()) throw std::runtime_error("npy shape parse failed: " + path);

    size_t total = 1;
    for (int d : dims) total *= static_cast<size_t>(d);
    std::vector<float> buf(total);
    f.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(total * sizeof(float)));
    if (!f) throw std::runtime_error("npy payload incomplete: " + path);

    if (dims.size() == 1) {
        cv::Mat m(dims[0], 1, CV_32F, buf.data());
        return m.clone();
    }
    cv::Mat m(dims[0], dims[1], CV_32F, buf.data());
    return m.clone();
}

} // namespace pan
