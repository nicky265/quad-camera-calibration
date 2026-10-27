// npy_io.cpp —— .npy 解析实现（参考 numpy 格式规范 v1.0/v2.0）
#include <panostitch/npy_io.hpp>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace pan {

namespace {

// 从 header 字典字符串里取 "'key': value" 的值（粗糙但够用）
std::string field(const std::string& header, const std::string& key) {
    auto pos = header.find("'" + key + "'");
    if (pos == std::string::npos) return "";
    pos = header.find(':', pos);
    if (pos == std::string::npos) return "";
    ++pos;
    const auto end = header.find(',', pos);
    const std::string value =
        header.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
    const size_t first = value.find_first_not_of(" \t");
    const size_t last = value.find_last_not_of(" \t");
    return (first == std::string::npos) ? "" : value.substr(first, last - first + 1);
}

// npy 的 header 长度字段固定为小端序，不能直接 memcpy 到 uint16_t/uint32_t。
uint16_t readLe16(std::ifstream& f) {
    unsigned char b[2] = {0, 0};
    f.read(reinterpret_cast<char*>(b), 2);
    return static_cast<uint16_t>(b[0] | (static_cast<uint16_t>(b[1]) << 8));
}

uint32_t readLe32(std::ifstream& f) {
    unsigned char b[4] = {0, 0, 0, 0};
    f.read(reinterpret_cast<char*>(b), 4);
    return static_cast<uint32_t>(b[0]) | (static_cast<uint32_t>(b[1]) << 8) |
           (static_cast<uint32_t>(b[2]) << 16) | (static_cast<uint32_t>(b[3]) << 24);
}

} // namespace

cv::Mat loadNpy(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f.good()) throw std::runtime_error("cannot open npy: " + path);

    char magic[6] = {};
    f.read(magic, 6);
    if (!f || std::memcmp(magic, "\x93NUMPY", 6) != 0) {
        throw std::runtime_error("not a valid npy: " + path);
    }

    unsigned char version[2] = {0, 0};
    f.read(reinterpret_cast<char*>(version), 2);
    if (!f) throw std::runtime_error("npy version truncated: " + path);
    const unsigned char major = version[0];
    if (major < 1 || major > 3) {
        throw std::runtime_error("unsupported npy version " + std::to_string(major) + ": " + path);
    }

    const uint32_t hlen = (major == 1) ? readLe16(f) : readLe32(f);
    if (!f || hlen == 0 || hlen > (1u << 20)) {
        throw std::runtime_error("npy header length invalid: " + path);
    }

    std::string header(hlen, '\0');
    f.read(header.data(), static_cast<std::streamsize>(hlen));
    if (!f) throw std::runtime_error("npy header truncated: " + path);

    const std::string descr = field(header, "descr");
    const std::string fortran = field(header, "fortran_order");
    if (descr.find("f4") == std::string::npos || descr.find('>') != std::string::npos) {
        throw std::runtime_error("npy only supports little-endian float32, got " + descr + ": " +
                                 path);
    }
    if (fortran.find("True") != std::string::npos) {
        throw std::runtime_error("npy fortran_order not supported: " + path);
    }

    // shape: (...) 里的整数
    const std::string shape = field(header, "shape");
    std::vector<int> dims;
    {
        std::stringstream ss(shape);
        char c = 0;
        while (ss >> c) {
            if (c == '(' || c == ',' || c == ' ') continue;
            if (c == ')') break;
            ss.putback(c);
            int dim = 0;
            if (!(ss >> dim)) break;
            dims.push_back(dim);
        }
    }
    if (dims.empty()) throw std::runtime_error("npy shape parse failed: " + path);
    if (dims.size() > 2) throw std::runtime_error("npy only supports 1D/2D arrays: " + path);

    size_t total = 1;
    for (int dim : dims) {
        if (dim <= 0) throw std::runtime_error("npy shape has non-positive dimension: " + path);
        const size_t d = static_cast<size_t>(dim);
        if (total > std::numeric_limits<size_t>::max() / d) {
            throw std::runtime_error("npy shape is too large: " + path);
        }
        total *= d;
    }

    std::vector<float> buf(total);
    f.read(reinterpret_cast<char*>(buf.data()),
           static_cast<std::streamsize>(total * sizeof(float)));
    if (!f) throw std::runtime_error("npy payload incomplete: " + path);

    cv::Mat mat;
    if (dims.size() == 1) {
        mat = cv::Mat(dims[0], 1, CV_32F, buf.data()).clone();
    } else {
        mat = cv::Mat(dims[0], dims[1], CV_32F, buf.data()).clone();
    }
    return mat;
}

} // namespace pan
